//! Prospector 兼容的键盘状态广播（`status-adv` Cargo feature）。
//!
//! 协议与调度常量的权威定义在 `rmk-status-adv` crate（与 Prospector
//! `status_advertisement.h` 逐字节兼容）；本模块只负责 RMK 侧的无线电执行：
//!
//! - **MODE 1（未连接、RMK 正在广播 HID）**：把 26 字节载荷注入自身广播的
//!   SCAN_RSP。连接循环按状态刷新节奏重新拉起广播并读取最新载荷；
//! - **MODE 2（已连接 / USB 活跃 / 广播停止）**：独立 non-connectable +
//!   scannable 广播集，ADV 携带 Flags + manufacturer 载荷，SCAN_RSP 携带设备名；
//! - **MODE 3（connectable 代理广播）**：预留给 RMK dongle，本版本不启用。
//!
//! 分体 peripheral 走 `split::ble::peripheral` 的独立传输路径，从不运行本
//! 模块，因此天然静默（与 Prospector 的行为一致）。
//!
//! 广播节奏与 Prospector 一致：active 1000ms / idle 30000ms（5000ms 无活动
//! 判定进入 idle）；关键状态变化触发 5×15ms 突发；分体 central 等待外设接入
//! 期间采用 burst/silent 200ms/1800ms 时间分片，把 radio 让给 scan/connect。

use core::cell::Cell;

use bt_hci::cmd::le::{LeSetAdvData, LeSetAdvParams, LeSetAdvEnable, LeSetScanResponseData};
use bt_hci::controller::ControllerCmdSync;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::BlockingMutex;
use embassy_time::{Duration, Instant, Timer};
use trouble_host::prelude::*;

#[cfg(feature = "split")]
use crate::split::driver::peripheral_connected_count;

// ---------------------------------------------------------------------------
// 与 rmk-status-adv crate 保持一致的调度参数（crate 是协议与调度的权威定义，
// 此处仅镜像常量以避免 rmk 反向依赖）。
// ---------------------------------------------------------------------------

/// 载荷长度（26 字节 manufacturer data，含 0xFFFF/0xABCD 前缀）。
pub const PAYLOAD_LEN: usize = 26;
/// MODE 1 SCAN_RSP 的 AD structure 长度（len + type + 载荷）。
pub const SCAN_RSP_AD_LEN: usize = 28;
/// 活动态刷新间隔（ms）。
const ACTIVE_UPDATE_INTERVAL_MS: u64 = 1_000;
/// 空闲态刷新间隔（ms）。
const IDLE_UPDATE_INTERVAL_MS: u64 = 30_000;
/// 无活动判定超时（ms）。
const ACTIVITY_TIMEOUT_MS: u64 = 5_000;
/// 突发帧数。
const BURST_COUNT: u8 = 5;
/// 突发帧间隔（ms）。
const BURST_INTERVAL_MS: u64 = 15;
/// 分体未全连时的广播窗口（ms）。
const SPLIT_PARTIAL_BURST_MS: u64 = 200;
/// 分体未全连时的静默窗口（ms）。
const SPLIT_PARTIAL_SILENT_MS: u64 = 1_800;

/// MODE 2 广播间隔（对应 Prospector 的 100-150ms fast interval）。
const MODE2_ADV_INTERVAL: Duration = Duration::from_millis(120);

// ---------------------------------------------------------------------------
// 共享状态（全部通过 BlockingMutex + Cell 访问，无锁内嵌套，无死锁风险）。
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct StatusAdvState {
    /// 固件是否已提交载荷（未提交则整个 feature 不生效，行为与原生 RMK 一致）。
    payload_valid: bool,
    /// 连接循环是否正在广播（MODE 1 生效；为 true 时本任务不开独立广播集）。
    piggyback: bool,
    /// 剩余突发帧数。
    burst_remaining: u8,
    /// 上次活动（突发/刷新）时间（`Instant::now().as_millis()`）。
    last_activity_ms: u64,
}

static STATE: BlockingMutex<CriticalSectionRawMutex, Cell<StatusAdvState>> = BlockingMutex::new(
    Cell::new(StatusAdvState {
        payload_valid: false,
        piggyback: false,
        burst_remaining: 0,
        last_activity_ms: 0,
    }),
);

/// 最新 26 字节载荷。
static PAYLOAD: BlockingMutex<CriticalSectionRawMutex, Cell<[u8; PAYLOAD_LEN]>> =
    BlockingMutex::new(Cell::new([0u8; PAYLOAD_LEN]));

/// MODE 1 的 SCAN_RSP AD structure（`[0x1B][0xFF][26B]`），随 `set_payload`
/// 预编码，连接循环每次 advertise 时拷贝。
static PIGGYBACK_SD: BlockingMutex<CriticalSectionRawMutex, Cell<[u8; SCAN_RSP_AD_LEN]>> =
    BlockingMutex::new(Cell::new([0u8; SCAN_RSP_AD_LEN]));

fn read_state() -> StatusAdvState {
    STATE.lock(|s| s.get())
}

/// 载荷是否已就绪（feature 的总开关：未提交载荷时一切照旧）。
pub(crate) fn enabled() -> bool {
    read_state().payload_valid
}

// ---------------------------------------------------------------------------
// 固件处理器 API（同步，处理器可直接调用）
// ---------------------------------------------------------------------------

/// 提交最新载荷；`None` 停止广播。`burst = true` 表示关键变化
/// （层/修饰键/profile 切换），触发 5×15ms 突发。
pub fn set_payload(payload: Option<&[u8; PAYLOAD_LEN]>, burst: bool) {
    let now = Instant::now().as_millis();
    match payload {
        Some(p) => {
            PAYLOAD.lock(|slot| slot.set(*p));
            PIGGYBACK_SD.lock(|sd| {
                let mut buf = *sd;
                buf[0] = (PAYLOAD_LEN + 1) as u8; // len = type(1) + data(26)
                buf[1] = 0xFF; // manufacturer specific data
                buf[2..].copy_from_slice(p);
                sd.set(buf);
            });
            STATE.lock(|s| {
                let mut st = s.get();
                st.payload_valid = true;
                st.last_activity_ms = now;
                if burst {
                    st.burst_remaining = BURST_COUNT;
                }
                s.set(st);
            });
        }
        None => STATE.lock(|s| {
            let mut st = s.get();
            st.payload_valid = false;
            s.set(st);
        }),
    }
}

/// 触发一轮突发（5×15ms），并刷新活动时间。
pub fn request_burst() {
    let now = Instant::now().as_millis();
    STATE.lock(|s| {
        let mut st = s.get();
        st.last_activity_ms = now;
        st.burst_remaining = BURST_COUNT;
        s.set(st);
    });
}

/// 标记一次活动（如按键），不重发载荷、不触发突发；只把广播节奏切回
/// active（1000ms），避免高频按键刷屏。
pub fn mark_activity() {
    STATE.lock(|s| {
        let mut st = s.get();
        st.last_activity_ms = Instant::now().as_millis();
        s.set(st);
    });
}

// ---------------------------------------------------------------------------
// 连接循环接口（MODE 1）
// ---------------------------------------------------------------------------

/// 连接循环进入广播阶段时调用；离开广播阶段（连接建立 / 休眠）时关闭。
pub(crate) fn set_piggyback(on: bool) {
    STATE.lock(|s| {
        let mut st = s.get();
        st.piggyback = on;
        s.set(st);
    });
}

/// MODE 1：把 SCAN_RSP AD 拷贝进 `out`，返回是否有效。
pub(crate) fn piggyback_scan_rsp(out: &mut [u8; SCAN_RSP_AD_LEN]) -> bool {
    if !enabled() {
        return false;
    }
    PIGGYBACK_SD.lock(|sd| {
        *out = sd.get();
        true
    })
}

/// 连接循环的广播超时：载荷就绪时用刷新节奏替代 300s；突发期间 15ms 连发。
///
/// 注意：`status-adv` 生效后，未连接时的「300s 广播超时→休眠」会被替换为
/// 按 active/idle 节奏持续广播（与 ZMK/Prospector 行为一致），这是广播的
/// 预期功耗取舍。
pub(crate) fn advertise_timeout() -> Duration {
    if !enabled() {
        return Duration::from_secs(300);
    }
    let now = Instant::now().as_millis();
    let st = read_state();
    if st.burst_remaining > 0 {
        STATE.lock(|s| {
            let mut cur = s.get();
            cur.burst_remaining -= 1;
            s.set(cur);
        });
        return Duration::from_millis(BURST_INTERVAL_MS);
    }
    if now.saturating_sub(st.last_activity_ms) < ACTIVITY_TIMEOUT_MS {
        Duration::from_millis(ACTIVE_UPDATE_INTERVAL_MS)
    } else {
        Duration::from_millis(IDLE_UPDATE_INTERVAL_MS)
    }
}

// ---------------------------------------------------------------------------
// MODE 2 广播任务
// ---------------------------------------------------------------------------

/// 读取当前载荷（拷贝）。
fn current_payload() -> Option<[u8; PAYLOAD_LEN]> {
    if !enabled() {
        return None;
    }
    Some(PAYLOAD.lock(|p| p.get()))
}

/// 启动（或保持）MODE 2 广播：non-connectable + scannable，
/// ADV = Flags + manufacturer 载荷，SCAN_RSP = 设备名。
async fn start_mode2_broadcast<C>(
    peripheral: &mut Peripheral<'_, C, DefaultPacketPool>,
    payload: &[u8; PAYLOAD_LEN],
    product_name: &str,
    advertiser: &mut Option<Advertiser<'_, C, DefaultPacketPool>>,
) where
    C: Controller
        + for<'t> ControllerCmdSync<LeSetAdvData>
        + ControllerCmdSync<LeSetAdvParams>
        + for<'t> ControllerCmdSync<LeSetAdvEnable>
        + for<'t> ControllerCmdSync<LeSetScanResponseData>,
{
    if advertiser.is_some() {
        return; // 已在广播（保持期）
    }
    let mut adv_buf = [0u8; 31];
    let mut sd_buf = [0u8; 31];
    let Ok(adv_len) = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::Unknown {
                ty: 0xFF, // manufacturer specific data（Prospector 线格式）
                data: payload,
            },
        ],
        &mut adv_buf[..],
    ) else {
        return;
    };
    let Ok(sd_len) = AdStructure::encode_slice(
        &[AdStructure::CompleteLocalName(product_name.as_bytes())],
        &mut sd_buf[..],
    ) else {
        return;
    };
    let params = AdvertisementParameters {
        primary_phy: PhyKind::Le1M,
        secondary_phy: PhyKind::Le1M,
        tx_power: TxPower::ZerodBm,
        interval_min: MODE2_ADV_INTERVAL,
        interval_max: MODE2_ADV_INTERVAL,
        ..Default::default()
    };    let data = Advertisement::NonconnectableScannableUndirected {
        adv_data: &adv_buf[..adv_len],
        scan_data: &sd_buf[..sd_len],
    };
    match peripheral.advertise(&params, data).await {
        Ok(a) => *advertiser = Some(a),
        Err(e) => {
            #[cfg(feature = "defmt")]
            let e = defmt::Debug2Format(&e);
            warn!("[status-adv] MODE 2 advertise error: {:?}", e);
        }
    }
}

/// 停止当前广播（Drop Advertiser 即停）。
fn stop_broadcast<C, P: PacketPool>(advertiser: &mut Option<Advertiser<'_, C, P>>) {
    *advertiser = None;
}

/// MODE 2 广播主循环。在 `run_ble_keyboard` 中以 `join4` 与连接循环并行。
#[cfg(feature = "status-adv")]
pub(crate) async fn run_status_adv_task<C>(
    stack: &Stack<'_, C, DefaultPacketPool>,
    product_name: &'static str,
) -> !
where
    C: Controller
        + for<'t> ControllerCmdSync<LeSetAdvData>
        + ControllerCmdSync<LeSetAdvParams>
        + for<'t> ControllerCmdSync<LeSetAdvEnable>
        + for<'t> ControllerCmdSync<LeSetScanResponseData>,
{
    let mut peripheral = stack.peripheral();
    let mut advertiser: Option<Advertiser<'_, C, DefaultPacketPool>> = None;

    loop {
        let now = Instant::now().as_millis();
        let st = read_state();

        // MODE 1（连接循环正在广播）：载荷由 SCAN_RSP 携带，本任务不开广播集。
        if !st.payload_valid || st.piggyback {
            stop_broadcast(&mut advertiser);
            Timer::after_millis(200).await;
            continue;
        }

        let Some(payload) = current_payload() else {
            stop_broadcast(&mut advertiser);
            Timer::after_millis(200).await;
            continue;
        };

        // 分体未全连：burst/silent 时间分片（把 radio 让给 scan/connect）。
        let expected = crate::SPLIT_PERIPHERALS_NUM;
        #[cfg(feature = "split")]
        let connected = peripheral_connected_count();
        #[cfg(not(feature = "split"))]
        let connected = 0usize;
        let partial = expected > 0 && connected < expected;

        let hold_ms = if partial {
            let cycle = SPLIT_PARTIAL_BURST_MS + SPLIT_PARTIAL_SILENT_MS;
            let phase = now % cycle;
            if phase < SPLIT_PARTIAL_BURST_MS {
                // burst 窗口：起播并保持到窗口结束。
                start_mode2_broadcast(&mut peripheral, &payload, product_name, &mut advertiser).await;
                SPLIT_PARTIAL_BURST_MS - phase
            } else {
                // silent 窗口：静默等待下一 burst。
                stop_broadcast(&mut advertiser);
                Timer::after(Duration::from_millis(cycle - phase)).await;
                continue;
            }
        } else if st.burst_remaining > 0 {
            STATE.lock(|s| {
                let mut cur = s.get();
                cur.burst_remaining -= 1;
                s.set(cur);
            });
            BURST_INTERVAL_MS
        } else if now.saturating_sub(st.last_activity_ms) < ACTIVITY_TIMEOUT_MS {
            ACTIVE_UPDATE_INTERVAL_MS
        } else {
            IDLE_UPDATE_INTERVAL_MS
        };

        // 起播（未在播时）并保持，然后停播 → 下一次循环以新载荷重播。
        start_mode2_broadcast(&mut peripheral, &payload, product_name, &mut advertiser).await;
        Timer::after(Duration::from_millis(hold_ms)).await;
        stop_broadcast(&mut advertiser);
    }
}
