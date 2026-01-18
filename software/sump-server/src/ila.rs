//! SUMP3 ILA API endpoints
//!
//! Provides REST API for the open-source SUMP3 Integrated Logic Analyzer.
//! Uses polling-based register access via /dev/mem (no IRQ/kernel driver needed).

use axum::{
    extract::{Path, State},
    response::Json,
    routing::{get, post},
    Router,
};
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use parking_lot::Mutex;

use crate::devmem::DevMem;

const ILA_SIZE: usize = 0x100;

// Register offsets (from sump3_axi_wrapper.sv)
const REG_CMD: usize        = 0x00;
const REG_ADDR: usize       = 0x04;
const REG_WDATA: usize      = 0x08;
const REG_CTRL: usize       = 0x0C;
const REG_STATUS: usize     = 0x10;
const REG_RDATA: usize      = 0x14;
const REG_HW_INFO: usize    = 0x1C;
const REG_CAP_STATUS: usize = 0x20;

// Command codes - State commands
const CMD_ARM: u32          = 0x01;
const CMD_RESET: u32        = 0x02;
const CMD_INIT: u32         = 0x03;

// Command codes - Local reads
const CMD_RD_HW_ID: u32         = 0x10;
const CMD_RD_STATUS: u32        = 0x12;

// Command codes - Local writes
const CMD_WR_TRIG_TYPE: u32     = 0x23;
const CMD_WR_TRIG_DIG_FIELD: u32= 0x24;
const CMD_WR_DIG_POST_TRIG: u32 = 0x2A;

// Command codes - Serial bus reads (external CMD codes from sump3_axi_wrapper.sv)
const CMD_RD_HUB_FREQ: u32      = 0x30;
const CMD_RD_POD_COUNT: u32     = 0x31;
const CMD_RD_POD_REG: u32       = 0x32;
const CMD_RD_HUB_INSTANCE: u32  = 0x35;
const CMD_RD_HUB_NAME_0_3: u32  = 0x36;
const CMD_RD_HUB_NAME_4_7: u32  = 0x37;
const CMD_RD_HUB_NAME_8_11: u32 = 0x38;

// Command codes - Serial bus writes
const CMD_WR_POD_REG: u32       = 0x40;

// Pod register addresses
const POD_REG_HW_CFG: u8        = 0x00;
const POD_REG_TRIG_CFG: u8      = 0x03;
const POD_REG_TRIG_EN: u8       = 0x04;
const POD_REG_RAM_PTR: u8       = 0x08;
const POD_REG_RAM_DATA: u8      = 0x09;
const POD_REG_RAM_CFG: u8       = 0x0A;
const POD_REG_TRIGGERABLE: u8   = 0x0E;
const POD_REG_NAME_0_3: u8      = 0x1D;
const POD_REG_NAME_4_7: u8      = 0x1E;
const POD_REG_NAME_8_11: u8     = 0x1F;

// Control bits
const CTRL_START: u32 = 0x01;

// Trigger types
const TRIG_OR_RISING: u32       = 0x02;
const TRIG_OR_FALLING: u32      = 0x03;
const TRIG_EXT_RISING: u32      = 0x06;

/// Shared state containing the ILA memory map
pub struct IlaState {
    mem: Mutex<DevMem>,
    base_addr: usize,
}

impl IlaState {
    pub fn new(base_addr: usize) -> Result<Self, std::io::Error> {
        let mem = DevMem::new(base_addr, ILA_SIZE)?;
        tracing::info!(
            "SUMP3 ILA mapped at 0x{:08X}, size {} bytes",
            base_addr,
            ILA_SIZE
        );
        Ok(Self { 
            mem: Mutex::new(mem),
            base_addr,
        })
    }
    
    /// Execute a command and wait for completion (polling)
    fn exec_cmd(&self, cmd: u32, addr: u32, wdata: u32) -> Option<u32> {
        let mem = self.mem.lock();
        
        // Write command parameters
        mem.write32(REG_CMD, cmd);
        mem.write32(REG_ADDR, addr);
        mem.write32(REG_WDATA, wdata);
        
        // Set START bit to begin execution
        mem.write32(REG_CTRL, CTRL_START);
        
        // Poll for completion (DONE bit)
        for _ in 0..100000 {
            let status = mem.read32(REG_STATUS)?;
            let done = (status & 0x02) != 0;
            let error = (status & 0x04) != 0;
            
            if done {
                if error {
                    tracing::warn!("ILA command 0x{:02X} error", cmd);
                    return None;
                }
                return mem.read32(REG_RDATA);
            }
            std::hint::spin_loop();
        }
        tracing::warn!("ILA command 0x{:02X} timeout", cmd);
        None
    }
    
    /// Read a pod register
    fn read_pod_reg(&self, hub: u8, pod: u8, reg: u8) -> Option<u32> {
        let addr = ((hub as u32) << 16) | ((pod as u32) << 8) | (reg as u32);
        self.exec_cmd(CMD_RD_POD_REG, addr, 0)
    }
    
    /// Write a pod register
    fn write_pod_reg(&self, hub: u8, pod: u8, reg: u8, value: u32) -> bool {
        let addr = ((hub as u32) << 16) | ((pod as u32) << 8) | (reg as u32);
        self.exec_cmd(CMD_WR_POD_REG, addr, value).is_some()
    }
    
    /// Read hub name (12 ASCII chars)
    fn read_hub_name(&self, hub: u8) -> String {
        let addr = (hub as u32) << 16;
        let mut name = Vec::with_capacity(12);
        
        for cmd in [CMD_RD_HUB_NAME_0_3, CMD_RD_HUB_NAME_4_7, CMD_RD_HUB_NAME_8_11] {
            if let Some(data) = self.exec_cmd(cmd, addr, 0) {
                name.push(((data >> 24) & 0xFF) as u8);
                name.push(((data >> 16) & 0xFF) as u8);
                name.push(((data >> 8) & 0xFF) as u8);
                name.push((data & 0xFF) as u8);
            }
        }
        
        String::from_utf8_lossy(&name).trim().to_string()
    }
    
    /// Read pod name (12 ASCII chars)
    fn read_pod_name(&self, hub: u8, pod: u8) -> String {
        let mut name = Vec::with_capacity(12);
        
        for reg in [POD_REG_NAME_0_3, POD_REG_NAME_4_7, POD_REG_NAME_8_11] {
            if let Some(data) = self.read_pod_reg(hub, pod, reg) {
                name.push(((data >> 24) & 0xFF) as u8);
                name.push(((data >> 16) & 0xFF) as u8);
                name.push(((data >> 8) & 0xFF) as u8);
                name.push((data & 0xFF) as u8);
            }
        }
        
        String::from_utf8_lossy(&name).trim().to_string()
    }
    
    /// Read RLE sample from pod RAM (with configurable timestamp bits)
    fn read_rle_sample(&self, hub: u8, pod: u8, addr: u32, ts_bits: u8) -> Option<RleSample> {
        // Set RAM pointer to page 0, address
        self.write_pod_reg(hub, pod, POD_REG_RAM_PTR, addr);
        
        // Read low 32 bits (data)
        let data = self.read_pod_reg(hub, pod, POD_REG_RAM_DATA)?;
        
        // Read high bits from page 1
        self.write_pod_reg(hub, pod, POD_REG_RAM_PTR, (1 << 20) | addr);
        let hi = self.read_pod_reg(hub, pod, POD_REG_RAM_DATA)?;
        
        // Decode based on timestamp width
        let ts_mask = (1u32 << ts_bits) - 1;
        let code = ((hi >> ts_bits) & 0x3) as u8;
        let timestamp = (hi & ts_mask) as u32;
        
        Some(RleSample {
            address: addr,
            code,
            timestamp,
            data,
        })
    }
    
    /// Get pod configuration (timestamp bits, data bits, etc.)
    fn get_pod_config(&self, hub: u8, pod: u8) -> (u8, u16, u32) {
        let ram_cfg = self.read_pod_reg(hub, pod, POD_REG_RAM_CFG).unwrap_or(0);
        let depth_bits = (ram_cfg & 0xFF) as u8;
        let data_bits = ((ram_cfg >> 8) & 0xFFFF) as u16;
        let ts_bits = ((ram_cfg >> 24) & 0xFF) as u8;
        let ram_depth = 1u32 << depth_bits;
        (ts_bits, data_bits, ram_depth)
    }
}

// ============================================================================
// Signal generation helpers
// ============================================================================

/// Generate signal list based on norom_view_* flags
fn generate_norom_signals(
    pod_name: &str,
    data_bits: u16,
    view_dwords: bool,
    view_words: bool,
    view_bytes: bool,
    view_bits: bool,
    rle_disable: bool,
) -> (String, Vec<SignalInfo>) {
    let mut signals = Vec::new();
    let pod_name_trimmed = pod_name.trim();
    
    // Determine view mode based on flags
    let view_mode = if view_dwords {
        "dwords"
    } else if view_words {
        "words"
    } else if view_bytes {
        "bytes"
    } else if view_bits {
        "bits"
    } else {
        "dwords" // default
    };
    
    let signal_type = if rle_disable { "analog" } else { "vector" };
    
    // Special case: ADC I/Q pod with known layout
    if pod_name_trimmed.contains("adc") && pod_name_trimmed.contains("iq") && data_bits >= 25 {
        signals.push(SignalInfo {
            name: "adc_i[11:0]".to_string(),
            bit_high: 11,
            bit_low: 0,
            signal_type: "analog".to_string(),
        });
        signals.push(SignalInfo {
            name: "adc_q[11:0]".to_string(),
            bit_high: 23,
            bit_low: 12,
            signal_type: "analog".to_string(),
        });
        signals.push(SignalInfo {
            name: "adc_valid".to_string(),
            bit_high: 24,
            bit_low: 24,
            signal_type: "bit".to_string(),
        });
        return ("iq".to_string(), signals);
    }
    
    match view_mode {
        "dwords" => {
            let num_dwords = (data_bits + 31) / 32;
            for i in 0..num_dwords {
                let bit_low = i * 32;
                let bit_high = std::cmp::min((i + 1) * 32 - 1, data_bits - 1);
                signals.push(SignalInfo {
                    name: if num_dwords == 1 {
                        format!("{}[{}:0]", pod_name_trimmed, bit_high)
                    } else {
                        format!("{}_d{}[{}:{}]", pod_name_trimmed, i, bit_high, bit_low)
                    },
                    bit_high,
                    bit_low,
                    signal_type: signal_type.to_string(),
                });
            }
        }
        "words" => {
            let num_words = (data_bits + 15) / 16;
            for i in 0..num_words {
                let bit_low = i * 16;
                let bit_high = std::cmp::min((i + 1) * 16 - 1, data_bits - 1);
                signals.push(SignalInfo {
                    name: if num_words == 1 {
                        format!("{}[{}:0]", pod_name_trimmed, bit_high)
                    } else {
                        format!("{}_w{}[{}:{}]", pod_name_trimmed, i, bit_high, bit_low)
                    },
                    bit_high,
                    bit_low,
                    signal_type: signal_type.to_string(),
                });
            }
        }
        "bytes" => {
            let num_bytes = (data_bits + 7) / 8;
            for i in 0..num_bytes {
                let bit_low = i * 8;
                let bit_high = std::cmp::min((i + 1) * 8 - 1, data_bits - 1);
                signals.push(SignalInfo {
                    name: format!("{}_b{}[{}:{}]", pod_name_trimmed, i, bit_high, bit_low),
                    bit_high,
                    bit_low,
                    signal_type: "vector".to_string(),
                });
            }
        }
        "bits" => {
            for i in 0..data_bits {
                signals.push(SignalInfo {
                    name: format!("{}[{}]", pod_name_trimmed, i),
                    bit_high: i,
                    bit_low: i,
                    signal_type: "bit".to_string(),
                });
            }
        }
        _ => {}
    }
    
    (view_mode.to_string(), signals)
}

// ============================================================================
// Data structures
// ============================================================================

#[derive(Debug, Serialize)]
pub struct IlaInfo {
    pub connected: bool,
    pub hw_id: String,
    pub revision: u8,
    pub hub_count: u8,
    pub is_armed: bool,
    pub is_awake: bool,
    pub base_addr: String,
    pub hubs: Vec<HubInfo>,
}

#[derive(Debug, Serialize)]
pub struct HubInfo {
    pub index: u8,
    pub name: String,
    pub freq_mhz: u32,
    pub pod_count: u8,
    pub pods: Vec<PodInfo>,
}

#[derive(Debug, Serialize)]
pub struct PodInfo {
    pub index: u8,
    pub name: String,
    pub hw_rev: u8,
    pub ram_depth: u32,
    pub data_bits: u16,
    pub ts_bits: u8,
    pub triggerable: u32,
    pub rle_disable: bool,
    pub view_rom_en: bool,
    pub view_mode: String,
    pub signals: Vec<SignalInfo>,
}

#[derive(Debug, Serialize, Clone)]
pub struct SignalInfo {
    pub name: String,
    pub bit_high: u16,
    pub bit_low: u16,
    pub signal_type: String,
}

#[derive(Debug, Serialize, Clone)]
pub struct RleSample {
    pub address: u32,
    pub code: u8,
    pub timestamp: u32,
    pub data: u32,
}

#[derive(Debug, Serialize)]
pub struct CaptureStatus {
    pub armed: bool,
    pub pre_trigger: bool,
    pub triggered: bool,
    pub acquired: bool,
    pub init_in_progress: bool,
}

#[derive(Debug, Serialize)]
pub struct CaptureData {
    pub hub: u8,
    pub pod: u8,
    pub ts_bits: u8,
    pub data_bits: u16,
    pub status: CaptureStatus,
    pub samples: Vec<RleSample>,
    pub sample_count: u32,
}

#[derive(Debug, Deserialize)]
pub struct TriggerConfig {
    #[serde(default)]
    pub trigger_type: String,
    #[serde(default)]
    pub trigger_bits: u32,
    #[serde(default = "default_post_trigger")]
    pub post_trigger: u32,
}

fn default_post_trigger() -> u32 { 64 }

#[derive(Debug, Serialize)]
pub struct RegisterValue {
    pub offset: usize,
    pub value: Option<u32>,
}

#[derive(Debug, Serialize)]
pub struct CommandResult {
    pub success: bool,
    pub message: String,
}

// ============================================================================
// API handlers
// ============================================================================

/// GET /api/ila - Get ILA info with full hub/pod enumeration
async fn get_info(State(state): State<Arc<IlaState>>) -> Json<IlaInfo> {
    let mem = state.mem.lock();
    
    let hw_info = mem.read32(REG_HW_INFO).unwrap_or(0);
    drop(mem);
    
    let id = (hw_info >> 16) & 0xFFFF;
    let hub_count = ((hw_info >> 8) & 0xFF) as u8;
    let revision = (hw_info & 0xFF) as u8;
    
    let connected = id == 0x5303;
    let hw_id = format!("{}{}", 
        char::from_u32((id >> 8) & 0xFF).unwrap_or('?'),
        char::from_u32(id & 0xFF).unwrap_or('?')
    );
    
    let mem = state.mem.lock();
    let cap_status = mem.read32(REG_CAP_STATUS).unwrap_or(0);
    drop(mem);
    
    let is_armed = (cap_status & 0x01) != 0;
    let is_awake = (cap_status & 0x02) != 0;
    
    // Enumerate hubs and pods
    let mut hubs = Vec::new();
    if connected {
        for hub_idx in 0..hub_count {
            let addr = (hub_idx as u32) << 16;
            
            let name = state.read_hub_name(hub_idx);
            let freq = state.exec_cmd(CMD_RD_HUB_FREQ, addr, 0).unwrap_or(0);
            let freq_mhz = (freq >> 20) & 0xFFF;
            let pod_count = state.exec_cmd(CMD_RD_POD_COUNT, addr, 0)
                .map(|v| (v & 0xFF) as u8)
                .unwrap_or(0);
            
            let mut pods = Vec::new();
            for pod_idx in 0..pod_count {
                let pod_name = state.read_pod_name(hub_idx, pod_idx);
                
                let hw_cfg = state.read_pod_reg(hub_idx, pod_idx, POD_REG_HW_CFG).unwrap_or(0);
                let hw_rev = ((hw_cfg >> 24) & 0xFF) as u8;
                
                let norom_view_dwords = (hw_cfg & 0x0800) != 0;
                let norom_view_words = (hw_cfg & 0x0400) != 0;
                let norom_view_bytes = (hw_cfg & 0x0200) != 0;
                let norom_view_bits = (hw_cfg & 0x0100) != 0;
                let rle_disable = (hw_cfg & 0x04) != 0;
                let view_rom_en = (hw_cfg & 0x02) != 0;
                
                let ram_cfg = state.read_pod_reg(hub_idx, pod_idx, POD_REG_RAM_CFG).unwrap_or(0);
                let depth_bits = (ram_cfg & 0xFF) as u8;
                let data_bits = ((ram_cfg >> 8) & 0xFFFF) as u16;
                let ts_bits = ((ram_cfg >> 24) & 0xFF) as u8;
                let ram_depth = 1u32 << depth_bits;
                
                let triggerable = state.read_pod_reg(hub_idx, pod_idx, POD_REG_TRIGGERABLE).unwrap_or(0);
                
                let (view_mode, signals) = if view_rom_en {
                    ("custom".to_string(), Vec::new())
                } else {
                    generate_norom_signals(&pod_name, data_bits, 
                        norom_view_dwords, norom_view_words, norom_view_bytes, norom_view_bits,
                        rle_disable)
                };
                
                pods.push(PodInfo {
                    index: pod_idx,
                    name: pod_name,
                    hw_rev,
                    ram_depth,
                    data_bits,
                    ts_bits,
                    triggerable,
                    rle_disable,
                    view_rom_en,
                    view_mode,
                    signals,
                });
            }
            
            hubs.push(HubInfo {
                index: hub_idx,
                name,
                freq_mhz,
                pod_count,
                pods,
            });
        }
    }
    
    Json(IlaInfo {
        connected,
        hw_id,
        revision,
        hub_count,
        is_armed,
        is_awake,
        base_addr: format!("0x{:08X}", state.base_addr),
        hubs,
    })
}

/// GET /api/ila/status - Get capture status
async fn get_capture_status(State(state): State<Arc<IlaState>>) -> Json<CaptureStatus> {
    let status = state.exec_cmd(CMD_RD_STATUS, 0, 0).unwrap_or(0);
    
    Json(CaptureStatus {
        armed: (status & 0x01) != 0,
        pre_trigger: (status & 0x02) != 0,
        triggered: (status & 0x04) != 0,
        acquired: (status & 0x08) != 0,
        init_in_progress: (status & 0x10) != 0,
    })
}

/// POST /api/ila/reset - Reset ILA
async fn post_reset(State(state): State<Arc<IlaState>>) -> Json<CommandResult> {
    let success = state.exec_cmd(CMD_RESET, 0, 0).is_some();
    Json(CommandResult {
        success,
        message: if success { "Reset complete".into() } else { "Reset failed".into() },
    })
}

/// POST /api/ila/init - Initialize RAM
async fn post_init(State(state): State<Arc<IlaState>>) -> Json<CommandResult> {
    let success = state.exec_cmd(CMD_INIT, 0, 0).is_some();
    std::thread::sleep(std::time::Duration::from_millis(100));
    Json(CommandResult {
        success,
        message: if success { "Init complete".into() } else { "Init failed".into() },
    })
}

/// POST /api/ila/arm - Arm for capture
async fn post_arm(State(state): State<Arc<IlaState>>) -> Json<CommandResult> {
    let success = state.exec_cmd(CMD_ARM, 0, 0).is_some();
    Json(CommandResult {
        success,
        message: if success { "Armed".into() } else { "Arm failed".into() },
    })
}

/// POST /api/ila/trigger - Configure trigger and arm
async fn post_configure_trigger(
    State(state): State<Arc<IlaState>>,
    Json(config): Json<TriggerConfig>,
) -> Json<CommandResult> {
    if state.exec_cmd(CMD_RESET, 0, 0).is_none() {
        return Json(CommandResult { success: false, message: "Reset failed".into() });
    }
    
    let trig_type = match config.trigger_type.as_str() {
        "or_falling" => TRIG_OR_FALLING,
        "external" => TRIG_EXT_RISING,
        _ => TRIG_OR_RISING,
    };
    
    if state.exec_cmd(CMD_WR_TRIG_TYPE, 0, trig_type).is_none() {
        return Json(CommandResult { success: false, message: "Failed to set trigger type".into() });
    }
    
    let trig_bits = if config.trigger_bits == 0 { 0x00000001 } else { config.trigger_bits };
    if state.exec_cmd(CMD_WR_TRIG_DIG_FIELD, 0, trig_bits).is_none() {
        return Json(CommandResult { success: false, message: "Failed to set trigger field".into() });
    }
    
    if state.exec_cmd(CMD_WR_DIG_POST_TRIG, 0, config.post_trigger).is_none() {
        return Json(CommandResult { success: false, message: "Failed to set post-trigger".into() });
    }
    
    let pod_trig_cfg = (trig_type & 0x07) | 0x20;
    state.write_pod_reg(0, 0, POD_REG_TRIG_CFG, pod_trig_cfg);
    state.write_pod_reg(0, 0, POD_REG_TRIG_EN, trig_bits);
    
    if state.exec_cmd(CMD_INIT, 0, 0).is_none() {
        return Json(CommandResult { success: false, message: "Init failed".into() });
    }
    std::thread::sleep(std::time::Duration::from_millis(200));
    
    if state.exec_cmd(CMD_ARM, 0, 0).is_none() {
        return Json(CommandResult { success: false, message: "Arm failed".into() });
    }
    
    Json(CommandResult {
        success: true,
        message: format!("Configured: type={}, bits=0x{:08X}, post={}", 
            config.trigger_type, trig_bits, config.post_trigger),
    })
}

/// GET /api/ila/capture/:count - Get captured samples from hub 0, pod 0 (default)
async fn get_capture(
    State(state): State<Arc<IlaState>>,
    Path(count): Path<u32>,
) -> Json<CaptureData> {
    get_capture_from_pod(state, 0, 0, count).await
}

/// GET /api/ila/capture/:hub/:pod/:count - Get captured samples from specific hub/pod
async fn get_capture_hub_pod(
    State(state): State<Arc<IlaState>>,
    Path((hub, pod, count)): Path<(u8, u8, u32)>,
) -> Json<CaptureData> {
    get_capture_from_pod(state, hub, pod, count).await
}

/// Internal function to capture from a specific hub/pod
async fn get_capture_from_pod(
    state: Arc<IlaState>,
    hub: u8,
    pod: u8,
    count: u32,
) -> Json<CaptureData> {
    let status_val = state.exec_cmd(CMD_RD_STATUS, 0, 0).unwrap_or(0);
    let status = CaptureStatus {
        armed: (status_val & 0x01) != 0,
        pre_trigger: (status_val & 0x02) != 0,
        triggered: (status_val & 0x04) != 0,
        acquired: (status_val & 0x08) != 0,
        init_in_progress: (status_val & 0x10) != 0,
    };
    
    let (ts_bits, data_bits, ram_depth) = state.get_pod_config(hub, pod);
    
    let sample_count = count.min(ram_depth).min(2048);
    let mut samples = Vec::with_capacity(sample_count as usize);
    
    for i in 0..sample_count {
        if let Some(sample) = state.read_rle_sample(hub, pod, i, ts_bits) {
            samples.push(sample);
        }
    }
    
    Json(CaptureData {
        hub,
        pod,
        ts_bits,
        data_bits,
        status,
        samples,
        sample_count,
    })
}

/// GET /api/ila/reg/:offset - Read raw register
async fn get_register(
    State(state): State<Arc<IlaState>>,
    Path(offset): Path<usize>,
) -> Json<RegisterValue> {
    let value = if offset < ILA_SIZE {
        let mem = state.mem.lock();
        mem.read32(offset)
    } else {
        None
    };
    
    Json(RegisterValue { offset, value })
}

/// Create the ILA API router
pub fn ila_router(state: Arc<IlaState>) -> Router {
    Router::new()
        .route("/", get(get_info))
        .route("/status", get(get_capture_status))
        .route("/reset", post(post_reset))
        .route("/init", post(post_init))
        .route("/arm", post(post_arm))
        .route("/trigger", post(post_configure_trigger))
        .route("/capture/:hub/:pod/:count", get(get_capture_hub_pod))
        .route("/capture/:count", get(get_capture))
        .route("/reg/:offset", get(get_register))
        .with_state(state)
}
