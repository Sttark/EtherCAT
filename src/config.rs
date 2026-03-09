use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::constants::{ObjectAddress, OperationMode};

fn default_cycle_time_ms() -> f64 {
    5.0
}

fn default_ethercat_device_path() -> String {
    "/dev/EtherCAT0".to_owned()
}

fn default_force_release_retry_delay_s() -> f64 {
    1.0
}

fn default_force_release_attempts() -> u32 {
    3
}

fn default_op_timeout_s() -> f64 {
    10.0
}

fn default_enable_transition_period_ms() -> f64 {
    500.0
}

fn default_pp_ack_mask() -> u16 {
    0x1000
}

fn default_pp_ack_timeout_ms() -> f64 {
    100.0
}

fn default_status_publish_period_s() -> f64 {
    0.05
}

fn default_status_proxy_refresh_s() -> f64 {
    0.02
}

fn default_process_timing_log_period_s() -> f64 {
    1.0
}

fn default_dc_m2s_k() -> f64 {
    0.01
}

fn default_dc_m2s_window() -> u32 {
    11
}

fn default_dc_m2s_max_correction_ns() -> u64 {
    20_000
}

fn default_dc_settling_delay_s() -> f64 {
    2.0
}

fn default_sdo_timeout_s() -> f64 {
    2.0
}

fn default_velocity_lookahead_s() -> f64 {
    0.5
}

const fn default_jitter_window_size() -> usize {
    2048
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct XmlConfig {
    pub xml_file: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vendor_id: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub product_code: Option<u32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct PdoEntryRegistration {
    pub address: ObjectAddress,
    pub bit_length: u8,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct CustomPdoConfig {
    #[serde(default)]
    pub rx_entries: Vec<PdoEntryRegistration>,
    #[serde(default)]
    pub tx_entries: Vec<PdoEntryRegistration>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct PdoSelection {
    #[serde(default)]
    pub rx_pdos: Vec<u16>,
    #[serde(default)]
    pub tx_pdos: Vec<u16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom_pdo_config: Option<CustomPdoConfig>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct HomingConfig {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub method: Option<i8>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub search_velocity: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub zero_velocity: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub acceleration: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub offset: Option<f64>,
    #[serde(default = "default_native_unit")]
    pub unit: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct UnitConversion {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub units_per_pulse: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scale_factor: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RuckigConfig {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dt_ms: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_velocity: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_acceleration: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_jerk: Option<f64>,
    #[serde(default = "default_velocity_lookahead_s")]
    pub velocity_lookahead_s: f64,
    #[serde(default = "default_true")]
    pub hold_last_commanded_position: bool,
}

impl Default for RuckigConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            dt_ms: None,
            max_velocity: None,
            max_acceleration: None,
            max_jerk: None,
            velocity_lookahead_s: default_velocity_lookahead_s(),
            hold_last_commanded_position: true,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct PositionLimits {
    pub minimum: f64,
    pub maximum: f64,
    #[serde(default = "default_native_unit")]
    pub unit: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct FeatureFlags {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub touch_probe: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mode_command_in_pdo: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mode_display_in_pdo: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub controlword: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub statusword: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mode_command: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mode_display: Option<bool>,
    #[serde(default, flatten)]
    pub extra: BTreeMap<String, bool>,
}

impl FeatureFlags {
    pub fn is_empty(&self) -> bool {
        self.touch_probe.is_none()
            && self.mode_command_in_pdo.is_none()
            && self.mode_display_in_pdo.is_none()
            && self.controlword.is_none()
            && self.statusword.is_none()
            && self.mode_command.is_none()
            && self.mode_display.is_none()
            && self.extra.is_empty()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct StartupSdo {
    pub address: ObjectAddress,
    #[serde(default)]
    pub data: Vec<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SyncDirection {
    Output,
    Input,
}

impl Default for SyncDirection {
    fn default() -> Self {
        Self::Output
    }
}

impl SyncDirection {
    pub const fn raw(self) -> u8 {
        match self {
            Self::Output => 1,
            Self::Input => 2,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct SyncPdoConfig {
    pub pdo_index: u16,
    #[serde(default)]
    pub entries: Vec<PdoEntryRegistration>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct SyncManagerConfig {
    pub sync_manager_index: u8,
    pub direction: SyncDirection,
    #[serde(default)]
    pub pdos: Vec<SyncPdoConfig>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct PdoReadSize {
    pub address: ObjectAddress,
    pub size_bytes: u8,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct DriveConfig {
    pub position: u16,
    #[serde(default)]
    pub alias: u16,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vendor_id: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub product_code: Option<u32>,
    #[serde(default)]
    pub enable_dc: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dc_assign_activate: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dc_sync0_cycle_time_ns: Option<u32>,
    #[serde(default)]
    pub dc_sync0_shift_ns: i32,
    #[serde(default)]
    pub dc_sync1_cycle_time_ns: i32,
    #[serde(default)]
    pub dc_sync1_shift_ns: i32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub operation_mode: Option<OperationMode>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub profile_velocity: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub profile_acceleration: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_velocity: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_torque: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub homing: Option<HomingConfig>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub xml: Option<XmlConfig>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pdo: Option<PdoSelection>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub unit_conversion: Option<UnitConversion>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ruckig: Option<RuckigConfig>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rotation_direction: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub inertia_ratio: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub position_limits: Option<PositionLimits>,
    #[serde(default, skip_serializing_if = "FeatureFlags::is_empty")]
    pub features_overrides: FeatureFlags,
    #[serde(default)]
    pub pv_requires_setpoint_toggle: bool,
    #[serde(default)]
    pub pt_requires_setpoint_toggle: bool,
    #[serde(default = "default_true")]
    pub cia402: bool,
    #[serde(default)]
    pub startup_sdos: Vec<StartupSdo>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sm_watchdog_timeout_ms: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sm_watchdog_divider: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sync_configs: Option<Vec<SyncManagerConfig>>,
    #[serde(default)]
    pub register_entries: Vec<ObjectAddress>,
    #[serde(default)]
    pub pdo_read_sizes: Vec<PdoReadSize>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DcMasterSyncMode {
    #[serde(rename = "m2s")]
    MasterToSlave,
    #[serde(rename = "s2m")]
    SlaveToMaster,
}

impl Default for DcMasterSyncMode {
    fn default() -> Self {
        Self::MasterToSlave
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct EthercatNetworkConfig {
    pub master_index: u32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub network_interface: Option<String>,
    #[serde(default = "default_cycle_time_ms")]
    pub cycle_time_ms: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cpu_core: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub igh_master_cpu_core: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rt_priority: Option<u32>,
    #[serde(default)]
    pub irq_affinity: BTreeMap<u32, String>,
    #[serde(default)]
    pub sdo_only: bool,
    #[serde(default)]
    pub force_release_master_on_startup: bool,
    #[serde(default = "default_ethercat_device_path")]
    pub ethercat_device_path: String,
    #[serde(default = "default_true")]
    pub force_release_sigterm_first: bool,
    #[serde(default = "default_force_release_retry_delay_s")]
    pub force_release_retry_delay_s: f64,
    #[serde(default = "default_force_release_attempts")]
    pub force_release_attempts: u32,
    #[serde(default = "default_true")]
    pub force_release_debug_owners: bool,
    #[serde(default = "default_op_timeout_s")]
    pub op_timeout_s: f64,
    #[serde(default = "default_enable_transition_period_ms")]
    pub enable_transition_period_ms: f64,
    #[serde(default = "default_pp_ack_mask")]
    pub pp_ack_mask: u16,
    #[serde(default = "default_pp_ack_timeout_ms")]
    pub pp_ack_timeout_ms: f64,
    #[serde(default = "default_true")]
    pub auto_enable_cia402: bool,
    #[serde(default)]
    pub force_disable_on_start: bool,
    #[serde(default)]
    pub forbid_motion_commands: bool,
    #[serde(default = "default_jitter_window_size")]
    pub jitter_window_size: usize,
    #[serde(default)]
    pub jitter_warmup_cycles: usize,
    #[serde(default)]
    pub deadline_miss_threshold_ns: u64,
    #[serde(default = "default_status_publish_period_s")]
    pub status_publish_period_s: f64,
    #[serde(default = "default_status_proxy_refresh_s")]
    pub status_proxy_refresh_s: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub process_log_file: Option<String>,
    #[serde(default = "default_true")]
    pub process_log_stderr: bool,
    #[serde(default = "default_true")]
    pub process_status_debug_log: bool,
    #[serde(default = "default_true")]
    pub process_timing_log: bool,
    #[serde(default = "default_process_timing_log_period_s")]
    pub process_timing_log_period_s: f64,
    #[serde(default = "default_true")]
    pub process_op_transition_log: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dc_reference_slave: Option<u16>,
    #[serde(default)]
    pub dc_master_sync_mode: DcMasterSyncMode,
    #[serde(default = "default_dc_m2s_k")]
    pub dc_m2s_k: f64,
    #[serde(default = "default_dc_m2s_window")]
    pub dc_m2s_window: u32,
    #[serde(default = "default_dc_m2s_max_correction_ns")]
    pub dc_m2s_max_correction_ns: u64,
    #[serde(default = "default_dc_settling_delay_s")]
    pub dc_settling_delay_s: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sm_watchdog_timeout_ms: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sm_watchdog_divider: Option<u32>,
    #[serde(default = "default_sdo_timeout_s")]
    pub sdo_timeout_s: f64,
    #[serde(default = "default_true")]
    pub fault_error_code_sdo_fallback: bool,
    #[serde(default)]
    pub slaves: Vec<DriveConfig>,
}

fn default_native_unit() -> String {
    "native".to_owned()
}

const fn default_true() -> bool {
    true
}
