use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::config::FeatureFlags;
use crate::constants::{ObjectAddress, Statusword};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct ProbeStatus {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default)]
    pub active: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub position_1: Option<i32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub position_2: Option<i32>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct SlaveBusState {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub al_state: Option<u8>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub online: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub operational: Option<bool>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct OpLifecycle {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entered_first_ns: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entered_last_ns: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub left_last_ns: Option<u64>,
    #[serde(default)]
    pub dropout_count: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PdoPresence {
    Missing,
    Pdo,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PdoHealthEntry {
    pub address: ObjectAddress,
    pub presence: PdoPresence,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RawPdoValue {
    pub address: ObjectAddress,
    pub value: u64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct DriveStatus {
    #[serde(default)]
    pub in_op: bool,
    #[serde(default)]
    pub enabled: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub statusword: Option<Statusword>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mode_display: Option<i8>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub position_actual: Option<i32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub velocity_actual: Option<i32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub torque_actual: Option<i16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error_code: Option<u16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub digital_inputs: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dip_in_state: Option<u32>,
    #[serde(default)]
    pub probe: ProbeStatus,
    #[serde(default)]
    pub fault: bool,
    #[serde(default)]
    pub warning: bool,
    #[serde(default)]
    pub target_reached: bool,
    #[serde(default)]
    pub internal_limit_active: bool,
    #[serde(default)]
    pub setpoint_ack: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub enable_requested: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub manual_disable: Option<bool>,
    #[serde(default)]
    pub bus_state: SlaveBusState,
    #[serde(default)]
    pub op_lifecycle: OpLifecycle,
    #[serde(default, skip_serializing_if = "FeatureFlags::is_empty")]
    pub features: FeatureFlags,
    #[serde(default)]
    pub raw_pdo: Vec<RawPdoValue>,
    #[serde(default)]
    pub pdo_health: Vec<PdoHealthEntry>,
}

impl DriveStatus {
    pub fn refresh_derived_flags(&mut self) {
        if let Some(statusword) = self.statusword {
            self.fault = statusword.fault();
            self.warning = statusword.warning();
            self.target_reached = statusword.target_reached();
            self.internal_limit_active = statusword.internal_limit_active();
            self.setpoint_ack = statusword.setpoint_ack();
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct NetworkStatus {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub timestamp_ns: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cycle_time_ms_config: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_cycle_time_ns: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_cycle_time_us: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_cycle_jitter_ns: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_cycle_jitter_us: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_abs_cycle_jitter_ns: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_abs_cycle_jitter_us: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_abs_cycle_jitter_post_warmup_ns: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_abs_cycle_jitter_post_warmup_us: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub jitter_p95_ns: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub jitter_p99_ns: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub jitter_p999_ns: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub jitter_p95_us: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub jitter_p99_us: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub jitter_p999_us: Option<i64>,
    #[serde(default)]
    pub deadline_miss_count: u64,
    #[serde(default)]
    pub overrun_count: u64,
    #[serde(default)]
    pub max_overrun_ns: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_work_ns: Option<u64>,
    #[serde(default)]
    pub max_work_ns: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub work_p99_ns: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_send_interval_ns: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min_sleep_budget_ns: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub domain_wc: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub domain_wc_state: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub domain_wc_min: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub domain_wc_max: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dc_sync_error_ns: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dc_sync_error_p95_ns: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dc_sync_error_p99_ns: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dc_sync_error_p999_ns: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dc_sync_error_max_ns: Option<u64>,
    #[serde(default)]
    pub dc_sync_samples: u64,
    #[serde(default)]
    pub motion_command_block_count: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub all_slaves_op_first_ns: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub all_slaves_op_last_ns: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub all_slaves_left_op_last_ns: Option<u64>,
    #[serde(default)]
    pub sdo_only: bool,
    #[serde(default)]
    pub drives: BTreeMap<u16, DriveStatus>,
    #[serde(default)]
    pub domain_wc_drop_count: u64,
    #[serde(default)]
    pub domain_wc_expected: u32,
    #[serde(default)]
    pub slaves_not_in_op: Vec<u16>,
    #[serde(default)]
    pub rt_budget_usage_percent: f64,
    #[serde(default)]
    pub rt_budget_warnings: u64,
    #[serde(default)]
    pub phase_recv_ns: u64,
    #[serde(default)]
    pub phase_state_ns: u64,
    #[serde(default)]
    pub phase_cia_ns: u64,
    #[serde(default)]
    pub phase_ruckig_ns: u64,
    #[serde(default)]
    pub phase_write_ns: u64,
    #[serde(default)]
    pub phase_dc_send_ns: u64,
}
