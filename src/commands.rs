use serde::{Deserialize, Serialize};

use crate::constants::ObjectAddress;

pub type SlavePosition = u16;

pub const EXCLUDED_LEGACY_COMMANDS: &[&str] = &[
    "START_SEMI_ROTARY_RT",
    "UPDATE_SEMI_ROTARY_RT",
    "STOP_SEMI_ROTARY_RT",
    "START_DIE_VELOCITY_TEST",
    "STOP_DIE_VELOCITY_TEST",
];

fn default_unit() -> String {
    "native".to_owned()
}

fn default_homing_timeout_s() -> f64 {
    30.0
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Command {
    pub target_id: SlavePosition,
    #[serde(flatten)]
    pub kind: CommandKind,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", content = "payload")]
pub enum CommandKind {
    #[serde(rename = "SET_POSITION_MODE")]
    SetPositionMode(ModeChangeCommand),
    #[serde(rename = "SET_VELOCITY_MODE")]
    SetVelocityMode(ModeChangeCommand),
    #[serde(rename = "SET_TORQUE_MODE")]
    SetTorqueMode(ModeChangeCommand),
    #[serde(rename = "SET_CSP_MODE")]
    SetCspMode(ModeChangeCommand),
    #[serde(rename = "SET_CSV_MODE")]
    SetCsvMode(ModeChangeCommand),
    #[serde(rename = "SET_CST_MODE")]
    SetCstMode(ModeChangeCommand),
    #[serde(rename = "START_HOMING")]
    StartHoming(HomingCommand),
    #[serde(rename = "SET_POSITION")]
    SetPosition(PositionCommand),
    #[serde(rename = "SET_VELOCITY")]
    SetVelocity(VelocityCommand),
    #[serde(rename = "SET_TORQUE")]
    SetTorque(TorqueCommand),
    #[serde(rename = "SET_POSITION_CSP")]
    SetPositionCsp(PositionCommand),
    #[serde(rename = "SET_VELOCITY_CSV")]
    SetVelocityCsv(VelocityCommand),
    #[serde(rename = "SET_TORQUE_CST")]
    SetTorqueCst(TorqueCommand),
    #[serde(rename = "STOP_MOTION")]
    StopMotion(StopMotionCommand),
    #[serde(rename = "START_RUCKIG_POSITION")]
    StartRuckigPosition(RuckigPositionCommand),
    #[serde(rename = "START_RUCKIG_VELOCITY")]
    StartRuckigVelocity(RuckigVelocityCommand),
    #[serde(rename = "STOP_RUCKIG")]
    StopRuckig,
    #[serde(rename = "ENABLE_DRIVE")]
    EnableDrive,
    #[serde(rename = "DISABLE_DRIVE")]
    DisableDrive,
    #[serde(rename = "SHUTDOWN_DRIVE")]
    ShutdownDrive,
    #[serde(rename = "ARM_PROBE")]
    ArmProbe(ProbeArmCommand),
    #[serde(rename = "DISABLE_PROBE")]
    DisableProbe,
    #[serde(rename = "WRITE_RAW_PDO")]
    WriteRawPdo(RawPdoWriteCommand),
    #[serde(rename = "NO_OP")]
    NoOp,
    #[serde(rename = "CLEAR_FAULT")]
    ClearFault,
    #[serde(rename = "READ_SDO")]
    ReadSdo(SdoReadCommand),
    #[serde(rename = "WRITE_SDO")]
    WriteSdo(SdoWriteCommand),
}

impl CommandKind {
    pub const fn is_motion_command(&self) -> bool {
        matches!(
            self,
            Self::SetPosition(_)
                | Self::SetVelocity(_)
                | Self::SetTorque(_)
                | Self::SetPositionCsp(_)
                | Self::SetVelocityCsv(_)
                | Self::SetTorqueCst(_)
                | Self::StopMotion(_)
                | Self::StartRuckigPosition(_)
                | Self::StartRuckigVelocity(_)
                | Self::StopRuckig
                | Self::StartHoming(_)
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct ModeChangeCommand {
    #[serde(default)]
    pub safe_switch: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PositionCommand {
    pub position: f64,
    #[serde(default = "default_unit")]
    pub unit: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VelocityCommand {
    pub velocity: f64,
    #[serde(default = "default_unit")]
    pub unit: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TorqueCommand {
    pub torque: f64,
    #[serde(default = "default_unit")]
    pub unit: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct StopMotionCommand {
    #[serde(default = "default_true")]
    pub stop_internal_trajectory: bool,
}

const fn default_true() -> bool {
    true
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct HomingCommand {
    #[serde(default)]
    pub wait_for_completion: bool,
    #[serde(default = "default_homing_timeout_s")]
    pub timeout_s: f64,
}

impl Default for HomingCommand {
    fn default() -> Self {
        Self {
            wait_for_completion: false,
            timeout_s: default_homing_timeout_s(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RuckigPositionCommand {
    pub target_position: f64,
    #[serde(default = "default_unit")]
    pub unit: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_velocity: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_acceleration: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_jerk: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RuckigVelocityCommand {
    pub target_velocity: f64,
    #[serde(default = "default_unit")]
    pub unit: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_acceleration: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_jerk: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProbeEdge {
    Positive,
    Negative,
}

impl ProbeEdge {
    pub const fn probe_function(self) -> u16 {
        match self {
            Self::Positive => 0x0005,
            Self::Negative => 0x0009,
        }
    }

    pub const fn from_probe_function(raw: u16) -> Option<Self> {
        match raw {
            0x0005 => Some(Self::Positive),
            0x0009 => Some(Self::Negative),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProbeArmCommand {
    pub edge: ProbeEdge,
    #[serde(default)]
    pub continuous: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RawPdoWriteCommand {
    pub address: ObjectAddress,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct SdoReadCommand {
    pub address: ObjectAddress,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SdoWriteCommand {
    pub address: ObjectAddress,
    pub data: Vec<u8>,
}
