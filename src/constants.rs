use serde::{Deserialize, Serialize};

pub type ObjectIndex = u16;
pub type ObjectSubIndex = u8;

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize, Default,
)]
pub struct ObjectAddress {
    pub index: ObjectIndex,
    #[serde(default)]
    pub subindex: ObjectSubIndex,
}

impl ObjectAddress {
    pub const fn new(index: ObjectIndex, subindex: ObjectSubIndex) -> Self {
        Self { index, subindex }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum OperationMode {
    NoMode,
    ProfilePosition,
    Velocity,
    ProfileVelocity,
    ProfileTorque,
    Homing,
    CyclicSynchronousPosition,
    CyclicSynchronousVelocity,
    CyclicSynchronousTorque,
}

impl OperationMode {
    pub const fn raw(self) -> i8 {
        match self {
            Self::NoMode => 0,
            Self::ProfilePosition => 1,
            Self::Velocity => 2,
            Self::ProfileVelocity => 3,
            Self::ProfileTorque => 4,
            Self::Homing => 6,
            Self::CyclicSynchronousPosition => 8,
            Self::CyclicSynchronousVelocity => 9,
            Self::CyclicSynchronousTorque => 10,
        }
    }

    pub const fn from_raw(raw: i8) -> Option<Self> {
        match raw {
            0 => Some(Self::NoMode),
            1 => Some(Self::ProfilePosition),
            2 => Some(Self::Velocity),
            3 => Some(Self::ProfileVelocity),
            4 => Some(Self::ProfileTorque),
            6 => Some(Self::Homing),
            8 => Some(Self::CyclicSynchronousPosition),
            9 => Some(Self::CyclicSynchronousVelocity),
            10 => Some(Self::CyclicSynchronousTorque),
            _ => None,
        }
    }
}

pub const CW_INDEX: ObjectIndex = 0x6040;
pub const SW_INDEX: ObjectIndex = 0x6041;

pub const MODES_OP_INDEX: ObjectIndex = 0x6060;
pub const MODES_OP_DISPLAY_INDEX: ObjectIndex = 0x6061;

pub const TARGET_POSITION_INDEX: ObjectIndex = 0x607A;
pub const TARGET_VELOCITY_INDEX: ObjectIndex = 0x60FF;
pub const TARGET_TORQUE_INDEX: ObjectIndex = 0x6071;
pub const MAX_TORQUE_INDEX: ObjectIndex = 0x6072;

pub const POSITION_ACTUAL_INDEX: ObjectIndex = 0x6064;
pub const VELOCITY_ACTUAL_INDEX: ObjectIndex = 0x606C;
pub const TORQUE_ACTUAL_INDEX: ObjectIndex = 0x6077;

pub const ERROR_CODE_INDEX: ObjectIndex = 0x603F;

pub const PROBE_FUNCTION_INDEX: ObjectIndex = 0x60B8;
pub const PROBE_STATUS_INDEX: ObjectIndex = 0x60B9;
pub const PROBE_POS1_INDEX: ObjectIndex = 0x60BA;
pub const PROBE_POS2_INDEX: ObjectIndex = 0x60BC;
pub const PROBE_POS2_ALT_INDEX: ObjectIndex = 0x60BB;
pub const DIGITAL_INPUTS_INDEX: ObjectIndex = 0x60FD;
pub const DIP_IN_STATE_INDEX: ObjectIndex = 0x4020;

pub const CW_BIT_NEW_SET_POINT: u8 = 4;
pub const CW_BIT_CHANGE_IMMEDIATELY: u8 = 5;
pub const CW_BIT_ABS_REL: u8 = 6;
pub const CW_BIT_HALT: u8 = 8;

pub const CW_ENABLE_OP_SIMPLIFIED: u16 = 0x000F;

pub const SW_BIT_FAULT: u8 = 3;
pub const SW_BIT_WARNING: u8 = 7;
pub const SW_BIT_TARGET_REACHED: u8 = 10;
pub const SW_BIT_INTERNAL_LIMIT_ACTIVE: u8 = 11;
pub const SW_BIT_SETPOINT_ACK: u8 = 12;

pub const PROBE_FUNC_ENABLE_PROBE1: u16 = 0x0001;
pub const PROBE_FUNC_PROBE1_POS_EDGE: u16 = 0x0004;
pub const PROBE_FUNC_PROBE1_NEG_EDGE: u16 = 0x0008;

pub const fn bit_mask(bit: u8) -> u16 {
    1u16 << bit
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(transparent)]
pub struct Statusword(pub u16);

impl Statusword {
    pub const fn fault(self) -> bool {
        self.0 & bit_mask(SW_BIT_FAULT) != 0
    }

    pub const fn warning(self) -> bool {
        self.0 & bit_mask(SW_BIT_WARNING) != 0
    }

    pub const fn target_reached(self) -> bool {
        self.0 & bit_mask(SW_BIT_TARGET_REACHED) != 0
    }

    pub const fn internal_limit_active(self) -> bool {
        self.0 & bit_mask(SW_BIT_INTERNAL_LIMIT_ACTIVE) != 0
    }

    pub const fn setpoint_ack(self) -> bool {
        self.0 & bit_mask(SW_BIT_SETPOINT_ACK) != 0
    }

    pub const fn cia402_state_bits(self) -> u16 {
        self.0 & 0x006F
    }
}
