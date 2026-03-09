use serde::{Deserialize, Serialize};

pub type ShmWord = i64;

pub const SLOT_SEQUENCE: usize = 0;
pub const SLOT_CYCLE_COUNT: usize = 1;
pub const SLOT_TIMESTAMP_NS: usize = 2;
pub const SLOT_JITTER_NS: usize = 3;
pub const SLOT_MAX_JITTER_PW_NS: usize = 4;
pub const SLOT_LATENESS_NS: usize = 5;
pub const SLOT_WORK_NS: usize = 6;
pub const SLOT_MAX_WORK_NS: usize = 7;
pub const SLOT_OVERRUN_COUNT: usize = 8;
pub const SLOT_MAX_OVERRUN_NS: usize = 9;
pub const SLOT_SEND_INTERVAL_NS: usize = 10;
pub const SLOT_MIN_SLEEP_BUDGET_NS: usize = 11;
pub const SLOT_DEADLINE_MISS_COUNT: usize = 12;
pub const SLOT_DC_SYNC_ERR_NS: usize = 13;
pub const SLOT_DC_SYNC_ERR_MAX_NS: usize = 14;
pub const SLOT_DOMAIN_WC: usize = 15;
pub const SLOT_DOMAIN_WC_STATE: usize = 16;
pub const SLOT_DOMAIN_WC_MIN: usize = 17;
pub const SLOT_DOMAIN_WC_MAX: usize = 18;
pub const SLOT_ALL_OP_FIRST_NS: usize = 19;
pub const SLOT_ALL_OP_LAST_NS: usize = 20;
pub const SLOT_ALL_OP_LEFT_LAST_NS: usize = 21;
pub const SLOT_CYCLE_TIME_MS_X1000: usize = 22;
pub const SLOT_IO_DI_BASE: usize = 23;
pub const SLOT_IO_DI_COUNT: usize = 4;
pub const SLOT_IO_DO_BASE: usize = 27;
pub const SLOT_IO_DO_COUNT: usize = 4;
pub const HEADER_SIZE: usize = 32;

pub const DRIVE_BASE: usize = HEADER_SIZE;
pub const DRIVE_STRIDE: usize = 10;
pub const DRIVE_POSITION: usize = 0;
pub const DRIVE_VELOCITY: usize = 1;
pub const DRIVE_TORQUE: usize = 2;
pub const DRIVE_STATUSWORD: usize = 3;
pub const DRIVE_MODE: usize = 4;
pub const DRIVE_ERROR_CODE: usize = 5;
pub const DRIVE_IN_OP: usize = 6;
pub const DRIVE_ENABLED: usize = 7;
pub const DRIVE_DIGITAL_INPUTS: usize = 8;
pub const DRIVE_DIP_IN_STATE: usize = 9;

pub const MAX_DRIVES: usize = 16;
pub const ACTUALS_SIZE: usize = DRIVE_BASE + MAX_DRIVES * DRIVE_STRIDE;

pub const TARGET_SEQUENCE: usize = 0;
pub const TARGET_BASE: usize = 1;
pub const TARGET_STRIDE: usize = 3;
pub const TARGET_CSP: usize = 0;
pub const TARGET_MODE: usize = 1;
pub const TARGET_VALID: usize = 2;
pub const TARGETS_SIZE: usize = TARGET_BASE + MAX_DRIVES * TARGET_STRIDE;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HeaderField {
    Sequence,
    CycleCount,
    TimestampNs,
    JitterNs,
    MaxJitterPostWarmupNs,
    LatenessNs,
    WorkNs,
    MaxWorkNs,
    OverrunCount,
    MaxOverrunNs,
    SendIntervalNs,
    MinSleepBudgetNs,
    DeadlineMissCount,
    DcSyncErrorNs,
    DcSyncErrorMaxNs,
    DomainWorkingCounter,
    DomainWorkingCounterState,
    DomainWorkingCounterMin,
    DomainWorkingCounterMax,
    AllSlavesOpFirstNs,
    AllSlavesOpLastNs,
    AllSlavesLeftOpLastNs,
    CycleTimeMsX1000,
}

impl HeaderField {
    pub const fn offset(self) -> usize {
        match self {
            Self::Sequence => SLOT_SEQUENCE,
            Self::CycleCount => SLOT_CYCLE_COUNT,
            Self::TimestampNs => SLOT_TIMESTAMP_NS,
            Self::JitterNs => SLOT_JITTER_NS,
            Self::MaxJitterPostWarmupNs => SLOT_MAX_JITTER_PW_NS,
            Self::LatenessNs => SLOT_LATENESS_NS,
            Self::WorkNs => SLOT_WORK_NS,
            Self::MaxWorkNs => SLOT_MAX_WORK_NS,
            Self::OverrunCount => SLOT_OVERRUN_COUNT,
            Self::MaxOverrunNs => SLOT_MAX_OVERRUN_NS,
            Self::SendIntervalNs => SLOT_SEND_INTERVAL_NS,
            Self::MinSleepBudgetNs => SLOT_MIN_SLEEP_BUDGET_NS,
            Self::DeadlineMissCount => SLOT_DEADLINE_MISS_COUNT,
            Self::DcSyncErrorNs => SLOT_DC_SYNC_ERR_NS,
            Self::DcSyncErrorMaxNs => SLOT_DC_SYNC_ERR_MAX_NS,
            Self::DomainWorkingCounter => SLOT_DOMAIN_WC,
            Self::DomainWorkingCounterState => SLOT_DOMAIN_WC_STATE,
            Self::DomainWorkingCounterMin => SLOT_DOMAIN_WC_MIN,
            Self::DomainWorkingCounterMax => SLOT_DOMAIN_WC_MAX,
            Self::AllSlavesOpFirstNs => SLOT_ALL_OP_FIRST_NS,
            Self::AllSlavesOpLastNs => SLOT_ALL_OP_LAST_NS,
            Self::AllSlavesLeftOpLastNs => SLOT_ALL_OP_LEFT_LAST_NS,
            Self::CycleTimeMsX1000 => SLOT_CYCLE_TIME_MS_X1000,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DriveField {
    Position,
    Velocity,
    Torque,
    Statusword,
    ModeDisplay,
    ErrorCode,
    InOp,
    Enabled,
    DigitalInputs,
    DipInState,
}

impl DriveField {
    pub const fn offset(self) -> usize {
        match self {
            Self::Position => DRIVE_POSITION,
            Self::Velocity => DRIVE_VELOCITY,
            Self::Torque => DRIVE_TORQUE,
            Self::Statusword => DRIVE_STATUSWORD,
            Self::ModeDisplay => DRIVE_MODE,
            Self::ErrorCode => DRIVE_ERROR_CODE,
            Self::InOp => DRIVE_IN_OP,
            Self::Enabled => DRIVE_ENABLED,
            Self::DigitalInputs => DRIVE_DIGITAL_INPUTS,
            Self::DipInState => DRIVE_DIP_IN_STATE,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TargetField {
    CspTarget,
    ModeCommand,
    Valid,
}

impl TargetField {
    pub const fn offset(self) -> usize {
        match self {
            Self::CspTarget => TARGET_CSP,
            Self::ModeCommand => TARGET_MODE,
            Self::Valid => TARGET_VALID,
        }
    }
}

pub const fn drive_base(drive_index: usize) -> usize {
    DRIVE_BASE + drive_index * DRIVE_STRIDE
}

pub const fn drive_slot(drive_index: usize, field: DriveField) -> usize {
    drive_base(drive_index) + field.offset()
}

pub const fn target_base(drive_index: usize) -> usize {
    TARGET_BASE + drive_index * TARGET_STRIDE
}

pub const fn target_slot(drive_index: usize, field: TargetField) -> usize {
    target_base(drive_index) + field.offset()
}

pub const fn is_valid_drive_index(drive_index: usize) -> bool {
    drive_index < MAX_DRIVES
}
