from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Dict, Optional


class CommandType(Enum):
    # Mode control
    SET_POSITION_MODE = auto()
    SET_VELOCITY_MODE = auto()
    SET_TORQUE_MODE = auto()
    SET_CSP_MODE = auto()
    SET_CSV_MODE = auto()
    SET_CST_MODE = auto()
    START_HOMING = auto()

    # Motion
    SET_POSITION = auto()
    SET_VELOCITY = auto()
    SET_TORQUE = auto()
    SET_POSITION_CSP = auto()
    SET_VELOCITY_CSV = auto()
    SET_TORQUE_CST = auto()
    STOP_MOTION = auto()
    START_RUCKIG_POSITION = auto()
    START_RUCKIG_VELOCITY = auto()
    STOP_RUCKIG = auto()

    # Drive state
    ENABLE_DRIVE = auto()
    DISABLE_DRIVE = auto()
    SHUTDOWN_DRIVE = auto()

    # Probe and IO (device dependent)
    ARM_PROBE = auto()
    DISABLE_PROBE = auto()

    # Raw I/O
    WRITE_RAW_PDO = auto()

    # Diagnostics and housekeeping
    NO_OP = auto()
    CLEAR_FAULT = auto()
    READ_SDO = auto()
    WRITE_SDO = auto()


@dataclass
class Command:
    target_id: int
    type: CommandType
    value: Optional[Any] = None
    params: Dict[str, Any] = field(default_factory=dict)
    correlation_id: Optional[str] = None



