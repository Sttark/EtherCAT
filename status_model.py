from dataclasses import dataclass, field
from typing import Dict, Optional, Any


@dataclass
class DriveStatus:
    in_op: bool = False
    enabled: bool = False
    statusword: Optional[int] = None
    position_actual: Optional[float] = None
    velocity_actual: Optional[float] = None
    error_code: Optional[int] = None
    digital_inputs: Optional[int] = None
    probe_enabled: bool = False
    probe_active: bool = False
    probe_pos1: Optional[float] = None
    probe_pos2: Optional[float] = None
    features: Dict[str, Any] = field(default_factory=dict)


@dataclass
class NetworkStatus:
    timestamp_ns: Optional[int] = None
    cycle_time_ms_config: Optional[float] = None
    last_cycle_time_us: Optional[int] = None
    sdo_only: bool = False
    drives: Dict[int, Dict[str, Any]] = field(default_factory=dict)
    # drives mapping: position -> { ... DriveStatus fields ... }



