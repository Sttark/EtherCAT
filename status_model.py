from dataclasses import dataclass, field
from typing import Dict, Optional, Any, List


@dataclass
class DriveStatus:
    in_op: bool = False
    enabled: bool = False
    statusword: Optional[int] = None
    position_actual: Optional[float] = None
    velocity_actual: Optional[float] = None
    error_code: Optional[int] = None
    digital_inputs: Optional[int] = None
    dip_in_state: Optional[int] = None
    probe_enabled: bool = False
    probe_active: bool = False
    probe_pos1: Optional[float] = None
    probe_pos2: Optional[float] = None
    features: Dict[str, Any] = field(default_factory=dict)


@dataclass
class NetworkStatus:
    timestamp_ns: Optional[int] = None
    cycle_time_ms_config: Optional[float] = None
    last_cycle_time_ns: Optional[int] = None
    last_cycle_time_us: Optional[int] = None
    last_cycle_jitter_ns: Optional[int] = None
    last_cycle_jitter_us: Optional[int] = None
    max_abs_cycle_jitter_ns: Optional[int] = None
    max_abs_cycle_jitter_us: Optional[int] = None
    max_abs_cycle_jitter_post_warmup_ns: Optional[int] = None
    max_abs_cycle_jitter_post_warmup_us: Optional[int] = None
    jitter_p95_ns: Optional[int] = None
    jitter_p99_ns: Optional[int] = None
    jitter_p999_ns: Optional[int] = None
    jitter_p95_us: Optional[int] = None
    jitter_p99_us: Optional[int] = None
    jitter_p999_us: Optional[int] = None
    deadline_miss_count: int = 0
    overrun_count: int = 0
    max_overrun_ns: int = 0
    last_work_ns: Optional[int] = None
    max_work_ns: int = 0
    work_p99_ns: Optional[int] = None
    last_send_interval_ns: Optional[int] = None
    min_sleep_budget_ns: Optional[int] = None
    domain_wc: Optional[int] = None
    domain_wc_state: Optional[int] = None
    domain_wc_min: Optional[int] = None
    domain_wc_max: Optional[int] = None
    dc_sync_error_ns: Optional[int] = None
    dc_sync_error_p95_ns: Optional[int] = None
    dc_sync_error_p99_ns: Optional[int] = None
    dc_sync_error_p999_ns: Optional[int] = None
    dc_sync_error_max_ns: Optional[int] = None
    dc_sync_samples: int = 0
    motion_command_block_count: int = 0
    all_slaves_op_first_ns: Optional[int] = None
    all_slaves_op_last_ns: Optional[int] = None
    all_slaves_left_op_last_ns: Optional[int] = None
    sdo_only: bool = False
    drives: Dict[int, Dict[str, Any]] = field(default_factory=dict)
    # drives mapping: position -> { ... DriveStatus fields ... }
    
    # Working counter monitoring (non-RT)
    domain_wc_drop_count: int = 0
    domain_wc_expected: int = 0
    slaves_not_in_op: List[int] = field(default_factory=list)
    
    # RT performance monitoring (non-RT)
    rt_budget_usage_percent: float = 0.0
    rt_budget_warnings: int = 0
    phase_recv_ns: int = 0
    phase_state_ns: int = 0
    phase_cia_ns: int = 0
    phase_ruckig_ns: int = 0
    phase_write_ns: int = 0
    phase_dc_send_ns: int = 0



