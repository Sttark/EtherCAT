from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any


@dataclass
class XmlConfig:
    """
    Configuration for parsing ESI XML to derive device-specific features and PDOs.
    Paths should be absolute for reliability.
    """
    xml_file: str
    vendor_id: Optional[int] = None
    product_code: Optional[int] = None


@dataclass
class PdoSelection:
    """
    Optional custom PDO mapping to override or augment ESI mappings.
    Each list contains PDO indices (e.g., 0x1600, 0x1A00) in order.
    """
    rx_pdos: List[int] = field(default_factory=list)
    tx_pdos: List[int] = field(default_factory=list)
    custom_pdo_config: Optional[Dict[str, List[Tuple[int, int, int]]]] = None


@dataclass
class HomingConfig:
    method: Optional[int] = None
    search_vel: Optional[float] = None
    zero_vel: Optional[float] = None
    accel: Optional[float] = None
    offset: Optional[float] = None
    unit: str = "native"


@dataclass
class UnitConversion:
    """
    Define conversion between drive pulses and user units.
    All conversions are provided by configuration; nothing is hard-coded.
    """
    units_per_pulse: Optional[float] = None
    scale_factor: Optional[float] = None


@dataclass
class RuckigConfig:
    """
    Optional jerk-limited trajectory generation settings for CSP streaming.
    All values are in drive-native units (e.g., pulses, pulses/s, pulses/s², pulses/s³).
    """
    enabled: bool = False
    # Optional override of the control timestep used by the Ruckig generator.
    # If None, defaults to EthercatNetworkConfig.cycle_time_ms.
    dt_ms: Optional[float] = None
    max_velocity: Optional[float] = None
    max_acceleration: Optional[float] = None
    max_jerk: Optional[float] = None
    # Velocity command behavior: how far ahead (seconds) we keep the target position when
    # commanding a steady velocity via CSP position streaming.
    velocity_lookahead_s: float = 0.5
    # When stopping, hold the last commanded position (True) vs. hold the latest measured position (False).
    hold_last_commanded_position: bool = True


@dataclass
class DriveConfig:
    """
    Generic drive configuration for CiA 402 devices.
    Keep configuration minimal and explicit; avoid hard-coded defaults.
    """
    position: int
    alias: int = 0
    vendor_id: Optional[int] = None
    product_code: Optional[int] = None
    enable_dc: bool = False
    dc_assign_activate: Optional[int] = None
    dc_sync0_cycle_time_ns: Optional[int] = None
    dc_sync0_shift_ns: int = 0
    dc_sync1_cycle_time_ns: int = 0
    dc_sync1_shift_ns: int = 0
    operation_mode: Optional[int] = None
    profile_velocity: Optional[float] = None
    profile_acceleration: Optional[float] = None
    max_velocity: Optional[float] = None
    homing: Optional[HomingConfig] = None
    xml: Optional[XmlConfig] = None
    pdo: Optional[PdoSelection] = None
    unit_conversion: Optional[UnitConversion] = None
    ruckig: Optional[RuckigConfig] = None
    rotation_direction: Optional[str] = None
    inertia_ratio: Optional[float] = None
    position_limits: Optional[Tuple[float, float, str]] = None
    features_overrides: Dict[str, Any] = field(default_factory=dict)
    # --- Motion semantics overrides (device-specific) ---
    # Some drives require a set-point strobe (controlword bit 4) not only for PP but also for PV/PT.
    pv_requires_setpoint_toggle: bool = False
    pt_requires_setpoint_toggle: bool = False


@dataclass
class EthercatNetworkConfig:
    master_index: int
    network_interface: Optional[str] = None
    cycle_time_ms: float = 5.0
    cpu_core: Optional[int] = None
    rt_priority: Optional[int] = None
    sdo_only: bool = False
    # --- Startup preflight (optional, safety-first) ---
    # If True, and requesting the master fails, attempt to release any prior process holding the
    # EtherCAT device (typically /dev/EtherCAT0) and retry.
    #
    # WARNING: the release step may require root privileges (fuser -k) and can kill other processes
    # using EtherCAT on this machine. Keep default False and enable only when you own the box.
    force_release_master_on_startup: bool = False
    ethercat_device_path: str = "/dev/EtherCAT0"
    # When force-releasing, try SIGTERM first, then SIGKILL.
    force_release_sigterm_first: bool = True
    # Delay between release attempt and retrying master request.
    force_release_retry_delay_s: float = 1.0
    # Number of force-release + retry cycles to attempt before giving up.
    force_release_attempts: int = 3
    # If True, print `fuser -v` output to logs before/after force-release to identify owners.
    force_release_debug_owners: bool = True
    # --- Cyclic safety / correctness knobs ---
    # Timeout to reach OP after activation. If any slave stays out of OP longer than this,
    # the process will error/stop (prevents infinite SAFEOP stalls).
    op_timeout_s: float = 10.0
    # CiA402 enable state machine pacing (time-based). Default 100ms is conservative and
    # matches the proven behavior in the legacy runtime. Can be tightened for faster drives.
    enable_transition_period_ms: float = 100.0
    # Profile Position (PP) "new set-point" pulse handshake parameters.
    # Bit 12 (0x1000) is commonly "set-point acknowledged" in CiA402 PP.
    pp_ack_mask: int = 0x1000
    pp_ack_timeout_ms: float = 100.0
    slaves: List[DriveConfig] = field(default_factory=list)



