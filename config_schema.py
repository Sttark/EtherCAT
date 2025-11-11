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
    rotation_direction: Optional[str] = None
    inertia_ratio: Optional[float] = None
    position_limits: Optional[Tuple[float, float, str]] = None
    features_overrides: Dict[str, Any] = field(default_factory=dict)


@dataclass
class EthercatNetworkConfig:
    master_index: int
    network_interface: Optional[str] = None
    cycle_time_ms: float = 5.0
    cpu_core: Optional[int] = None
    rt_priority: Optional[int] = None
    sdo_only: bool = False
    slaves: List[DriveConfig] = field(default_factory=list)



