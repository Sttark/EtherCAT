import os
import sys

_repo_root = os.path.dirname(os.path.dirname(__file__))
_repo_parent = os.path.dirname(_repo_root)

if _repo_parent not in sys.path:
    sys.path.insert(0, _repo_parent)

_loaded = sys.modules.get("EtherCAT")
if _loaded is not None and not hasattr(_loaded, "__path__"):
    del sys.modules["EtherCAT"]

from EtherCAT.config_schema import (
    DriveConfig,
    EthercatNetworkConfig,
    PdoSelection,
    RuckigConfig,
    XmlConfig,
)

__all__ = [
    "DriveConfig",
    "EthercatNetworkConfig",
    "PdoSelection",
    "RuckigConfig",
    "XmlConfig",
]
