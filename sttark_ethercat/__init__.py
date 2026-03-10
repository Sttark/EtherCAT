"""
ethercat_v2

Modular EtherCAT network manager and CiA 402 v2 drivers.

This package provides a process-isolated EtherCAT network manager with
non-blocking CiA 402 drive handles, PDO/SDO auto-selection, and optional
add-ons like Ruckig-based motion planning.
"""

__all__ = [
    "client",
    "config_schema",
    "commands",
    "status_model",
    "igh_master",
    "ruckig_planner",
]

__version__ = "0.1.0"



