# EtherCAT Usage

This document describes how to use and integrate the modular EtherCAT network manager and CiA 402 driver handles from another application.

## Prerequisites
- IgH EtherCAT Master installed, libethercat.so.1 available
- Python path includes SttarkStandardLibrary/Python/packages/pyethercat
- ESI XML files available for each device

## Configure
Use the typed configuration to describe your network and slaves.

```python
from ethercat.config_schema import EthercatNetworkConfig, DriveConfig, XmlConfig

cfg = EthercatNetworkConfig(
    master_index=0,
    cycle_time_ms=5.0,
    sdo_only=False,
    slaves=[
        DriveConfig(
            position=0,
            vendor_id=0x00000000,
            product_code=0x00000000,
            xml=XmlConfig(xml_file="/abs/path/to/servo.xml"),
        )
    ]
)
```

## Start network and create drive handle
```python
from ethercat.process_manager import EtherCATProcessManager
from ethercat.client import attach_drive_handle

mgr = EtherCATProcessManager(cfg)
mgr.start()
servo0 = attach_drive_handle(mgr, 0)
```

## Command the drive (non-blocking)
```python
servo0.set_velocity_mode()
servo0.set_velocity(400)
# PP
servo0.set_position_mode()
servo0.set_position_absolute(2000)
# CSP
servo0.set_csp_mode()
servo0.set_position_csp(2050)
```

The process maintains controlword bits (0x6040), modes (0x6060), targets (0x607A/0x60FF), and device features in the cyclic loop, using PDO when mapped or SDO as available.

## Feature detection
- Touch probe and mode command/display mapping are detected from ESI XML by default; you can override PDO entries in config.

## Status reading
Use the handle's getters to read the latest process-published snapshot, e.g. `get_position()`, `get_velocity()`, `get_status_word()`.

## Ruckig (optional)
If the `ruckig` package is installed, integrate trajectories by streaming CSP targets via the handle in your application loop.
