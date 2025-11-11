# EtherCAT

Modular EtherCAT network manager with isolated process, non-blocking CiA 402 drives, and XML-driven PDO/SDO mapping.

## Goals
- Single-responsibility: network/process, device drivers, and adapters are decoupled
- Non-blocking: all drive APIs enqueue intents; cyclic task realizes them
- Config-first: no hard-coded device specifics; use ESI XML or user overrides
- Future-proof: supports additional device types beyond drives

## Setup
1) Ensure IgH master and libethercat.so.1 are installed and available.
2) Ensure SttarkStandardLibrary/Python/packages/pyethercat is on PYTHONPATH.
3) Provide ESI XML paths per slave in config.

## Minimal example
```python
from ethercat.config_schema import EthercatNetworkConfig, DriveConfig, XmlConfig
from ethercat.process_manager import EtherCATProcessManager
from ethercat.client import attach_drive_handle

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
        ),
    ],
)

mgr = EtherCATProcessManager(cfg)
mgr.start()
servo0 = attach_drive_handle(mgr, 0)
servo0.set_velocity_mode()
servo0.set_velocity(400)
```

## SDO vs PDO
- The process auto-detects if an object is mapped to PDO; if not, SDO is used when safe.
- Modes (0x6060/0x6061) and controlword (0x6040) can be maintained in PDO if configured.

## Touch probe
- If the XML (or override) exposes 0x60B8/0x60B9/0x60BA/0x60BC, probe is supported.

## Ruckig (optional)
- Integrates via CSP target streaming; enable when ruckig is available.

## File structure (subject to change with packaging)
- constants.py: CoE indices, masks, modes
- master_adapter.py: Standalone IgH Master (ctypes exact)
- xml_decoder.py: ESI parsing and PDO maps
- process_manager.py: Isolated process (queues, cyclic maintenance, status)
- commands.py / status_model.py: transport models
- cia402/driver.py: CiA 402 handle (non-blocking)
- client.py: handle attach and status proxy
- ruckig_addon.py: optional CSP trajectory shim
- USAGE.md: external usage notes
