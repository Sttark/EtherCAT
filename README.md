# EtherCAT

Modular EtherCAT network manager with isolated process, non-blocking CiA 402 drives, and XML-driven PDO/SDO mapping.

## Goals
- Single-responsibility: network/process, device drivers, and adapters are decoupled
- Non-blocking: all drive APIs enqueue intents; cyclic task realizes them
- Config-first: no hard-coded device specifics; use ESI XML or user overrides
- Future-proof: supports additional device types beyond drives

## Setup
1) Ensure IgH master and libethercat.so.1 are installed and available.
2) No external pyethercat/SttarkStandardLibrary dependency is required. This package uses a direct ctypes binding to `libethercat.so.1` via `EtherCAT/igh_master.py`.
3) Provide ESI XML paths per slave in config.

## Safe shutdown + startup preflight (systemd-friendly)
- **Graceful stop**: the isolated process traps **SIGTERM/SIGINT** and exits cleanly, ensuring `Master.deactivate()`/`Master.release()` are called.
- **Release stuck master on startup (optional)**: if the master request fails, you can enable a best-effort preflight that attempts to kill processes holding `/dev/EtherCAT0` (via `fuser`) and then retries the request.

These are controlled via `EthercatNetworkConfig`:
- `force_release_master_on_startup` (default `False`)
- `ethercat_device_path` (default `"/dev/EtherCAT0"`)
- `force_release_sigterm_first` (default `True`)
- `force_release_retry_delay_s` (default `1.0`)

An optional helper script is provided:
- `safe_shutdown.sh`: sends SIGTERM to a matching app process and (optionally) runs `fuser` to release `/dev/EtherCAT0`.

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
- Integrates by streaming **CSP target position (0x607A)** every cycle from inside the isolated process.
- Enable via `DriveConfig.ruckig.enabled=True` and set jerk-limited limits (`max_velocity`, `max_acceleration`, `max_jerk`) in drive-native units.

## File structure (subject to change with packaging)
- constants.py: CoE indices, masks, modes
- igh_master.py: Standalone IgH Master (ctypes exact)
- xml_decoder.py: ESI parsing and PDO maps
- process_manager.py: Isolated process (queues, cyclic maintenance, status)
- commands.py / status_model.py: transport models
- cia402/driver.py: CiA 402 handle (non-blocking)
- client.py: handle attach and status proxy
- ruckig_addon.py: optional CSP trajectory shim
- USAGE.md: external usage notes
