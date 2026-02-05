# EtherCAT Usage

This document describes how to use and integrate the modular EtherCAT network manager and CiA 402 driver handles from another application.

## Prerequisites
- IgH EtherCAT Master installed, libethercat.so.1 available
- No external pyethercat/SttarkStandardLibrary dependency is required. `EtherCAT/igh_master.py` binds directly to `libethercat.so.1`.
- ESI XML files available for each device

## Configure
Use the typed configuration to describe your network and slaves.

```python
from ethercat.config_schema import EthercatNetworkConfig, DriveConfig, XmlConfig

cfg = EthercatNetworkConfig(
    master_index=0,
    cycle_time_ms=5.0,
    sdo_only=False,
    # Startup preflight (optional; best-effort)
    # If the master is busy, attempt to release prior holders of /dev/EtherCAT0 and retry.
    force_release_master_on_startup=False,
    ethercat_device_path="/dev/EtherCAT0",
    # Safety/correctness knobs (optional)
    op_timeout_s=10.0,                 # Fail fast if slaves never reach OP (prevents SAFEOP stalls)
    enable_transition_period_ms=100.0, # Time-based CiA402 enable pacing (default 100ms)
    pp_ack_mask=0x1000,                # PP set-point acknowledged bit (CiA402 typical)
    pp_ack_timeout_ms=100.0,           # Clear PP bit4 if no ack within this time
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

## Safe shutdown (systemd stop)
If your application is managed by systemd, it will typically stop via SIGTERM. The EtherCAT isolated
process traps SIGTERM/SIGINT and exits cleanly, ensuring `Master.deactivate()` and `Master.release()`
run during teardown.

If you want a manual helper similar to Core-Cutter’s workflow, use:
- `safe_shutdown.sh --pattern "<your app regex>"`

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

**PDO mode requirement:** if `sdo_only=False`, each slave must have **0x6040 (controlword)** and **0x6041 (statusword)** mapped in its PDOs, otherwise the process will raise an error at startup.

## Feature detection
- Touch probe and mode command/display mapping are detected from ESI XML by default; you can override PDO entries in config.

## Status reading
Use the handle's getters to read the latest process-published snapshot, e.g. `get_position()`, `get_velocity()`, `get_status_word()`.

## Ruckig (optional)
If the `ruckig` package is installed, you can run jerk-limited (S-curve) motion generation inside the isolated process
and stream CSP targets (0x607A) every cycle.

At minimum, your drive must have **0x6064 (position actual)** and **0x606C (velocity actual)** mapped in PDOs for
Ruckig initialization.

### Configure
```python
from ethercat.config_schema import EthercatNetworkConfig, DriveConfig, XmlConfig, RuckigConfig

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
            ruckig=RuckigConfig(
                enabled=True,
                max_velocity=20000,
                max_acceleration=50000,
                max_jerk=200000,
                velocity_lookahead_s=0.5,
            ),
        )
    ],
)
```

### Command
```python
# Start a position move (S-curve) while in CSP
servo0.ruckig_move_to(100000)

# Start/adjust a velocity command (smooth accel/jerk), also via CSP streaming
servo0.ruckig_set_velocity(5000)

# Stop (hold position)
servo0.ruckig_stop()
```
