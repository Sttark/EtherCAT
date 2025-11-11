# V2 Isolated Process Architecture: Detailed Report and Minute Differences

This document details the design of the V2 isolated EtherCAT process and catalogs minute discrepancies to close so the application can “just call easy functions that work.”

Files reviewed:
- `ethercat_v2/process_manager.py`
- `ethercat_v2/client.py`
- `ethercat_v2/cia402/driver.py`
- `ethercat_v2/constants.py`
- `ethercat_v2/xml_decoder.py`
- `ethercat_v2/master_adapter.py`

---

## 1) Architecture overview
- Process separation:
  - Application process: enqueues commands, reads status (non-RT safe).
  - EtherCAT process (daemon): owns master handle; runs cyclic I/O and state maintenance.
- Communication:
  - `multiprocessing.Queue` for commands (size 1024) and status (size 64).
  - Client injects `_enqueue_command` and `_read_status` closures into `CiA402Drive` handle for a clean separation.
- Cycle:
  - Pump up to 16 commands per iteration.
  - If PDO is enabled:
    - Optionally set application time.
    - receive → domain process → cyclic write (apply intents) → queue → send.
  - Publish aggregated status every ~50 ms.

Benefits:
- Real-time isolation and failure isolation; the app remains responsive even if the EtherCAT process misbehaves.

---

## 2) Command intent model
- The process maintains “last intent” per slave:
  - `last_mode_cmd`, `last_velocity_cmd`, `last_position_cmd`, `last_probe_arm`.
- The cyclic writer realizes intent by writing PDO values or falling back to SDO if the PDO entry is missing.
- Probe arm is treated as write-once and cleared to prevent continuous re-arming.

Minute enhancement:
- Mark mode “verified” when 0x6061 readback matches the requested mode, to stop writing mode every cycle.

---

## 3) Status publication
- Status includes `statusword`, `mode_display`, `position_actual`, `velocity_actual`, probe fields (pos1/pos2/active), `digital_inputs`, `features`, and a “pdo_health” map indicating present/missing objects.
- Ensure:
  - Endianness is little-endian for numeric reads.
  - Signedness correct for position/velocity (signed 32-bit).
  - Probe indices (0x60BA positive, 0x60BC negative) match device ESI. If the drive uses 0x60BB for negative, make that configurable/auto-detected.

---

## 4) Critical minute discrepancies and gaps to close
1) Direct master adapter method mismatch:
   - `process_manager` imports `Master as IGHMasterAdapter` (direct ctypes class) but calls `read_domain`/`write_domain`, which are NOT implemented on that class.
   - Fix by either switching to the `IGHMasterAdapter` wrapper (which maps to pyethercat and exposes those methods) or by implementing `read_domain`/`write_domain` in the direct wrapper.

2) MODE_HM import:
   - `process_manager` uses `MODE_HM` but does not import it from `constants`. This is a potential NameError at runtime; import it explicitly.

3) Unit conversion:
   - `DriveConfig.unit_conversion` exists, but conversion is not applied for targets/feedback in the process.
   - Implement UU↔pulses conversion centrally in the process write/read paths to keep the app free from conversion logic.

4) Homing implementation:
   - `START_HOMING` sets `MODE_HM` and repurposes PP strobe, but homing parameter writes (converted to pulses) are not performed, and the state machine is not implemented.
   - Implement a homing sub-state machine with timeouts and success/failure publication.

5) Probe arm sequence:
   - V1 requires disable (0x0000) → delay (~50 ms) → new arm value for AS715N. V2 does not yet implement the “disable-before-re-arm” rule.
   - Add a one-shot transitional state to perform the disable and pause maintenance until the new write is issued.

6) Mode verification:
   - Implement non-blocking verification loop that polls 0x6061 with a bounded retry policy and lag tolerance.
   - Clear `last_mode_cmd` (or set a separate verified flag) once mode_display matches.

7) Bit-4 cycling for PP:
   - Detect stalls via delta of `position_actual` over a short window and cycle bit 4 (with min interval and max cycles).
   - Only set bit 4 when target changes; do not keep it asserted continuously.

8) Vendor/product identity:
   - `xml_decoder` does not provide vendor_id/product_code; `process_manager` requires them. Some tests pass 0x0, which might be accepted but is not robust.
   - Extract identity from ESI and fill `DriveConfig` automatically, or define explicit “wildcard” semantics and document limits.

9) DC sync and reference clock:
   - V2 sets application time each cycle; ensure initial application time is set before activation.
   - Optionally select an explicit reference clock using the bound `ecrt_master_select_reference_clock()`.

10) IRQ/cgroups scheduling:
   - V1 configures IRQ affinity/priority and optional SCHED_DEADLINE/cgroups. V2 should document required system setup or provide helper utilities to apply these settings externally to the process.

---

## 5) CiA402 drive handle semantics (client-side)
- The drive handle is intentionally “thin”: methods enqueue commands; verification is done by the process via status readings.
- `nonblocking_check` decorator (driver layer) uses `_verify_last_action()` which maps to reading `mode_display` or other fields.
Enhancement:
- Extend `_verify_last_action()` to cover homing completion and stalled move detection cues published by the process.

---

## 6) Testing and validation plan
- Bring-up:
  - Initialize process with valid vendor/product, PDO map, and correct DC order (set time before activation).
  - Confirm status publication cadence and content.
- Modes:
  - Issue PP/PV/CSP commands; verify delayed `0x6061` readback within retries.
  - Ensure no faults when modes left out of PDO.
- Motion:
  - PP: target changes pulse bit 4; stall cycles bit 4 with bounded rate.
  - PV: velocity holds and zeroes on stop.
  - CSP: stream target positions; verify smoothness and latency bounds.
- Probe:
  - Exercise disable-before-re-arm sequence; latch both edges; publish latched positions.
  - Confirm DI mapping matches the ESI and actual hardware wiring (bit index for DI5).
- Homing:
  - Verify homing parameter conversion and sequencing; bounded wait with success/failure.
- Faults:
  - Inject faults (e.g., over-command) and verify detection and clear-fault flow.
- Conversion:
  - Validate conversion accuracy round-trip and under max velocities/positions.

---

## 7) Summary of “make it just work” behaviors to implement
- Automatic SDO fallback for modes and targets when not in PDO.
- Non-blocking mode verification with retries and lag tolerance.
- Probe disable-before-re-arm with enforced dwell.
- Event-driven bit-4 pulses and stall recovery for PP.
- Complete UU↔pulses conversion for all relevant objects.
- Homing as a first-class command with its own state machine.
- Adapter method parity (either use pyethercat wrapper or implement domain read/write).
- Explicit import and usage of `MODE_HM` and other constants in the process.

Once these deltas are closed, the application can call simple, high-level actions (move, set velocity, home, arm probe) without re-issuing mode switches or juggling device quirks; the driver will handle it under the hood.


