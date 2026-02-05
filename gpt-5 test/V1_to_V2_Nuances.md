# V1 → V2 Driver Nuances (Process- and Task-Oriented Comparison)

This document captures every detail, workaround, and operational nuance needed to make the servo “just work,” organized by the tasks each layer performs from the main application down to the EtherCAT driver. It is written to be executed against V2 so we can test and implement each item cleanly. It deliberately avoids line-by-line code diffs in favor of process-level behaviors, per your preference [[memory:6561503]].

Model folder: gpt-5 test  
Device focus: AS715N (STEPPERONLINE A6) servo at slave 0; YAKO stepper at slave 1 where relevant

---

## 1) Master lifecycle and DC sync (required ordering)
- What V1 does (MachineController + test scripts):
  - Release any stuck master before requesting.
  - Request master in full PDO mode when cyclic exchange is needed.
  - Create domain prior to slave/PDO configuration.
  - Set application/DC time before activation; maintain time during cyclic.
  - Activate master after all configuration is finished.
- What V2 must ensure:
  - Same ordering. Set application time before activation and maintain it every cycle.
  - If a “stuck master” can happen, expose an explicit “release_master_if_reserved” preflight or document how to run it safely before the process starts.

Relevant V2 gap/notes:
- V2 updates application time each cycle (good); still ensure initial set before activation.
- Consider adding a graceful release at process teardown and an optional preflight release path.

---

## 2) PDO mapping for AS715N (fault-avoidance rule)
- Observed constraint (V1): Mapping 0x6060 (Modes of Operation) and/or 0x6061 (Modes Display) into PDO caused immediate faults on AS715N.
- V1 solution: Keep modes out of PDO. Put controlword, targets, probe, DI, statusword, pos/vel actual in PDO.
  - RX PDO must include: 0x6040, 0x607A, 0x60FF, 0x60B8
  - TX PDO should include: 0x6041, 0x6064, 0x606C, 0x60B9, 0x60BA, 0x60BB, 0x60FD, 0x603F
- V2 requirement:
  - If modes are not in PDO, write 0x6060 via SDO and read 0x6061 via SDO for verification. This fallback must be automatic.
  - Continue to keep 0x6060/0x6061 out of PDO for AS715N.

---

## 3) Mode switching (repeat-until-verified behavior)
- Behavior (V1):
  - Non-blocking SDO write to 0x6060; verification polls 0x6061 until it matches the requested mode or a timeout.
  - Retry up to 5 times with ~200 ms between retries.
  - Mode display (0x6061) may lag 1–2 cycles; code tolerates this.
- V2 requirement:
  - Queue the intent; cyclic task writes mode via SDO if 0x6060 is not in PDO.
  - Provide non-blocking verification loop (polled from process with cached status) that tolerates readback lag and retries intelligently.
  - Keep mode “sticky”: the cyclic writer should reapply mode until verified, then leave it alone unless changed.

Minute V2 discrepancy:
- `process_manager` writes mode every cycle whenever `last_mode_cmd` is set. That’s acceptable but should clear or enter a “verified” state when 0x6061 matches to avoid gratuitous re-writes.

---

## 4) Touch probe arming (disable-before-re-arm invariant)
- Critical V1 invariant on AS715N:
  - Must write 0x0000 to 0x60B8 (disable) and wait ~50 ms before arming with a new edge setting; otherwise, the new arm may be ignored and readback returns 0.
- Values mismatch to note:
  - V1: Positive=0x0011, Negative=0x0021 (enable + latched edge bits pattern)
  - V2 constants currently use Positive=0x0005, Negative=0x0009 (enable=0x0001 + edge bit).
  - Action: Pick one encoding and make it consistent across driver and docs; ensure disable-before-re-arm sequence is implemented in V2.
- V2 requirement:
  - Implement a single-cycle “disable (0), wait 50 ms, then write desired value” sequence inside the cyclic writer when probe arming changes.
  - Ensure the cyclic writer does not immediately overwrite during transition (pause maintenance during that window).

---

## 5) Controlword bit 4 cycling for PP moves (stall workaround)
- V1 behavior:
  - For PP moves, pulse bit 4 (NEW SET-POINT) to trigger motion, and if position stalls (>0.5 s without movement), clear and set bit 4 again to kick the move.
  - Max total wait about 5 s; limit bit-4 cycles to every ~300 ms minimum.
- V2 requirement:
  - Only set bit 4 on actual target changes; detect stall via position delta and cycle bit 4 if stalled per the above rules.
  - Current V2 always sets bit 4 if a position command exists; refine to event-driven pulses and stall recovery logic.

---

## 6) Unit conversion (pulses ↔ user units)
- V1: Configure conversion early and convert all targets/feedback (AS715N: 1040.42 pulses = 1 UU = 0.01 inch).
- V2 today:
  - `DriveConfig.unit_conversion` exists but conversion is not yet applied in the process loop.
  - Requirement: Apply conversion uniformly:
    - On write: UU → pulses (0x607A, 0x60FF, profiles, homing params).
    - On read: pulses → UU (0x6064, 0x606C, probe latched positions).
  - Note: Conversion must be done in the process (real-time side), not left to application.

---

## 7) Error handling (fault-first discipline)
- V1:
  - Always check and clear fault before operations; read 0x603F on fault; stop motion and disarm probe on errors during motion.
- V2 requirement:
  - Add proactive periodic fault checks in the process and publish error code via status.
  - Provide a “clear fault” command path that performs shutdown → clear faults → ready sequence.

---

## 8) Homing (method 1: negative limit + index)
- V1:
  - Homing parameters are written in PREOP, then homing is started (either via mode=HM + pulse or by drive specifics). Values are in UU and converted to pulses.
- V2 today:
  - `START_HOMING` queues intent; process sets `MODE_HM` and piggybacks the PP strobe.
  - Gaps:
    - `MODE_HM` is used in `process_manager` but not imported (NameError risk).
    - Homing parameters conversion/writes not yet implemented.
  - Requirement: Implement full homing state machine and conversion; import `MODE_HM` explicitly.

---

## 9) IRQ threading, RT scheduling, and cgroups (platform hygiene)
- V1 (MachineController) configures:
  - NIC IRQ threads affinity to the isolated RT core with SCHED_FIFO priority higher than cyclic task.
  - Optional SCHED_DEADLINE for the cyclic thread (budgeted), otherwise SCHED_FIFO.
  - cgroups isolation to avoid priority inversion and RT throttling conflicts.
- V2 approach:
  - Keep the EtherCAT loop isolated in its own process (great).
  - Requirements:
    - Document external system configuration (kernel threadirqs, IRQ affinity, RT priority).
    - Optionally expose helper scripts/APIs in V2 to apply IRQ/cgroup tuning if desired.

---

## 10) Vendor/Product identification and XML
- V1:
  - Device identity known; pyethercat config expects correct vendor/product.
- V2 today:
  - `XmlConfig` is present but `xml_decoder` does not extract vendor_id/product_code.
  - `process_manager` currently requires vendor/product or returns error; some tests pass 0x0 which may be master-accepted but is not portable.
  - Requirement: Parse vendor_id and product_code from ESI and fill them into `DriveConfig` automatically.

---

## 11) Master adapter behavior (read/write accessors)
- V1 pyethercat:
  - Provides `read_domain_data(offset, size)` and `write_domain_data(offset, bytes)` helpers.
- V2 today:
  - `ethercat_v2.igh_master.Master` (direct ctypes) lacks `read_domain`/`write_domain` helpers but `process_manager` calls them. The alternate `IGHMasterAdapter` class (wrapper around pyethercat) does expose them.
  - Requirement: Either:
    - Use `IGHMasterAdapter` wrapper in `process_manager`, or
    - Implement `read_domain`/`write_domain` in the local `Master` wrapper (by caching the domain pointer and slicing).

This is a correctness gap to close before broader testing.

---

## 12) Status publication and “PDO health”
- V2 publishes:
  - statusword, mode_display, position_actual, velocity_actual, probe fields, digital inputs, feature flags, and a “pdo_health” map.
- Requirements:
  - Confirm signatures (e.g., `PROBE_POS2_INDEX` uses 0x60BC in constants but V1 used 0x60BB for negative edge; ensure the chosen indices match the drive’s ESI).
  - Ensure endianness and signedness are correct for each field.

---

## 13) Repeaters and “jank fixes” to preserve
Keep these semantics in V2:
1. Mode switch retries with SDO readback verification.
2. Probe disable-before-re-arm with 50 ms delay.
3. Bit-4 cycling on PP stall with conservative intervals.
4. Pre-op writes for motion parameters and homing config.
5. DC timing set before activation; maintained each cycle.
6. Fault-first checks before critical motion; immediate stop and disarm on error.

---

## 14) Known minute discrepancies requiring action
- MODE_HM used in process without import (NameError risk) – import it from `constants`.
- `Master` wrapper missing `read_domain`/`write_domain` while process expects them.
- Probe function encoding differs between V1 and current V2 constants – pick one and unify.
- `xml_decoder` does not extract vendor/product; `process_manager` requires them.
- Unit conversion not yet applied in process reads/writes (positions, velocities, probe latches).
- Stall detection and PP bit-4 cycling logic not implemented yet.

---

## 15) Test checklist (for V2 acceptance)
- Power-on to OP:
  - [ ] Master request, domain create, DC application time set before activation.
  - [ ] Activation succeeds; application time maintained cyclically.
- Modes:
  - [ ] SDO-only 0x6060 writes; 0x6061 verification with lag tolerance and retries.
  - [ ] No faults when 0x6060/0x6061 excluded from PDO.
- Motion:
  - [ ] PP moves trigger with bit-4 pulse; stall detection cycles bit-4 safely.
  - [ ] PV velocity holds steady; set to 0 on stop.
  - [ ] CSP streams position each cycle.
- Probe:
  - [ ] Disable-before-re-arm sequence is performed; both edges latch.
  - [ ] DI bit mapping correct; probe latched positions readable and converted.
- Safety:
  - [ ] Max velocity limit applied and enforced.
  - [ ] Faults detected and cleared through a defined sequence.
- Conversion:
  - [ ] All numeric I/O converted consistently (UU↔pulses).


