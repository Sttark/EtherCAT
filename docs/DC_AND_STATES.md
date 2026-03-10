# EtherCAT DC Configuration and State Management

## DC (Distributed Clocks) Configuration

### What is `dc_assign_activate`?

`dc_assign_activate` is a 16-bit register value that controls which SYNC signals are enabled for the slave device.

**Common values:**
- `0x0000`: DC disabled (Freerun mode)
- `0x0300`: SYNC0 enabled (most common for cyclic synchronous operation)
- `0x0700`: SYNC0 + SYNC1 enabled (for dual-rate applications)

**Bit breakdown:**
- Bits 0-7: SYNC unit assignment
- Bits 8-15: SYNC signal activation
  - Bit 8 (0x0100): Reserved
  - Bit 9 (0x0200): Activate SYNC0
  - Bit 10 (0x0400): Activate SYNC1

**For AS715N Servo:**
- Requires DC synchronization (does not support freerun)
- Standard config: `0x0300` (SYNC0 enabled)
- SYNC0 cycle time = network cycle time (e.g., 5ms = 5,000,000 ns)

### DC Configuration Parameters

```python
DriveConfig(
    enable_dc=True,                    # Enable DC for this slave
    dc_assign_activate=0x0300,         # SYNC0 enabled
    dc_sync0_cycle_time_ns=None,       # Auto: uses network cycle_time_ms
    dc_sync0_shift_ns=0,               # Phase shift for SYNC0
    dc_sync1_cycle_time_ns=0,          # SYNC1 cycle (0 = disabled)
    dc_sync1_shift_ns=0,               # Phase shift for SYNC1
)
```

## EtherCAT AL (Application Layer) States

### State Values
- `0` or `1`: **INIT** - Slave is initializing
- `2`: **PREOP** - Pre-operational (configuration phase)
- `3`: **SAFEOP_TRANSITION** - Transitioning to SAFEOP (or error)
- `4`: **SAFEOP** - Safe operational (PDOs configured, not cycling yet)
- `8`: **OP** - Operational (full cyclic PDO exchange active)

### State Transitions
```
INIT → PREOP → SAFEOP → OP
  ↓      ↓        ↓      ↓
  ← ← ← ERROR ← ← ← ← ← ←
```

### Why Slave Stays in SAFEOP

**Common reasons:**
1. **DC not configured** - Slave requires DC but it's not enabled
   - Error: "Freerun not supported"
   - Solution: Set `enable_dc=True` and configure DC
   
2. **DC time not set** - Application time not initialized before activation
   - Solution: Call `set_application_time()` before `activate()`
   
3. **Wrong PDO configuration** - PDO mapping doesn't match slave requirements
   - Check XML file for supported PDO configurations
   
4. **DC reference clock not selected** - No reference clock chosen
   - Solution: Call `select_reference_clock()` for DC-enabled slave

## CiA 402 Status Word (0x6041)

### Statusword 0x8218 Decoding

```
Binary: 1000 0010 0001 1000
        │    │  │ │  │ │││└─ Bit 0: Ready to switch on = 0
        │    │  │ │  │ ││└── Bit 1: Switched on = 0
        │    │  │ │  │ │└─── Bit 2: Operation enabled = 0
        │    │  │ │  │ └──── Bit 3: Fault = 1 ⚠️
        │    │  │ │  └─────── Bit 4: Voltage enabled = 1
        │    │  │ └────────── Bit 9: Remote = 1
        │    │  └───────────── Bit 13: ? = 1
        └──────────────────── Bit 15: Internal limit = 1
```

**0x8218 = FAULT STATE**

The drive is in fault and needs:
1. Read error code (0x603F)
2. Clear fault condition
3. Send FAULT_RESET control word (0x0080)
4. Run CiA 402 state machine to enable

## CiA 402 State Machine

To enable a drive from FAULT or power-on:

```python
# State machine sequence
if state == FAULT:
    send_control_word(0x0080)  # FAULT_RESET
    wait_for_state(SWITCH_ON_DISABLED)

if state == SWITCH_ON_DISABLED:
    send_control_word(0x0006)  # SHUTDOWN
    wait_for_state(READY_TO_SWITCH_ON)

if state == READY_TO_SWITCH_ON:
    send_control_word(0x0007)  # SWITCH_ON
    wait_for_state(SWITCHED_ON)

if state == SWITCHED_ON:
    send_control_word(0x000F)  # ENABLE_OPERATION
    wait_for_state(OPERATION_ENABLED)
```

**Next step for this driver:** Implement automatic CiA 402 state machine in the cyclic process!




