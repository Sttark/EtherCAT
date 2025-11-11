# V1 vs V2 Quick Reference Guide

## Command Patterns

### Mode Switching

**V1 (BROKEN - requires manual verification):**
```python
# Queue mode change
servo.set_velocity_mode(safe_switch=False)

# MUST manually verify
for i in range(10):
    time.sleep(0.1)
    if servo.read_parameter(0x6061) == 3:  # PV mode = 3
        break
else:
    raise RuntimeError("Mode switch failed")
```

**V2 (CLEAN - automatic verification):**
```python
# Queue mode change - driver maintains until acknowledged
drive.set_velocity_mode(safe_switch=False)

# Status automatically reflects mode when changed
status = manager.get_latest_status()
current_mode = status.drives[slave_id]['mode_display']
```

---

### Position Moves (PP Mode)

**V1 (BROKEN - requires bit-4 cycling):**
```python
servo.set_position_absolute(target, unit='native')

# Wait with MANUAL bit-4 rescue
while not_reached(timeout=5.0):
    if stalled_for_500ms():
        # JANK: Manually cycle bit 4
        cw = servo.read_parameter(0x6040)
        servo.write_parameter(0x6040, cw & ~0x0010)  # Clear
        time.sleep(0.02)
        servo.write_parameter(0x6040, cw | 0x0010)  # Set
```

**V2 (CLEAN - automatic bit-4):**
```python
# Queue position - driver sets bit-4 every cycle
drive.set_position_absolute(target, unit='native')

# Poll status when needed
status = manager.get_latest_status()
current_pos = status.drives[slave_id]['position_actual']
```

---

### Touch Probe

**V1 (BROKEN - requires retry loop):**
```python
# Retry loop required
for attempt in range(5):
    servo.arm_probe(edge='positive')
    if verify_armed():
        break
    time.sleep(0.1)
else:
    raise RuntimeError("Probe arm failed")

# Poll for trigger
while time.time() - start < timeout:
    probe_status = servo.get_probe_status()
    if probe_status['positive_edge_position'] is not None:
        position = probe_status['positive_edge_position']
        break
    time.sleep(0.05)
```

**V2 (CLEAN - automatic maintenance):**
```python
# Queue probe arm - driver maintains until confirmed
drive.arm_probe(edge='positive')

# Status reflects probe state automatically (50Hz updates)
status = manager.get_latest_status()
if status.drives[slave_id]['probe_pos1'] is not None:
    position = status.drives[slave_id]['probe_pos1']
```

---

## Critical Bugs Fixed in V2

### 1. ec_pdo_info_t.n_entries Type Mismatch

**V1 Bug:**
```c
struct ec_pdo_info_t {
    uint16_t index;
    uint8_t n_entries;  // BUG: Should be unsigned int!
    ec_pdo_entry_info_t *entries;
};
```

**Impact:**
- Limited to 255 entries per PDO (uint8 max)
- Memory corruption when kernel expects uint32
- Random crashes

**V2 Fix (master_adapter.py line 82):**
```python
class ec_pdo_info_t(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint16),
        ("n_entries", ctypes.c_uint),  # ✓ FIXED: Now matches kernel
        ("entries", ctypes.POINTER(ec_pdo_entry_info_t)),
    ]
```

---

### 2. ctypes Structure Garbage Collection

**V1 Bug:**
```python
def configure_pdos(slave):
    # Build structures
    sync_array = (ec_sync_info_t * 3)()
    pdo_array = (ec_pdo_info_t * 2)()
    
    # Call kernel
    libec.ecrt_slave_config_pdos(slave._handle, 2, sync_array)
    
    # BUG: Returns, structures can be garbage collected!
    # Kernel holds pointers → dangling pointers → crash
```

**V2 Fix (master_adapter.py lines 455-456):**
```python
def configure_slave_pdos(self, slave_config, sync_configs):
    # ... build structures ...
    
    # CRITICAL: Store to prevent GC
    slave_config._sync_array = sync_array
    slave_config._sync_infos = sync_infos
    
    return True  # Now safe
```

---

## Timing Requirements

| Operation | V1 Timing | V2 Timing | Notes |
|-----------|-----------|-----------|-------|
| Mode Switch | 100-1000ms + manual verify | 100-1000ms, auto-verify | V2 maintains until confirmed |
| Position Move | Variable + bit-4 rescue | Deterministic | V2 handles bit-4 automatically |
| Probe Arm | 50ms × 5 retries | 50ms, single attempt | V2 maintains until armed |
| OP State | 25s timeout, poll 1Hz | 25s timeout, poll 1Hz | Same (AS715N DC sync) |
| Status Update | On-demand (blocking) | 50Hz (non-blocking) | V2 decoupled from app |

---

## Hardware Quirks (Both V1 and V2)

### AS715N Servo
- **Requires DC sync** - 9s SafeopOpTimeout blocks network if not configured
- **0x6060/0x6061 NOT in PDO** - Mode switching causes fault if in PDO
- **Inertia ratio** - 0x2000:07 (vendor-specific, PREOP only)
- **Rotation direction** - 0x607E Polarity (PREOP only on some firmware)

### YAKO Stepper
- **TxPDO is 0x1A01** (not 0x1A00 like most drives)
- **Target-reached unreliable** - Use position-based completion
- **No DC sync** - Pure PDO, no distributed clocks

### Universal
- **Homing MUST be PREOP** - Parameters before activation
- **NIC IRQ priority > Cyclic** - Prevents deadlock
- **Unit conversion in software** - Keep drives in native pulses

---

## Error Patterns

### V1 Error Handling
```python
# Check and clear manually
if servo.is_in_fault():
    fault_code = servo.get_fault_code()
    logger.error(f"Fault: {fault_code:04X}")
    if not servo.clear_faults():
        raise RuntimeError("Cannot clear fault")
    
    # Wait for clear
    for _ in range(10):
        if not servo.is_in_fault():
            break
        time.sleep(0.1)
```

### V2 Error Handling
```python
# Automatic detection and clearing in cyclic task
status = manager.get_latest_status()
if status.drives[slave_id].get('in_fault'):
    fault_code = status.drives[slave_id].get('fault_code')
    logger.error(f"Fault: {fault_code}")
    # Driver auto-clears if configured
    # Status will reflect cleared state on next update
```

---

## Real-Time Configuration

### NIC IRQ Priority (CRITICAL)
```python
# V1 & V2: Must configure NIC IRQ HIGHER than cyclic task
NIC_IRQ_PRIORITY = 55      # Higher priority
CYCLIC_PRIORITY = 45       # Lower priority
ISOLATED_CORE = 2

# Why: If NIC priority ≤ cyclic priority → priority inversion → deadlock
```

### Thread/Process Configuration
```python
# V1: Single threaded
os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(45))
os.sched_setaffinity(0, {2})

# V2: Isolated process
# Process spawns with RT priority in isolated process
# Application can't block RT loop
```

---

## Key Takeaways

1. **V1 works but requires workarounds** - Application compensates for driver issues
2. **V2 eliminates workarounds** - Driver handles everything properly
3. **Success criterion: ZERO retry loops** - If app needs retries, driver failed
4. **Intent-based model** - Commands maintained until acknowledged
5. **Status polling at 50Hz** - Decouple application from RT timing

