# AS715N Servo Drive: V1 to V2 Implementation Nuances

## Overview
This document captures every minute detail, workaround, jank fix, and nuance required to get the AS715N (STEPPERONLINE A6) servo drive working correctly from the main application through to the EtherCAT driver layer.

**Model:** AS715N (STEPPERONLINE A6 Servo)  
**Slave Position:** 0  
**XML File:** `STEPPERONLINE_A6_Servo_V0.04.xml`  
**Purpose:** Linear positioning axis (cutting head carriage movement along tube length)

---

## 1. Custom PDO Mapping Requirements

### Critical Discovery: Modes of Operation Must Be updated and monitored every cycle. just like other PDO values. 

**V1 Implementation:**
```python
as715n_custom_pdos = {
    'rx_entries': [
        (0x6040, 0, 16),  # Controlword
        (0x607A, 0, 32),  # Target Position
        (0x60FF, 0, 32),  # Target Velocity ✨
        (0x60B8, 0, 16),  # Touch Probe Function ✨
        # NOTE: 0x6060/0x6061 NOT included
    ],
    'tx_entries': [
        (0x6041, 0, 16),  # Statusword
        (0x6064, 0, 32),  # Position Actual
        (0x606C, 0, 32),  # Velocity Actual ✨
        (0x60B9, 0, 16),  # Touch Probe Status ✨
        (0x60BA, 0, 32),  # Probe-1 Positive Edge ✨
        (0x60BB, 0, 32),  # Probe-1 Negative Edge ✨
        (0x60FD, 0, 32),  # Digital Inputs ✨
        (0x603F, 0, 16),  # Error Code
    ]
}
```

**V2 Requirement:** ALL PDO values should dynamically create managers. ie for mode switching if 6060 is in SDO, use SDO. if 6060 is in PDO we should write to the handler of 6060 to set the value to the definition for whatever mode we want. the handler doesnt need to be smart about what its handling we just need to ensure the PDO value stays every cycle for items like this. 

whereas control word does need a smart handler to cycle the start movement command (bit 4) properly and so on.

**Nuance:** Mode readback (0x6061) is also NOT in PDO, so mode verification must use SDO reads if 0x6061 is not in PDO - in some cases its possible and should be handled that 0x6060 is in PDO but 0x6061 is not. 

---

## 2. DC Synchronization Requirements

### AS715N Requires DC Sync
**Issue:** AS715N has a 9-second SafeOpOpTimeout. If DC synchronization is not configured, it blocks the entire network from reaching OP state, causing ~8 second delays.

**V1 Implementation:**
```python
drive0 = CiA402Drive(
    network=network,
    slave_position=0,
    xml_file=servo_xml,
    use_pdo=True,
    rx_pdos=[0x1600],
    tx_pdos=[0x1A00],
    enable_dc=True,    # Required for AS715N
    custom_pdo_config=as715n_custom_pdos
)
```

**Critical Timing:**
- Cycle time MUST be set BEFORE configuring drives: `network._cycle_time = 5.0 / 1000.0`
- Initial application time MUST be set BEFORE master activation: `master.set_application_time(initial_time_ns)`
- DC application time must be maintained continuously via cyclic task

**V2 Requirement:** 
- DC sync must be configured during slave setup
- Application time must be set before activation
- Application time must be updated every cycle (or periodically) in the cyclic task

---

## 3. Unit Conversion: 1040.42 Pulses per User Unit

### Critical Conversion Factor
**Issue:** AS715N operates in raw pulses internally, but application needs user units (0.01 inch increments).

**V1 Implementation:**
```python
# Configure unit conversion BEFORE connect()
drive0.set_unit_conversion(units_per_pulse=1.0, scale_factor=1040.42)
# Result: 1040.42 pulses = 1 user unit = 0.01 inch
# Example: 104042 pulses = 100 user units = 1.00 inch
```

**Timing:** Unit conversion MUST be configured BEFORE calling `connect()`. The drive stays in pulses internally; driver handles all conversion.

**V2 Requirement:**
- All position/velocity commands must be converted from user units to pulses
- All position/velocity feedback must be converted from pulses to user units
- Conversion factor: set dynamically per application
- This should be set-able at any time. if we update this the class varibables should update then the movement functions should immediatly use the new values. user app side will handle ensuring the proper conversion is set before trying to use the drive in User Units

---

## 4. Mode Switching: SDO-Only Operation

### Mode Changes Require SDO Writes
**Issue:** Since 0x6060 is not in PDO, mode switching must use SDO writes.

**V1 Implementation:**
```python
# Mode switching is non-blocking - queues SDO write
drive.set_position_mode(safe_switch=False)  # Writes 0x6060 = 1 via SDO
drive.set_velocity_mode(safe_switch=False)  # Writes 0x6060 = 3 via SDO
drive.set_csp_mode(safe_switch=False)       # Writes 0x6060 = 8 via SDO

# Verification uses SDO read of 0x6061
mode = drive.read_parameter(0x6061)  # Mode Display
```

**Critical Timing:**
- Mode write is queued immediately (non-blocking)
- Mode verification happens in background via cyclic task or polling
- Mode display (0x6061) may lag behind mode command (0x6060) by 1-2 cycles

**V2 Requirement:**
- Mode commands must queue SDO writes (not PDO writes)
- Mode verification must use SDO reads of 0x6061
- Driver must handle mode verification asynchronously (non-blocking)

---

## 5. Touch Probe: Critical Disable-Before-Re-Arm Sequence

### Most Critical Workaround: Probe Register Must Be Cleared
**Issue:** Touch probe function (0x60B8) on AS715N requires explicit disable (0x0000) before changing edge detection types or re-arming. The drive will reject new probe configurations if the register is not cleared first.

**Observed Behavior:**
- ✅ Negative edge (0x0021): Always worked on first attempt
- ❌ Positive edge (0x0011): Always failed - drive returned readback 0x0000
- ✅ Test mode: Both edges worked because there was sufficient time/state between operations

**V1 Implementation:**
```python
# Before arming probe, explicitly disable it first
self._probe_function = None  # Stop cyclic task from writing
probe_key = (self.slave_position, 0x60B8, 0)
if probe_key in self.network.pdo_offsets:
    self.network.write_pdo(self.slave_position, 0x60B8, 0x0000, 0, size=2)
    time.sleep(0.05)  # Give drive firmware time to process disable

# Then arm with new configuration
self._probe_function = probe_value  # 0x0011 or 0x0021
```

**V2 Requirement:**
1. Always write 0x0000 to disable probe before arming with new configuration
2. Add 50ms delay after disable to allow drive firmware to process
3. Stop cyclic task from maintaining old value during transition
4. This applies to ALL edge type changes, not just positive edge
5. Consider this a vendor-specific requirement that may apply to other drives

**Probe Function Values:**
- 0x0000: Disabled
- 0x0011: Enabled, single-shot, positive edge (bit 0 + bit 4)
- 0x0021: Enabled, single-shot, negative edge (bit 0 + bit 5)
- 0x0031: Enabled, single-shot, both edges (bit 0 + bit 4 + bit 5)

**Probe Register Map:**
- 0x60B8: Touch Probe Function (write)
- 0x60B9: Touch Probe Status (read-only, indicates trigger state)
- 0x60BA: Probe 1 Positive Edge Position (captured position on rising edge)
- 0x60BB: Probe 1 Negative Edge Position (captured position on falling edge)
- 0x2004:11: DI5 Function Assignment (set to 30 for Probe 1 input on AS715N)

---

## 6. Profile Configuration: User Units vs Pulses

### Profile Parameters Must Be in User Units
**Issue:** Profile velocity and acceleration are configured in user units, but drive stores them in pulses.

**V1 Implementation:**
```python
# Profile values in USER UNITS (driver converts internally)
drive0.connect(
    profile_velocity=500,      # 500 units/s = 5 in/s
    profile_acceleration=1000, # 1000 units/s² = 10 in/s²
    operation_mode=1,
    # ... other params
)
```

**V2 Requirement:**
- Profile velocity/acceleration must be converted from user units to pulses before writing to drive
- Conversion: `pulses_per_s = user_units_per_s * 1040.42`
- Drive stores values in pulses internally

---

## 7. Homing Configuration: Method 1 (Negative Limit Switch)

### Homing Parameters in User Units
**Issue:** Homing configuration must be passed during `connect()` to write in PREOP state.

**V1 Implementation:**
```python
homing_config={
    'method': 1,          # Method 1: Negative limit switch with index pulse
    'search_vel': 400,     # 400 user units/s = 4.0 in/s
    'zero_vel': 50,        # 50 user units/s = 0.5 in/s  
    'accel': 1000,         # 1000 user units/s² = 10 in/s²
    'offset': 99,          # 99 user units = 0.99 inches
    'unit': 'native'       # Using user units (same as profile params)
}
```

**V2 Requirement:**
- Homing config must be written during PREOP state (before activation)
- All homing parameters must be converted from user units to pulses
- Method 1 uses negative limit switch with index pulse

---

## 8. Rotation Direction: REVERSED

### Motor Direction Must Be Reversed
**Issue:** Physical motor direction must be reversed for correct linear movement direction.

**V1 Implementation:**
```python
drive0.connect(
    # ... other params
    rotation_direction='reverse',  # Reverse motor direction
    # ...
)
```

**V2 Requirement:**
- Rotation direction must be configured during PREOP state
- Parameter: 0x607E (Polarity) or drive-specific parameter
- Must be set BEFORE activation

---

## 9. Inertia Ratio: 3:1 (DO NOT CHANGE)

### Critical Inertia Ratio
**Issue:** Inertia ratio must be set to 3:1 for proper servo tuning.

**V1 Implementation:**
```python
drive0.connect(
    # ... other params
    inertia_ratio=3,  # 3:1 load/motor inertia DO NOT CHANGE THIS VALUE
    # ...
)
```

**V2 Requirement:**
- Inertia ratio must be configured during PREOP state
- Value: 3 (3:1 ratio)
- This is a critical tuning parameter - do not change

---

## 10. Software Position Limits: DISABLED

### Physical Sensors Used Instead
**Issue:** Software limits are disabled; physical POT/NOT sensors are used instead.

**V1 Implementation:**
```python
drive0.connect(
    # ... other params
    position_limits=(0, 0, 'disable')  # Disable software limits (use POT/NOT sensors)
)
```

**V2 Requirement:**
- Software position limits must be disabled during PREOP state
- Physical limit switches (POT/NOT) are used for position limits
- Parameter: 0x607D (Software Position Limit) or drive-specific

---

## 11. Maximum Velocity Limit: Safety Limit

### Hard Velocity Limit for Safety
**Issue:** Maximum velocity limit prevents hitting physical endstops.

**V1 Implementation:**
```python
# Set hard velocity limit for safety (prevents hitting physical endstops)
drive0.set_max_velocity(500, unit='native')  # 500 units/s = 5 in/s max
```

**V2 Requirement:**
- Maximum velocity limit must be set after drive is enabled
- Value: 500 user units/s = 5.00 in/s (safety limit)
- Drive cannot exceed this speed (safety limit for deceleration)
- Parameter: 0x607F (Max Profile Velocity) or drive-specific

---

## 12. Position Move Completion: Bit 4 Cycling Workaround

### Critical Workaround: Bit 4 Must Be Cycled for Position Moves
**Issue:** Position moves sometimes stall and require cycling controlword bit 4 (new set-point) to trigger movement.

**V1 Implementation:**
```python
# CRITICAL: Wait for the position move with aggressive bit 4 cycling
position_reached = False
max_total_time = 5.0  # 5 seconds total max wait
max_stall_time = 0.5  # If no movement for 0.5s, cycle bit 4
position_tolerance = 50  # 0.5 inches tolerance
bit4_cycle_interval = 0.3  # Cycle bit 4 every 300ms if not moving

while time.time() - start_time < max_total_time:
    current_pos = drive.get_position(unit='native')
    position_error = abs(current_pos - target_position_uu)
    
    # Check if position is moving
    if last_position is not None:
        position_change = abs(current_pos - last_position)
        if position_change > 5:  # Moved at least 0.05 inches
            last_movement_time = time.time()
    
    # Check if we reached target
    if position_error < position_tolerance:
        position_reached = True
        break
    
    # If position hasn't moved for max_stall_time, cycle bit 4
    time_since_movement = time.time() - last_movement_time
    if time_since_movement > max_stall_time:
        # Cycle bit 4: Clear it, then set it again to trigger move
        control_word = drive.read_parameter(0x6040)
        control_word_clear = control_word & ~(0x0010 | 0x0100)  # Clear bit 4 and HALT
        drive.write_parameter(0x6040, control_word_clear)
        time.sleep(0.02)  # Brief pause
        
        # Set bit 4 again (new setpoint)
        control_word_set = control_word_clear | 0x0010 | 0x0020  # bit 4 + bit 5
        drive.write_parameter(0x6040, control_word_set)
        bit4_cycles += 1
        last_movement_time = time.time()  # Reset stall timer
```

**V2 Requirement:**
- Position move completion detection must monitor actual position change
- If position stalls for >0.5s, cycle controlword bit 4 (new set-point)
- Cycle interval: 300ms minimum between bit 4 cycles
- Maximum wait time: 5 seconds total
- Position tolerance: 50 user units (0.5 inches)

---

## 13. Mode Verification: Delayed Readback

### Mode Display Lags Behind Mode Command
**Issue:** Mode display (0x6061) may lag behind mode command (0x6060) by 1-2 cycles.

**V1 Implementation:**
```python
# Mode verification with retries
mode_verified = False
for attempt in range(10):  # Try for up to 1 second
    time.sleep(0.1)
    current_mode = drive.read_parameter(0x6061)  # Mode display
    if current_mode == 3:  # PV mode
        mode_verified = True
        break
```

**V2 Requirement:**
- Mode verification must poll 0x6061 with retries
- Retry interval: 100ms
- Maximum retries: 10 (1 second total)
- Mode display may lag by 1-2 cycles

---

## 14. Digital Inputs: Bit 20 (DI5) for Probe Sensor

### Probe Sensor on Digital Input Bit 20
**Issue:** Probe sensor is connected to DI5 (bit 20) of digital inputs register (0x60FD).

**V1 Implementation:**
```python
# Read DI register (0x60FD - 32-bit, now in PDO)
di_value = drive.network.read_pdo(drive.slave_position, 0x60FD, 0, size=4) or 0

# Check bit 20 (DI5 - probe sensor)
tube_present = bool(di_value & 0x00100000)  # Bit 20
```

**V2 Requirement:**
- Digital inputs register (0x60FD) must be in PDO mapping
- Probe sensor state is bit 20 (0x00100000)
- DI5 Function Assignment (0x2004:11) must be set to 30 for Probe 1 input

---

## 15. Bidirectional Probe Detection: Complex State Machine

### Complex Probe Logic with Multiple Edge Cases
**Issue:** Bidirectional probe detection requires complex state machine with multiple edge cases.

**V1 Implementation:** See `lib/servo_extensions.py` - `find_probe_point_bidirectional()`

**Key Nuances:**
1. Check initial probe state (tube present or not)
2. If no tube: Move negative, arm for rising edge
3. If tube present: Move positive, arm for falling edge
4. Handle immediate trigger (sensor already active)
5. Handle oversized tube detection (>5550 units)
6. Handle position >5400 units (reduce velocity)
7. Handle POT sensor hit before probe trigger
8. Mode switching with verification
9. Position move with bit 4 cycling
10. Offset application after probe detection

**V2 Requirement:**
- All probe logic must be implemented in v2 driver or application layer
- Complex state machine with multiple edge cases
- Multiple mode switches with verification
- Position moves with bit 4 cycling workaround

---

## 16. Repeating Commands: Mode Switches and Probe Arming

### Commands Must Be Repeated Until Verified
**Issue:** Some commands (mode switches, probe arming) must be repeated until verification succeeds.

**V1 Implementation:**
```python
# Mode switch with retries
max_mode_attempts = 5
for attempt in range(1, max_mode_attempts + 1):
    drive.set_position_mode(safe_switch=False)
    # Wait and verify mode change
    timeout = 2.0
    start_time = time.time()
    while time.time() - start_time < timeout:
        actual_mode = drive.read_parameter(0x6061)
        if actual_mode == 1:  # PP mode
            mode_changed = True
            break
    if mode_changed:
        break
    time.sleep(0.2)  # Wait before retry
```

**V2 Requirement:**
- Mode switches must be retried up to 5 times
- Each retry must wait up to 2 seconds for verification
- 200ms delay between retries
- Probe arming must be retried up to 5 times (PROBE_ARM_MAX_RETRIES)

---

## 17. Error Handling: Fault Detection and Recovery

### Drive Fault Detection and Recovery
**Issue:** Drive faults must be detected and cleared before operations can continue.

**V1 Implementation:**
```python
# Check and clear errors first
if not drive.check_and_clear_error():
    logger.error("Drive has errors that could not be cleared")
    return False

# Check for drive errors during motion
if not drive.check_and_clear_error():
    logger.error("Drive error detected during motion")
    drive.set_velocity(0, unit='native')
    time.sleep(0.1)
    drive.disable_probe()
    return False
```

**V2 Requirement:**
- Error checking must happen before critical operations
- Error checking must happen during motion (periodic)
- Fault recovery: shutdown → clear faults → ready drive
- Error code reading: 0x603F (Error Code register)

---

## 18. Status Word Monitoring: State Machine Tracking

### Status Word Bits Must Be Monitored
**Issue:** Status word (0x6041) contains critical state information that must be monitored.

**V1 Implementation:**
```python
status = drive.get_status_word() or 0

# Check for fault state
if (status & 0x000F) == 0x0008:  # Fault state
    error_code = drive.read_parameter(0x603F)

# Check for warning
has_warning = bool(status & 0x0080)  # Bit 7: Warning
```

**V2 Requirement:**
- Status word must be read every cycle (via PDO)
- Fault state detection: (status & 0x000F) == 0x0008
- Warning detection: (status & 0x0080) != 0
- State machine tracking via status word bits

---

## Summary: Critical Requirements for V2

1. **Custom PDO mapping** - Modes NOT in PDO, must use SDO
2. **DC synchronization** - Required, 9-second timeout
3. **Unit conversion** - 1040.42 pulses = 1 user unit
4. **Mode switching** - SDO-only, with verification retries
5. **Probe disable-before-re-arm** - Critical 50ms delay workaround
6. **Profile configuration** - User units converted to pulses
7. **Homing configuration** - Method 1, user units
8. **Rotation direction** - REVERSED
9. **Inertia ratio** - 3:1 (DO NOT CHANGE)
10. **Software limits** - DISABLED (use physical sensors)
11. **Max velocity limit** - 500 units/s safety limit
12. **Bit 4 cycling** - Position move completion workaround
13. **Mode verification** - Delayed readback with retries
14. **Digital inputs** - Bit 20 for probe sensor
15. **Bidirectional probe** - Complex state machine
16. **Repeating commands** - Mode switches and probe arming
17. **Error handling** - Fault detection and recovery
18. **Status word monitoring** - State machine tracking

**All of these nuances must be handled cleanly in V2 to ensure reliable operation.**

