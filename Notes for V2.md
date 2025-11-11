# Notes for EtherCAT V2 Implementation

This document captures important learnings and requirements discovered during the development of the current EtherCAT implementation that should be incorporated into V2.

---

## Probing

### Critical Discovery: Probe Register Must Be Cleared Before Re-Arming

**Issue:** Touch probe function (0x60B8) on AS715N servo would arm successfully for **negative edge (0x0021)** but consistently fail for **positive edge (0x0011)** with readback of 0x0000.

**Root Cause:** The AS715N drive firmware requires the probe function register to be **explicitly disabled (0x0000)** before changing edge detection types or re-arming. The drive will reject new probe configurations if the register is not cleared first.

**Observed Behavior:**
- ✅ Negative edge (0x0021): Always worked on first attempt
- ❌ Positive edge (0x0011): Always failed - drive returned readback 0x0000
- ✅ Test mode: Both edges worked because there was sufficient time/state between operations

**The Fix:**
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

**Requirements for V2:**
1. Always write 0x0000 to disable probe before arming with new configuration
2. Add 50ms delay after disable to allow drive firmware to process
3. Stop cyclic task from maintaining old value during transition
4. This applies to ALL edge type changes, not just positive edge
5. Consider this a vendor-specific requirement that may apply to other drives

**Additional Findings:**
- Probe function works in both PP (mode 1) and PV (mode 3) - no mode restriction
- Mode verification is still good practice but wasn't the root cause
- The cyclic task correctly maintains probe function via PDO writes
- DI5 configuration (0x2004:11 = 30) is required and working correctly

### Probe Configuration Summary

**Register Map:**
- 0x60B8: Touch Probe Function (write 0x0000 to disable, 0x0011 for positive edge, 0x0021 for negative edge)
- 0x60B9: Touch Probe Status (read-only, indicates trigger state)
- 0x60BA: Probe 1 Positive Edge Position (captured position on rising edge)
- 0x60BB: Probe 1 Negative Edge Position (captured position on falling edge)
- 0x2004:11: DI5 Function Assignment (set to 30 for Probe 1 input on AS715N)

**Edge Detection Bit Values:**
- Bit 0 = 1: Enable Probe 1
- Bit 1 = 0: Single-shot mode (1 = continuous)
- Bit 4 = 1: Latch positive edge (LOW→HIGH)
- Bit 5 = 1: Latch negative edge (HIGH→LOW)

**Standard Values:**
- 0x0000: Disabled
- 0x0011: Enabled, single-shot, positive edge (bit 0 + bit 4)
- 0x0021: Enabled, single-shot, negative edge (bit 0 + bit 5)
- 0x0031: Enabled, single-shot, both edges (bit 0 + bit 4 + bit 5)

---

## Future Sections

- [ ] Mode Switching Best Practices
- [ ] PDO Mapping Requirements
- [ ] Timing and Synchronization
- [ ] Error Handling Strategies
- [ ] Real-Time Performance Tuning

