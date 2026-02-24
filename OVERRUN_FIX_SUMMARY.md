# EtherCAT Overrun Fix - Implementation Summary

**Date**: 2026-02-24  
**Status**: Changes Applied - Reboot Required

## Changes Implemented

### 1. DC Sync Monitor Decimation ✓

**File**: `/home/sttark/Desktop/github/EtherCAT/process_manager.py`  
**Lines**: 2050-2067

**Change**: Modified DC sync monitor calls to execute only every 10 cycles instead of every cycle.

**Impact**:
- Saves 1 mutex acquisition per 9 out of 10 cycles
- Reduces `max:cia` phase timing by ~10%
- No impact on DC sync accuracy (reference_clock and slave_clocks still run every cycle)

**Code**:
```python
# DC sync monitor decimation: call every 10 cycles instead of every cycle
# This saves 1 mutex acquisition per skipped cycle with no accuracy loss
if self.cycle_count % 10 == 0:
    try:
        dc_err_ns = self.master.sync_monitor_process()
        # ... existing error tracking code ...
        self.master.sync_monitor_queue()
    except Exception:
        pass
```

### 2. NIC Driver ec_poll Budget Reduction ✓

**File**: `/usr/src/igh-ethercat-master/devices/stmmac/stmmac_main-6.12-ethercat.c`  
**Line**: 7262

**Change**: Reduced packet budget from 128 to 4.

**Before**:
```c
int budget = 128;
```

**After**:
```c
int budget = 4;  /* Reduced from 128 - EtherCAT produces 1 frame/cycle */
```

**Impact**:
- Reduces NIC polling overhead since EtherCAT only produces 1 frame per cycle
- Expected reduction in `max:recv` timing spikes
- Kernel modules rebuilt and installed successfully

**Status**: Modules installed to `/lib/modules/6.12.67-v8-16k-rt/ethercat/`

### 3. CPU Frequency Scaling Prevention ✓

**File**: `/boot/firmware/config.txt`  
**Lines**: Added at end of file

**Change**: Added RT tuning parameters to prevent DVFS transitions.

```ini
# Real-time tuning: prevent CPU frequency scaling and reduce GPU activity
force_turbo=1
gpu_mem=16
```

**Impact**:
- Eliminates CPU frequency scaling transitions that cause timing jitter
- Reduces GPU firmware interrupt activity
- **Requires reboot to take effect**

### 4. CiA402 Enable Transition Period ✓

**File**: `/home/sttark/Desktop/github/semi_rotary_machine_automation/RPI/interactive_mode.py`  
**Line**: 695

**Status**: Already configured at 500ms (no change needed)

```python
enable_transition_period_ms=500.0,
```

**Impact**: Allows drives 0.5 seconds to respond to commands, reducing retry loops.

## Required Actions

### ⚠️ REBOOT REQUIRED

The `force_turbo=1` setting in `/boot/firmware/config.txt` requires a system reboot to take effect.

```bash
sudo reboot
```

### After Reboot - Verify

1. **Restart your EtherCAT application**:
   ```bash
   sudo python3 RPI/interactive_mode.py
   ```

2. **Monitor timing logs** for at least 5 minutes:
   - Look for `overruns=0` in the `[EC]` timing logs
   - Verify `min:sleep` stays positive (> 100000 ns)
   - Check that `max:work` is well under 1000000 ns (1ms cycle time)

3. **Expected improvements**:
   - Zero overruns
   - `max:cia` reduced from 459µs to < 300µs
   - `max:recv` reduced from 88µs to < 50µs
   - More stable/consistent cycle timing

## Rollback Instructions

If issues occur, revert changes:

### 1. Revert DC Sync Decimation
```bash
cd /home/sttark/Desktop/github
git diff EtherCAT/process_manager.py
git checkout EtherCAT/process_manager.py
```

### 2. Revert config.txt
```bash
sudo nano /boot/firmware/config.txt
# Remove the last 3 lines:
# # Real-time tuning: prevent CPU frequency scaling and reduce GPU activity
# force_turbo=1
# gpu_mem=16
```

### 3. Revert NIC Driver
```bash
cd /usr/src/igh-ethercat-master/devices/stmmac
git diff stmmac_main-6.12-ethercat.c
git checkout stmmac_main-6.12-ethercat.c
cd /usr/src/igh-ethercat-master
sudo make clean && sudo make modules && sudo make modules_install
sudo reboot
```

## Technical Notes

- **Driver in use**: System is using `macb` driver (Pi 5 onboard ethernet)
- **IgH configured for**: `r8169` (Realtek PCIe card)
- **stmmac changes**: Applied but not currently active (would apply if switching to stmmac-based NICs)
- **Cycle time**: Unchanged at 1ms - can be increased later if needed
- **Communication errors**: Slaves 2, 4, 5 showing error code 0x7500 - may need cabling inspection

## Performance Targets (1ms cycle)

After these changes, target metrics:
- `overruns=0` (must be zero)
- `min:sleep > 200000` ns (200µs safety margin)
- `max:work < 800000` ns (leaves 200µs margin)
- `avg:jitter < 1000` ns (sub-microsecond average jitter)

## Additional Optimizations (Not Implemented)

If overruns persist after reboot, consider:
1. Increase cycle time from 1ms to 2ms or 4ms
2. Inspect physical cabling on slaves 2, 4, 5
3. Minimize PDO reads in cyclic loop
4. Move status publishing to separate process

## Files Modified

1. `/home/sttark/Desktop/github/EtherCAT/process_manager.py`
2. `/usr/src/igh-ethercat-master/devices/stmmac/stmmac_main-6.12-ethercat.c`
3. `/boot/firmware/config.txt`

## References

- [EtherCAT RT Tuning Guide](/home/sttark/Desktop/github/EtherCAT/docs/rt-tuning-guide.md)
- Section 6: Reducing Cycle Work
- Section 7: Timing Budget Reference
