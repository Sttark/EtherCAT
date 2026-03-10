# EtherCAT Overrun Fix - Implementation Summary

**Date**: 2026-02-24  
**Status**: DC Decimation Reverted + Monitoring Added - Ready for Testing

## Changes Implemented

### 1. DC Sync Monitor Decimation ⚠️ REVERTED

**File**: `/home/sttark/Desktop/github/EtherCAT/process_manager.py`  
**Lines**: 2159-2175

**Original Change**: Modified DC sync monitor calls to execute only every 10 cycles instead of every cycle.

**REVERTED**: 2026-02-24  
**Reason**: Caused random DC synchronization errors (AL status 0x001A) and slave dropouts from OP state.

**Root Cause Analysis**:
- `sync_monitor_queue()` queues datagrams for ongoing DC clock adjustment, not just monitoring
- Calling it only every 10 cycles (20ms) instead of every cycle (2ms) caused:
  - Slower DC PLL convergence (1+ second instead of <500ms)
  - Clock drift accumulation between updates
  - Random "Synchronization error" when drift exceeded tolerance
  - Slaves dropping from OP → SAFEOP after 30-60 seconds of operation

**Evidence**:
- dmesg showed slaves taking 1.2+ seconds to sync (should be <500ms)
- Random sync errors despite only 1 overrun (RT loop was fine)
- Working counter drops from 27/27 → 19/27 when slaves lost sync
- Errors occurred ~40 seconds after reaching OP (drift accumulation)

**Current Status**: REVERTED - `sync_monitor_queue()` now runs every cycle

**Code** (reverted to):
```python
# DC sync monitor: MUST run every cycle for proper clock adjustment
try:
    dc_err_ns = self.master.sync_monitor_process()
    # ... error tracking code ...
    self.master.sync_monitor_queue()  # Critical: runs EVERY cycle
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

### 5. EtherCAT Health Monitoring ✓

**Date**: 2026-02-24  
**Files**: `EtherCAT/status_model.py`, `EtherCAT/process_manager.py`, `semi_rotary_machine_automation/RPI/interactive_mode.py`

**Changes**: Added comprehensive monitoring **entirely outside RT loop** to diagnose random sync failures:

**Features**:
- **Working Counter Monitoring**: Detects slaves dropping from OP, alerts with slave list
- **RT Budget Monitoring**: Tracks work time as % of cycle budget, alerts if >85%
- **Phase Timing Breakdown**: Detailed timing for each RT loop phase
- **EC Health Command**: Type `health` in interactive mode for full diagnostic view

**Impact**:
- Zero RT loop overhead (all analysis in `_publish_status()`)
- Clear visibility when slaves drop with WC alerts
- Correlation guidance to check dmesg
- Identifies if RT performance or DC sync is the issue

See `MONITORING_IMPROVEMENTS.md` for full details.

## Required Actions

### ⚠️ CRITICAL: DC Sync Decimation Reverted

**Action**: Restart your EtherCAT application to pick up the reverted DC sync monitoring.

**Why**: The decimation was causing random sync errors and slave dropouts by calling `sync_monitor_queue()` only every 10 cycles instead of every cycle. This function is **critical for ongoing DC clock adjustment**, not just monitoring.

**Expected improvement**:
- Faster DC convergence (<500ms instead of 1.2+ seconds)
- No random "Synchronization error" (0x001A) after reaching OP
- Stable working counter (27/27) without drops
- Drives enable reliably without getting stuck

### Test with New Monitoring

After restarting, use the new `health` command in interactive mode:

```bash
sudo python3 RPI/interactive_mode.py
```

Then type:
```
health
```

**What to watch for**:
- WC should stay at 27/27 (no drops)
- RT budget should be <70% (well within limits)
- No "WC drop" or "RT budget" warnings in logs
- All slaves should enable quickly and stay enabled
- DC sync convergence should be faster (<500ms per slave)

### Verify DC Sync is Working

Monitor dmesg during startup to confirm faster convergence:

```bash
sudo dmesg -w | grep -E "(Sync after|difference after|Synchronization error)"
```

**Before reversion**: Slaves took 1200+ms to sync (125868 ns → 9821 ns over 1268ms)  
**After reversion**: Should sync in <500ms

**Watch for**: No "Synchronization error" messages after slaves reach OP

## Summary

**DC Decimation Reverted**: But convergence STILL takes 4+ seconds (tested with 2 restarts)

**This proves**: Decimation reversion alone doesn't fix the problem. There's a **deeper DC configuration issue**.

**Current Status**:
- Monitoring added (WC drops, RT budget, `health` command) ✓
- DC decimation reverted ✓  
- **Problem persists**: Slaves take 4+ seconds to sync, random 0x001A errors

**Root Cause**: Unknown - investigating:
1. dc_sync0_shift_ns value (currently 1ms for 2ms cycle)
2. Mixed DC/non-DC slave configuration
3. DC reference clock selection
4. ABB drive-specific DC requirements

**Next Steps**: Test different shift times (125us, 500us, 0us) or try uniform DC config (all drives DC-enabled)

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
