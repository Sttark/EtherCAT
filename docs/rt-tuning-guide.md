# EtherCAT Real-Time Tuning Guide

Reference document for achieving reliable 1ms (and sub-1ms) EtherCAT cycle times on Raspberry Pi 5 with IgH EtherCAT Master and PREEMPT_RT kernel.

## Hardware Platform

- Raspberry Pi 5 (BCM2712, quad-core Cortex-A76)
- Kernel: PREEMPT_RT (e.g., `6.12.67-v8-16k-rt`)
- EtherCAT master: IgH EtherCAT Master (cdev/ioctl path)
- NIC drivers: stmmac or r8169 with EtherCAT patches

---

## 1. Kernel Boot Parameters

In `/boot/firmware/cmdline.txt`, append:

```
isolcpus=3 nohz_full=3 rcu_nocbs=3
```

| Parameter | Effect |
|-----------|--------|
| `isolcpus=3` | Prevents kernel from scheduling normal tasks on core 3 |
| `nohz_full=3` | Disables timer tick on core 3 when only one task is running |
| `rcu_nocbs=3` | Offloads RCU callbacks from core 3 to other cores |

Core 3 is the RT EtherCAT cyclic process. Cores 0-2 handle OS, IRQs, and all other work.

## 2. RT Scheduler Throttle

**Critical:** Linux throttles RT tasks by default, forcibly descheduling them for 50ms every second.

Check current setting:
```bash
cat /proc/sys/kernel/sched_rt_runtime_us
# Default: 950000 (950ms of every 1000ms period)
```

Disable throttle:
```bash
sudo sh -c 'echo -1 > /proc/sys/kernel/sched_rt_runtime_us'
```

Make persistent:
```bash
sudo sh -c 'echo "kernel.sched_rt_runtime_us = -1" > /etc/sysctl.d/99-rt-nosched.conf'
```

## 3. Process-Level RT Configuration

Applied in the EtherCAT cyclic process (`process_manager.py`):

| Setting | Value | API |
|---------|-------|-----|
| Scheduler | SCHED_FIFO | `os.sched_setscheduler(0, 1, sched_param(99))` |
| RT priority | 99 (maximum) | Same as above |
| CPU affinity | Core 3 only | `os.sched_setaffinity(0, {3})` |
| Memory lock | All pages | `libc.mlockall(MCL_CURRENT \| MCL_FUTURE)` via ctypes |
| Sleep | Absolute monotonic | `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` via ctypes |

## 4. IRQ Affinity

Steer the EtherCAT NIC interrupt away from the RT core:

```python
# In EthercatNetworkConfig:
irq_affinity = {115: "0-2"}
```

This writes `"0-2"` to `/proc/irq/115/smp_affinity_list`, keeping IRQ 115 (the NIC) off core 3.

Find your NIC IRQ number:
```bash
cat /proc/interrupts | grep eth
```

## 5. IgH EtherCAT Master Kernel Module

### io_mutex Contention

The IgH master uses `io_mutex` (an rt_mutex) for several ioctl calls per cycle:

| Call | Locks io_mutex | Weight |
|------|---------------|--------|
| `ecrt_master_receive()` | **Yes** | Heavy (NIC poll + frame parse) |
| `ecrt_domain_process()` | No | Medium (datagram bookkeeping) |
| `ecrt_domain_state()` | No | Light |
| `ecrt_master_application_time()` | No | Trivial |
| `ecrt_master_sync_reference_clock()` | **Yes** | Light |
| `ecrt_master_sync_slave_clocks()` | **Yes** | Light |
| `ecrt_master_sync_monitor_process()` | No | Trivial |
| `ecrt_master_sync_monitor_queue()` | **Yes** | Light |
| `ecrt_domain_queue()` | **Yes** | Medium |
| `ecrt_master_send()` | **Yes** | Heavy (frame assembly + NIC TX) |

**6 mutex acquisitions per cycle.** Each has rt_mutex overhead even without contention.

### ec_poll() Budget

The NIC driver's EtherCAT poll function processes up to N packets per call:

| Driver | Default Budget | Recommended |
|--------|---------------|-------------|
| stmmac | 128 | 4-8 |
| r8169 | 100 | 4-8 |

EtherCAT produces 1 frame per cycle. Budget of 128 means scanning for up to 128 packets every `receive()` call -- wasted work that adds variable latency.

To patch, edit the ec_poll function in the driver source:
- `stmmac_main-6.12-ethercat.c` ~line 7260: `int budget = 128;` → `int budget = 4;`
- `r8169_main-6.12-ethercat.c` ~line 5481: budget in `rtl_tx`/`rtl_rx` calls

Rebuild IgH kernel modules after patching.

### Datagram Queue Matching

Received frames are matched to queued datagrams via **linear scan** (O(n) per received datagram). With many slaves/PDOs, this adds queue-depth-dependent jitter. No easy fix without modifying IgH source to use indexed lookup.

### RTDM Mode

The IgH master supports `--enable-rtdm` which makes io_mutex locks no-ops. However, this build requires **Xenomai or RTAI** headers/toolchain -- it is not a generic PREEMPT_RT backend. Significant platform change.

## 6. Reducing Cycle Work

### Minimize Domain Reads in the RT Path

Only read PDO values needed for the frame path per cycle:
- **Statusword** -- needed for CiA402 state machine
- **Position + Velocity** -- needed for Ruckig trajectory (active axes only)

All other reads (torque, mode_display, error_code) should be deferred to the sleep budget (after `master.send()`) or moved to a separate process via shared memory.

### Separate Non-RT Work

Move these out of the cyclic process entirely:
- **Status publishing** -- read from shared memory in parent process
- **Semi-rotary cam interpolation** -- separate process, communicates via shared memory
- **Logging** -- non-blocking queue to background thread

### DC Sync Monitor Decimation

`sync_monitor_queue()` and `sync_monitor_process()` can be called every Nth cycle (e.g., every 10-100 cycles) instead of every cycle. This saves 1 mutex acquisition per skipped cycle. DC sync accuracy is unaffected since `sync_reference_clock()` and `sync_slave_clocks()` still run every cycle.

## 7. Timing Budget Reference (1ms cycle)

| Phase | Typical (us) | Spike (us) | Notes |
|-------|-------------|------------|-------|
| Wakeup lateness | 5 | 20-65 | clock_nanosleep accuracy |
| receive + process_domain | 7-10 | 30-70 | IgH kernel poll + parse |
| Slave state tracking | 25 | 35-135 | Python dict ops + get_state() |
| CiA402 state machine | 0.2-0.4 | 1-5 | Light when stable |
| Ruckig trajectory | 2-3 | 10-35 | Native C++ via pybind11 |
| _cyclic_write (PDO writes) | 80-150 | 170-560 | Domain writes + cache reads |
| DC sync + send | 15-25 | 30-120 | Includes frame assembly |
| **Total work** | **130-220** | **350-770** | |
| **Sleep budget** | **780-870** | **230-650** | Must stay > 0 to avoid overrun |

Spikes are caused by IgH kernel module variable latency (NIC poll, mutex, datagram matching), not Python code.

## 8. Firmware / Hardware Mitigations

### Pi 5 config.txt Options

```ini
# Prevent CPU frequency scaling transitions
force_turbo=1

# Reduce GPU firmware activity (if no display needed)
gpu_mem=16

# Disable HDMI if not needed
dtoverlay=vc4-kms-v3d,nohdmi
```

### Potential Stall Sources

| Source | Impact | Mitigation |
|--------|--------|------------|
| RT scheduler throttle | 50ms forced preemption/sec | `sched_rt_runtime_us=-1` |
| CPU frequency scaling | DVFS transition stalls | `force_turbo=1` |
| GPU firmware interrupts | Periodic firmware calls | Reduce `gpu_mem`, disable HDMI |
| IgH io_mutex | 6 acquisitions/cycle | Reduce ioctl count, RTDM |
| NIC poll budget | Variable RX/TX cleanup | Reduce budget to 4-8 |
| Memory bus contention | DMA from other cores/GPU | Minimize GPU activity |
| SMI/TrustZone | Firmware-level, invisible | Platform limitation |

## 9. Diagnostic Tools

### Timing Log

The cyclic process logs per-second statistics:

```
[EC] avg:actual=1000000 avg:jitter=900 max:lateness=25000 max:work=220000
     min:sleep=168000 overruns=0
     max:recv=13000 max:state=36000 max:cia=700 max:ruckig=5000
     max:pdo_w=91000 max:dc_send=25000 dc_err=1023 cycles=1000
```

| Field | Meaning |
|-------|---------|
| avg:actual | Average cycle period (should be ~target) |
| avg:jitter | Average absolute jitter over 1 second |
| max:lateness | Worst wakeup lateness in the second |
| max:work | Worst total work time (wakeup to post-send) |
| min:sleep | Worst-ever sleep budget (overall watermark) |
| overruns | Total cycles where work exceeded cycle budget |
| max:recv | Worst receive+process_domain time |
| max:state | Worst slave state tracking time |
| max:cia | Worst CiA402 state machine time |
| max:ruckig | Worst Ruckig + read_targets time |
| max:pdo_w | Worst _cyclic_write time |
| max:dc_send | Worst DC sync + queue + send time |
| dc_err | Last DC sync error (ns) |
| cycles | Cycles in this reporting period |

### External Tools

```bash
# Check RT throttle
cat /proc/sys/kernel/sched_rt_runtime_us

# Check CPU isolation
cat /sys/devices/system/cpu/isolated

# Check IRQ affinity
cat /proc/irq/115/smp_affinity_list

# Check kernel tick mode
cat /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor

# Measure worst-case latency
sudo cyclictest -t1 -p99 -a3 -i1000 -l10000

# Detect hardware latency
sudo hwlatdetect --duration=60 --threshold=10
```

## 10. Summary Checklist

- [ ] `isolcpus=3 nohz_full=3 rcu_nocbs=3` in cmdline.txt
- [ ] `sched_rt_runtime_us=-1` (persistent in `/etc/sysctl.d/99-rt-nosched.conf`)
- [ ] SCHED_FIFO priority 99 on core 3
- [ ] mlockall(MCL_CURRENT | MCL_FUTURE)
- [ ] clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)
- [ ] NIC IRQ steered to cores 0-2
- [ ] _publish_status removed from RT loop (shared memory instead)
- [ ] Semi-rotary cam in separate process
- [ ] Logging via non-blocking queue to background thread
- [ ] Minimal PDO reads in work path (statusword + active Ruckig axes only)
- [ ] Consider: ec_poll budget reduction (128 → 4-8)
- [ ] Consider: DC sync monitor decimation
- [ ] Consider: force_turbo=1 in config.txt
