import bisect
import ctypes
import logging
import multiprocessing as mp
import os
import queue
import signal
import sys
import time
import traceback
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

_parent = Path(__file__).resolve().parent.parent
if str(_parent) not in sys.path:
    sys.path.insert(0, str(_parent))

_semi_rotary_rpi = _parent / "semi_rotary_machine_automation" / "RPI"
if _semi_rotary_rpi.is_dir() and str(_semi_rotary_rpi) not in sys.path:
    sys.path.insert(0, str(_semi_rotary_rpi))

try:
    from lib.trajectory_core import RuckigVelocityIntegrator, ContinuousRotationTrajectory, RUCKIG_AVAILABLE as RUCKIG_AVAILABLE_CORE
except ImportError:
    from RPI.lib.trajectory_core import RuckigVelocityIntegrator, ContinuousRotationTrajectory, RUCKIG_AVAILABLE as RUCKIG_AVAILABLE_CORE

from .commands import Command, CommandType
from .status_model import NetworkStatus
from .constants import (
    CW_INDEX, SW_INDEX,
    MODES_OP_INDEX, MODES_OP_DISPLAY_INDEX,
    TARGET_POSITION_INDEX, TARGET_VELOCITY_INDEX, TARGET_TORQUE_INDEX, MAX_TORQUE_INDEX,
    POSITION_ACTUAL_INDEX, VELOCITY_ACTUAL_INDEX, TORQUE_ACTUAL_INDEX,
    PROBE_FUNCTION_INDEX, PROBE_STATUS_INDEX, PROBE_POS1_INDEX, PROBE_POS2_INDEX, PROBE_POS2_ALT_INDEX, DIGITAL_INPUTS_INDEX, DIP_IN_STATE_INDEX,
    ERROR_CODE_INDEX,
    MODE_PP, MODE_PV, MODE_PT, MODE_CSP, MODE_HM,
    CW_BIT_NEW_SET_POINT, CW_BIT_CHANGE_IMMEDIATELY, CW_BIT_ABS_REL, CW_BIT_HALT, CW_ENABLE_OP_SIMPLIFIED,
    PROBE_FUNC_ENABLE_PROBE1, PROBE_FUNC_PROBE1_POS_EDGE, PROBE_FUNC_PROBE1_NEG_EDGE,
)
from .config_schema import EthercatNetworkConfig, DriveConfig
from .igh_master import (
    Master, MasterException, SDOException,
    EC_REQUEST_BUSY, EC_REQUEST_ERROR, EC_REQUEST_SUCCESS,
)
from .xml_decoder import decode_esi
from .ruckig_planner import RuckigCspPlanner, RuckigUnavailable
from . import shm_layout as SHM


try:
    import ruckig
except ImportError:
    ruckig = None


logger = logging.getLogger(__name__)

VELOCITY_DEADBAND_COUNTS_PER_S = 30000.0
ACCELERATION_DEADBAND_COUNTS_PER_S2 = 100000.0

AL_STATE_SAFEOP = 0x04
AL_STATE_OP = 0x08
_SM_WD_DEFAULT_DIVIDER_40NS = 2500  # 2500 * 40ns = 100us base interval
FAULT_CODE_LABELS: Dict[int, str] = {
    0x0000: "No error",
    0x1000: "Generic error",
    0x2300: "Current sensor",
    0x3100: "Overvoltage",
    0x3200: "Undervoltage",
    0x4200: "Temperature",
    0x5400: "Driver disabled",
    0x6010: "Software reset",
    0x6100: "Internal software",
    0x6320: "Rated current",
    0x7300: "Sensor error",
    0x7500: "Communication",
    0x8400: "Velocity/speed",
    0x8600: "Velocity control",
    0x8611: "Velocity demand overflow",
    0x8612: "Velocity demand too high",
    0xFF00: "Manufacturer-specific",
}


def _wrap_i32(v: int) -> int:
    return ((v + 0x8000_0000) % 0x1_0000_0000) - 0x8000_0000


class _RefClock32Extender:
    """
    Extend a wrapping 32-bit nanosecond counter to a monotonic 64-bit value.

    IgH exposes the lower 32 bits of the reference clock's system time, which wraps
    every ~4.29s. We extend it in userspace so we can feed a stable `app_time`.
    """

    def __init__(self) -> None:
        self._last32: Optional[int] = None
        self._ext64: int = 0

    def seed_near(self, target64: int, cur32: int) -> int:
        """
        Seed the extended clock so that the 32-bit sample is placed near `target64`.

        This prevents catastrophic app_time jumps when switching from a fallback
        timebase to reference-clock-based time.
        """
        cur32 = int(cur32) & 0xFFFF_FFFF
        target64 = int(target64)

        base = target64 & ~0xFFFF_FFFF
        cand = base | cur32

        # Pick the closest of {cand-2^32, cand, cand+2^32}.
        step = 0x1_0000_0000
        best = cand
        best_err = abs(best - target64)
        alt = cand - step
        alt_err = abs(alt - target64)
        if alt_err < best_err:
            best, best_err = alt, alt_err
        alt = cand + step
        alt_err = abs(alt - target64)
        if alt_err < best_err:
            best = alt

        self._last32 = cur32
        self._ext64 = int(best)
        return self._ext64

    def update(self, cur32: int) -> int:
        cur32 = int(cur32) & 0xFFFF_FFFF
        if self._last32 is None:
            self._last32 = cur32
            self._ext64 = cur32
            return self._ext64

        last32 = self._last32
        delta = (cur32 - last32) & 0xFFFF_FFFF

        # If the delta looks like a large backwards jump (glitch), ignore this sample.
        # Legitimate forward wrap produces a small delta after modulo subtraction.
        if delta > 0x8000_0000:
            return self._ext64

        self._ext64 += delta
        self._last32 = cur32
        return self._ext64


def _median_i64(values: List[int]) -> int:
    if not values:
        return 0
    s = sorted(int(v) for v in values)
    return int(s[len(s) // 2])


@dataclass
class _SdoReqSlot:
    slave_pos: int
    index: int
    subindex: int
    req: Any
    max_size: int
    # Desired operations (set by callers; queued by queue_due()).
    desired_read: bool = False
    desired_write: Optional[bytes] = None
    desired_write_hash: Optional[int] = None
    # Inflight tracking
    inflight: bool = False
    inflight_kind: Optional[str] = None  # "read" | "write"
    inflight_write_hash: Optional[int] = None
    last_queue_cycle: int = -10**12
    last_success_cycle: int = -10**12
    last_error_cycle: int = -10**12
    last_error: Optional[str] = None
    last_success_write_hash: Optional[int] = None
    # Read cache
    cached_read: Optional[bytes] = None
    cached_read_cycle: int = -10**12
    # Dedup/backoff knobs (in cycles)
    read_min_interval_cycles: int = 500  # default 1s @2ms
    write_backoff_cycles: int = 10       # default 20ms @2ms


class _SdoRequestManager:
    """
    Manage IgH SDO request objects in the cyclic process, with strict dedup/backoff.

    - Call `poll()` after `receive()` to collect completions.
    - Call `queue_due()` before `send()` to queue any desired operations.
    """

    def __init__(self) -> None:
        self._slots: Dict[Tuple[int, int, int], _SdoReqSlot] = {}

    def register(self, slave_pos: int, index: int, subindex: int, req: Any, max_size: int) -> None:
        key = (int(slave_pos), int(index), int(subindex))
        self._slots[key] = _SdoReqSlot(
            slave_pos=int(slave_pos),
            index=int(index),
            subindex=int(subindex),
            req=req,
            max_size=int(max_size),
        )

    def has(self, slave_pos: int, index: int, subindex: int) -> bool:
        return (int(slave_pos), int(index), int(subindex)) in self._slots

    def set_desired_read(self, slave_pos: int, index: int, subindex: int, *, min_interval_cycles: Optional[int] = None) -> None:
        slot = self._slots.get((int(slave_pos), int(index), int(subindex)))
        if slot is None:
            return
        slot.desired_read = True
        if min_interval_cycles is not None:
            slot.read_min_interval_cycles = max(1, int(min_interval_cycles))

    def clear_desired_read(self, slave_pos: int, index: int, subindex: int) -> None:
        slot = self._slots.get((int(slave_pos), int(index), int(subindex)))
        if slot is None:
            return
        slot.desired_read = False

    def set_desired_write(self, slave_pos: int, index: int, subindex: int, data: bytes, *, backoff_cycles: Optional[int] = None) -> None:
        slot = self._slots.get((int(slave_pos), int(index), int(subindex)))
        if slot is None:
            return
        b = bytes(data)
        slot.desired_write = b
        slot.desired_write_hash = hash(b)
        if backoff_cycles is not None:
            slot.write_backoff_cycles = max(1, int(backoff_cycles))

    def get_cached_read(
        self,
        slave_pos: int,
        index: int,
        subindex: int,
        *,
        cycle_count: int,
        max_age_cycles: int = 500,
    ) -> Optional[bytes]:
        slot = self._slots.get((int(slave_pos), int(index), int(subindex)))
        if slot is None:
            return None
        if slot.cached_read is None:
            return None
        if int(max_age_cycles) <= 0:
            return slot.cached_read
        age = int(cycle_count) - int(slot.cached_read_cycle)
        if age <= int(max_age_cycles):
            return slot.cached_read
        return None

    def poll(self, cycle_count: int) -> None:
        cc = int(cycle_count)
        for slot in self._slots.values():
            if not slot.inflight:
                continue
            try:
                st = int(slot.req.state())
            except Exception as e:
                slot.inflight = False
                slot.last_error_cycle = cc
                slot.last_error = f"state() failed: {e}"
                continue
            if st == EC_REQUEST_BUSY:
                continue
            if st == EC_REQUEST_SUCCESS:
                slot.inflight = False
                slot.last_success_cycle = cc
                slot.last_error = None
                if slot.inflight_kind == "read":
                    try:
                        slot.cached_read = bytes(slot.req.read_data())
                        slot.cached_read_cycle = cc
                    except Exception as e:
                        slot.last_error_cycle = cc
                        slot.last_error = f"read_data() failed: {e}"
                elif slot.inflight_kind == "write":
                    slot.last_success_write_hash = slot.inflight_write_hash
                slot.inflight_kind = None
                slot.inflight_write_hash = None
                continue
            if st == EC_REQUEST_ERROR:
                slot.inflight = False
                slot.last_error_cycle = cc
                slot.last_error = "request state=ERROR"
                slot.inflight_kind = None
                slot.inflight_write_hash = None
                continue
            # Unknown state: treat as not busy but not successful.
            slot.inflight = False
            slot.last_error_cycle = cc
            slot.last_error = f"unknown request state={st}"
            slot.inflight_kind = None
            slot.inflight_write_hash = None

    def queue_due(self, cycle_count: int) -> None:
        cc = int(cycle_count)
        for slot in self._slots.values():
            if slot.inflight:
                continue

            # Prefer writes over reads if both are desired.
            if slot.desired_write is not None:
                # Dedup: if desired value is same as last successful write and no error since, skip.
                if slot.desired_write_hash is not None and slot.desired_write_hash == slot.last_success_write_hash:
                    # Still allow retry if we had a recent error.
                    if slot.last_error_cycle > slot.last_success_cycle:
                        pass
                    else:
                        slot.desired_write = None
                        continue
                if (cc - int(slot.last_queue_cycle)) < int(slot.write_backoff_cycles):
                    continue
                try:
                    slot.req.queue_write(slot.desired_write)
                    slot.inflight = True
                    slot.inflight_kind = "write"
                    slot.inflight_write_hash = slot.desired_write_hash
                    slot.last_queue_cycle = cc
                    # Keep desired_write until success to allow retry on error; cleared in poll() on success via hash dedup above.
                except Exception as e:
                    slot.last_error_cycle = cc
                    slot.last_error = f"queue_write failed: {e}"
                continue

            if slot.desired_read:
                if (cc - int(slot.last_queue_cycle)) < int(slot.read_min_interval_cycles):
                    continue
                try:
                    slot.req.queue_read()
                    slot.inflight = True
                    slot.inflight_kind = "read"
                    slot.last_queue_cycle = cc
                except Exception as e:
                    slot.last_error_cycle = cc
                    slot.last_error = f"queue_read failed: {e}"

    def clear_all_desired(self) -> None:
        """
        Stop queueing any new SDO request-object operations.

        Note: does not cancel in-flight mailbox operations (IgH doesn't expose a safe cancel),
        but prevents additional queue pressure during unstable network periods.
        """
        for slot in self._slots.values():
            slot.desired_read = False
            slot.desired_write = None
            slot.desired_write_hash = None

class EtherCATProcess:
    def _sdo_queue_allowed_now(self) -> bool:
        """
        Gate runtime SDO request-object traffic in PDO (cyclic) mode.

        During DC sync faults / WC drops / OP dropouts, mailbox traffic can increase bus load
        and interfere with recovery. By default, we only queue SDOs when the network is healthy.
        """
        if not bool(getattr(self.cfg, "enable_runtime_sdo_requests", True)):
            return False

        # Default-on guards (can be overridden via config).
        guard_all_op = bool(getattr(self.cfg, "sdo_queue_only_when_all_slaves_op", True))
        guard_dc_settled = bool(getattr(self.cfg, "sdo_queue_only_when_dc_settled", True))
        guard_wc_ok = bool(getattr(self.cfg, "sdo_queue_only_when_wc_ok", True))

        if guard_all_op:
            if not self.slave_in_op or (not all(self.slave_in_op.values())):
                return False

        if guard_dc_settled and (not bool(getattr(self, "_dc_settling_complete", False))):
            return False

        if guard_wc_ok:
            wc = self._last_domain_wc
            if self._wc_expected > 0 and wc is not None and int(wc) < int(self._wc_expected):
                return False

        return True
    def _setup_runtime_sdo_requests_for_slave(self, slave_pos: int, sc: Any, *, is_cia402: bool) -> bool:
        """
        Create IgH SDO request objects for runtime mailbox access (OP-safe shape).

        These requests are serviced via the cyclic send/receive flow. We create them
        once during setup and then only queue/poll them in the cyclic loop.
        """
        if not is_cia402:
            return True
        if not bool(getattr(self.cfg, "enable_runtime_sdo_requests", True)):
            return True
        req_specs = [
            (ERROR_CODE_INDEX, 0, 2),          # 0x603F Error code
            (MODES_OP_INDEX, 0, 1),            # 0x6060 Modes of operation
            (TARGET_VELOCITY_INDEX, 0, 4),     # 0x60FF Target velocity
            (TARGET_TORQUE_INDEX, 0, 2),       # 0x6071 Target torque
            (TARGET_POSITION_INDEX, 0, 4),     # 0x607A Target position
            (PROBE_FUNCTION_INDEX, 0, 2),      # 0x60B8 Touch probe function
        ]
        for idx, sub, size in req_specs:
            try:
                req = sc.create_sdo_request(int(idx), int(sub), int(size))
                self._sdo_mgr.register(int(slave_pos), int(idx), int(sub), req=req, max_size=int(size))
            except Exception as e:
                self._emit_process_log(
                    f"[EC] Failed to create SDO request object slave={slave_pos} 0x{int(idx):04X}:{int(sub)} size={int(size)}: {e}",
                    stderr=True,
                )
                return False
        return True

    def _boost_igh_master_priority(self) -> None:
        """Boost IGH EtherCAT master thread to RT priority 99 for fast slave activation"""
        try:
            import subprocess
            import time as time_module
            
            # Wait briefly for EtherCAT-OP thread to spawn after activation
            time_module.sleep(0.1)

            igh_core_raw = getattr(self.cfg, "igh_master_cpu_core", None)
            igh_core: Optional[int] = None
            if igh_core_raw is not None:
                try:
                    igh_core = int(igh_core_raw)
                except Exception:
                    self._emit_process_log(f"[EC] Invalid igh_master_cpu_core={igh_core_raw!r}", stderr=True)
                    igh_core = None
            
            # Find and boost IGH master thread (retry a few times)
            for attempt in range(5):
                result = subprocess.run(
                    ["ps", "-eLo", "pid,comm"],
                    capture_output=True, text=True, timeout=2
                )
                
                # Boost IGH master thread to priority 99
                for line in result.stdout.splitlines():
                    if "EtherCAT-OP" in line or "EtherCAT-IDLE" in line:
                        parts = line.split()
                        if parts:
                            pid = int(parts[0])
                            ret = subprocess.run(
                                ["chrt", "-f", "-p", "99", str(pid)],
                                capture_output=True, timeout=2
                            )
                            if ret.returncode == 0:
                                self._emit_process_log(f"[EC] Boosted IGH master thread (PID {pid}) to RT priority 99")
                            else:
                                self._emit_process_log(f"[EC] Failed to boost IGH (PID {pid}): {ret.stderr.decode()}", stderr=True)

                            if igh_core is not None:
                                try:
                                    # Note: this PID is the thread ID (TID) from `ps -eLo`.
                                    ts = subprocess.run(
                                        ["taskset", "-pc", str(igh_core), str(pid)],
                                        capture_output=True, text=True, timeout=2
                                    )
                                    if ts.returncode == 0:
                                        out = (ts.stdout or "").strip()
                                        self._emit_process_log(f"[EC] Pinned IGH master thread (TID {pid}) to CPU {igh_core}: {out}")
                                    else:
                                        err = (ts.stderr or "").strip()
                                        self._emit_process_log(
                                            f"[EC] Failed to pin IGH master thread (TID {pid}) to CPU {igh_core}: {err}",
                                            stderr=True,
                                        )
                                except Exception as e:
                                    self._emit_process_log(
                                        f"[EC] Failed to pin IGH master thread (TID {pid}) to CPU {igh_core}: {e}",
                                        stderr=True,
                                    )
                            break
                else:
                    # Thread not found, wait and retry
                    if attempt < 4:
                        time_module.sleep(0.05)
                        continue
                    self._emit_process_log("[EC] IGH master thread not found for priority boost", stderr=True)
                break
            
            # Boost NIC IRQ thread(s) to priority 98 (based on configured irq_affinity keys).
            rules = getattr(self.cfg, "irq_affinity", None)
            irqs: List[int] = []
            if isinstance(rules, dict):
                for irq_raw in rules.keys():
                    try:
                        irqs.append(int(irq_raw))
                    except Exception:
                        continue
            if irqs:
                for irq in sorted(set(irqs)):
                    for line in result.stdout.splitlines():
                        if f"irq/{irq}" in line:
                            parts = line.split()
                            if parts:
                                pid = int(parts[0])
                                try:
                                    subprocess.run(["chrt", "-f", "-p", "98", str(pid)], timeout=2)
                                    self._emit_process_log(f"[EC] Boosted IRQ thread irq/{irq} (TID {pid}) to RT priority 98")
                                except Exception as e:
                                    self._emit_process_log(f"[EC] Failed to boost IRQ thread irq/{irq} (TID {pid}): {e}", stderr=True)
                            break
                        
        except Exception as e:
            self._emit_process_log(f"[EC] Failed to boost RT priorities: {e}", stderr=True)
    
    def _apply_irq_affinity(self) -> None:
        rules = getattr(self.cfg, "irq_affinity", None)
        if not isinstance(rules, dict) or not rules:
            return
        for irq_raw, affinity_raw in rules.items():
            try:
                irq = int(irq_raw)
            except Exception:
                self._emit_process_log(f"[EC] irq_affinity invalid irq={irq_raw}", stderr=True)
                continue
            affinity = str(affinity_raw).strip()
            if not affinity:
                self._emit_process_log(f"[EC] irq_affinity empty affinity irq={irq}", stderr=True)
                continue
            path = f"/proc/irq/{irq}/smp_affinity_list"
            try:
                with open(path, "w", encoding="utf-8") as f:
                    f.write(affinity)
                effective = affinity
                try:
                    with open(path, "r", encoding="utf-8") as f:
                        effective = f.read().strip() or affinity
                except Exception:
                    pass
                self._emit_process_log(
                    f"[EC] irq_affinity irq={irq} target={affinity} effective={effective}"
                )
            except Exception as e:
                self._emit_process_log(
                    f"[EC] irq_affinity failed irq={irq} target={affinity} error={e}",
                    stderr=True,
                )

    def _install_signal_handlers(self) -> None:
        """
        Ensure SIGTERM/SIGINT (e.g., systemd stop) results in a graceful shutdown.

        We translate signals into `stop_event.set()` so the cyclic loop exits and `_teardown()`
        runs (releasing the EtherCAT master).
        """
        def _handler(signum, _frame):
            try:
                name = signal.Signals(signum).name
            except Exception:
                name = str(signum)
            logger.info(f"Received {name} - stopping EtherCAT process gracefully...")
            self.stop_event.set()

        for sig in (signal.SIGTERM, signal.SIGINT):
            try:
                signal.signal(sig, _handler)
            except Exception:
                # Some environments may restrict signal handler installation.
                pass

    """
    Isolated process that owns the EtherCAT master and cyclic loop.
    Communication via command/status queues. Non-blocking, resilient runtime.
    """

    def __init__(self, cfg: EthercatNetworkConfig, cmd_q: mp.Queue, status_q: mp.Queue, stop_event: mp.Event,
                 actuals_shm=None, targets_shm=None):
        self.cfg = cfg
        self.cmd_q = cmd_q
        self.status_q = status_q
        self.stop_event = stop_event
        self._actuals_shm = actuals_shm
        self._targets_shm = targets_shm
        self.master = Master(master_index=cfg.master_index)
        self.domain = None
        self.slave_handles: Dict[int, Any] = {}
        self.offsets: Dict[int, Dict[Tuple[int, int], int]] = {}
        self.features: Dict[int, Dict[str, Any]] = {}
        self.last_mode_cmd: Dict[int, Optional[int]] = {}
        self.last_velocity_cmd: Dict[int, Optional[float]] = {}
        self.last_torque_cmd: Dict[int, Optional[float]] = {}
        self.last_position_cmd: Dict[int, Optional[float]] = {}
        self.warned_missing_pdo: Dict[int, set] = {}
        self._last_mode_sdo: Dict[int, Optional[int]] = {}
        self._last_velocity_sdo: Dict[int, Optional[int]] = {}
        self._last_torque_sdo: Dict[int, Optional[int]] = {}
        self._last_position_sdo: Dict[int, Optional[int]] = {}
        self.last_probe_arm: Dict[int, Optional[int]] = {}
        self._sdo_mgr = _SdoRequestManager()
        self.pdo_maps: Dict[int, Dict[str, Dict[int, list]]] = {}
        # CiA 402 state machine tracking
        self.enable_step: Dict[int, int] = {}  # Fault reset attempt counter
        self.enable_last_action_ns: Dict[int, int] = {}  # Time-based enable pacing per slave
        self.slave_in_op: Dict[int, bool] = {}  # Track which slaves reached OP
        self.drive_enabled: Dict[int, bool] = {}  # Track which drives are enabled
        self.desired_controlword: Dict[int, int] = {}  # Maintain controlword state per slave
        self.cycle_count = 0  # Track cycles for OP detection
        self.last_al_state: Dict[int, Optional[int]] = {}
        # PP new-set-point pulse tracking (bit 4 behavior)
        self._pp_pulse_active: Dict[int, bool] = {}
        self._pp_pulse_start_ns: Dict[int, Optional[int]] = {}
        self._pp_pulse_pending: Dict[int, bool] = {}
        # If a new PP/HM strobe request arrives while bit 4 is still asserted, we force a one-cycle clear
        # to guarantee a fresh 0→1 edge on the next cycle.
        self._pp_force_clear_cycles: Dict[int, int] = {}
        # PV optional set-point pulse (some drives require bit 4 toggle to latch new velocity)
        self._pv_pulse_active: Dict[int, bool] = {}
        self._pv_pulse_start_ns: Dict[int, Optional[int]] = {}
        self._pv_pulse_pending: Dict[int, bool] = {}
        self._pv_force_clear_cycles: Dict[int, int] = {}

        # Manual enable/disable latch (DISABLE_DRIVE must suppress auto-enable)
        self._enable_requested: Dict[int, bool] = {}
        self._manual_disable: Dict[int, bool] = {}

        self._csp_target_next: Dict[int, Optional[int]] = {}
        self._csp_target_cur: Dict[int, Optional[int]] = {}
        self._default_mode: Dict[int, Optional[int]] = {}
        self._activated_mono_ns: Optional[int] = None
        self._cia402_positions: set = set()
        self._raw_pdo_writes: Dict[int, Dict[Tuple[int, int], bytes]] = {}
        self._status_publish_count: int = 0
        self._dc_master_sync_mode = str(getattr(cfg, "dc_master_sync_mode", "m2s") or "m2s").strip().lower()
        # Pure M2S (master to reference slave) timebase.
        self._m2s_ref_ext = _RefClock32Extender()
        self._m2s_last_app_time_ns: Optional[int] = None
        self._m2s_logged_mode: bool = False
        self._m2s_logged_fallback: bool = False
        self._m2s_logged_lock: bool = False
        self._m2s_ref_time_ns: Optional[int] = None
        self._m2s_phase_err_window: deque = deque(maxlen=max(1, int(getattr(cfg, "dc_m2s_window", 11) or 11)))
        self._m2s_k: float = float(getattr(cfg, "dc_m2s_k", 0.01) or 0.01)
        self._m2s_max_correction_ns: int = int(getattr(cfg, "dc_m2s_max_correction_ns", 20_000) or 20_000)
        self._m2s_logged_pll: bool = False
        self._jitter_warmup_cycles: int = max(0, int(getattr(self.cfg, "jitter_warmup_cycles", 0)))
        self._deadline_miss_threshold_ns: int = int(getattr(self.cfg, "deadline_miss_threshold_ns", 0) or 0)
        if self._deadline_miss_threshold_ns <= 0:
            self._deadline_miss_threshold_ns = int(float(self.cfg.cycle_time_ms) * 1_000_000.0 * 0.25)
            if self._deadline_miss_threshold_ns <= 0:
                self._deadline_miss_threshold_ns = 1
        self._last_cycle_time_ns: Optional[int] = None
        self._last_cycle_time_us: Optional[int] = None
        self._last_cycle_jitter_ns: Optional[int] = None
        self._last_cycle_jitter_us: Optional[int] = None
        self._max_abs_cycle_jitter_ns: int = 0
        self._max_abs_cycle_jitter_us: int = 0
        self._max_abs_cycle_jitter_post_warmup_ns: int = 0
        self._max_abs_cycle_jitter_post_warmup_us: int = 0
        self._jitter_window_ns: deque[int] = deque(maxlen=max(32, int(getattr(self.cfg, "jitter_window_size", 2048))))
        self._jitter_window: deque[int] = deque(maxlen=max(32, int(getattr(self.cfg, "jitter_window_size", 2048))))
        self._deadline_miss_count: int = 0
        self._overrun_count: int = 0
        self._max_overrun_ns: int = 0
        self._last_work_ns: Optional[int] = None
        self._max_work_ns: int = 0
        self._work_window_ns: deque = deque(maxlen=max(32, int(getattr(self.cfg, "jitter_window_size", 2048))))
        self._prev_send_mono: Optional[int] = None
        self._last_send_interval_ns: Optional[int] = None
        self._min_sleep_budget_ns: Optional[int] = None
        self._min_sleep_budget_window_ns: Optional[int] = None
        self._last_publish_ns: int = 0
        self._max_publish_ns: int = 0
        self._jitter_sorted_ns: list = []
        self._jitter_sorted_us: list = []
        self._dc_sync_sorted_ns: list = []
        self._work_sorted_ns: list = []
        self._log_fh = None
        self._log_q = None
        self._log_thread = None
        self._pdo_cache: Dict[int, Dict[str, int]] = {}
        self._phase_recv: int = 0
        self._phase_state: int = 0
        self._phase_cia: int = 0
        self._phase_ruckig: int = 0
        self._phase_write: int = 0
        self._phase_dc_send: int = 0
        self._phase_tail: int = 0
        self._phase_recv_max: int = 0
        self._phase_state_max: int = 0
        self._phase_cia_max: int = 0
        self._phase_ruckig_max: int = 0
        self._phase_write_max: int = 0
        self._phase_dc_send_max: int = 0
        self._phase_tail_max: int = 0
        self._log_actual_sum: int = 0
        self._log_jitter_sum: int = 0
        self._log_lateness_max: int = 0
        self._log_work_max: int = 0
        self._log_cycle_count: int = 0
        self._last_domain_wc: Optional[int] = None
        self._last_domain_wc_state: Optional[int] = None
        self._domain_wc_min: Optional[int] = None
        self._domain_wc_max: Optional[int] = None
        self._last_dc_sync_error_ns: Optional[int] = None
        self._max_abs_dc_sync_error_ns: int = 0
        self._dc_sync_error_window_ns: deque[int] = deque(maxlen=max(32, int(getattr(self.cfg, "jitter_window_size", 2048))))
        self._motion_command_block_count: int = 0
        self._op_entered_first_ns: Dict[int, int] = {}
        self._op_entered_last_ns: Dict[int, int] = {}
        self._op_left_last_ns: Dict[int, int] = {}
        self._op_dropout_count: Dict[int, int] = {}
        self._all_op_first_ns: Optional[int] = None
        self._all_op_last_ns: Optional[int] = None
        self._all_left_op_last_ns: Optional[int] = None
        self.slave_in_safeop: Dict[int, bool] = {}
        self._all_safeop_first_ns: Optional[int] = None
        self._all_safeop_logged: bool = False
        self._dc_settling_complete: bool = False
        self._dc_settling_logged: bool = False
        
        # Working counter monitoring (non-RT analysis)
        self._domain_wc_drop_count: int = 0
        self._last_wc_alert_cycle: int = 0
        self._wc_expected: int = 0
        self._slave_was_in_op: Dict[int, bool] = {}
        
        # RT budget monitoring (non-RT analysis)
        self._rt_budget_warnings: int = 0
        self._last_rt_alert_cycle: int = 0
        self._fault_active_last: Dict[int, bool] = {}
        self._fault_error_code_last: Dict[int, Optional[int]] = {}
        self._op_timeout_active: bool = False

        # Ruckig (optional CSP trajectory generation)
        self._ruckig_planner: Optional[RuckigCspPlanner] = None
        self._ruckig_requests: Dict[int, Dict[str, Any]] = {}  # slave_pos -> request dict
        self._ruckig_last_error: Dict[int, Optional[str]] = {}
        self._ruckig_trace: Dict[int, Dict[str, Any]] = {}
        self._ruckig_prev_actual_position: Dict[int, int] = {}
        self._ruckig_prev_sample_cycle: Dict[int, int] = {}
        self._ruckig_prev_velocity: Dict[int, float] = {}
        self._semi_rotary_rt: Dict[str, Any] = {
            "active": False,
            "die_pos": None,
            "shuttle_pos": None,
            "nip_in_pos": None,
            "nip_out_pos": None,
            "die_start": 0,
            "shuttle_center": 0,
            "nip_in_start": 0,
            "nip_out_start": 0,
            "die_counts_per_rev": 0.0,
            "nip_in_counts_per_rev": 0.0,
            "nip_out_counts_per_rev": 0.0,
            "comp_counts": [],
            "n_samples": 0,
            "blend_cycles": 0,
            "cycle_count": 0,
            "max_shuttle_delta_per_cycle": None,
            "max_shuttle_excursion": None,
            "enable_nips": False,
            "error": None,
            "die_phase_rt": None,
            "comp_target_rt": None,
            "comp_target_delta_rt": None,
            "rev_count_rt": None,
            "blend_rt": None,
            "comp_raw_rt": None,
            "die_commanded_position": None,
            "target_counts_per_rev": None,
            "velocity_ramp_rate": 100000.0,
            "phase_offset": 0.0,
            "die_velocity_counts_per_s": None,
            "target_velocity_counts_per_s": None,
        }
        
        self._die_velocity_test_active = False
        self._die_velocity_integrator = None
        self._die_test_target_rpm = 0.0
        self._die_test_target_velocity = 0.0
        self._die_test_pos = 0
        
        raw_process_log_file = getattr(self.cfg, "process_log_file", None)
        if raw_process_log_file is None:
            self._process_log_file = None
        else:
            candidate = str(raw_process_log_file).strip()
            self._process_log_file = None if (not candidate or candidate.lower() == "none") else candidate

    def _emit_process_log(self, message: str, stderr: Optional[bool] = None) -> None:
        if stderr is None:
            stderr = bool(getattr(self.cfg, "process_log_stderr", True))
        
        # Add dmesg-style timestamp for state transition logs
        if any(x in message for x in ['entered OP', 'left OP', 'entered SAFEOP', 'CIA402']):
            from datetime import datetime
            ts = datetime.now().strftime('[%a %b %d %H:%M:%S %Y]')
            line = f"{ts} {message}"
        else:
            line = message
            
        line = line if line.endswith("\n") else (line + "\n")
        if self._log_q is not None:
            try:
                self._log_q.put_nowait((line, bool(self._process_log_file), stderr))
            except Exception:
                pass
        elif self._process_log_file or stderr:
            if self._process_log_file:
                if self._log_fh is None:
                    try:
                        self._log_fh = open(self._process_log_file, "a", encoding="utf-8")
                    except Exception:
                        self._process_log_file = None
                if self._log_fh is not None:
                    try:
                        self._log_fh.write(line)
                        self._log_fh.flush()
                    except Exception:
                        pass
            if stderr:
                try:
                    sys.stderr.write(line)
                    sys.stderr.flush()
                except Exception:
                    pass

    @staticmethod
    def _window_append(window: deque, sorted_list: list, value: int) -> None:
        if len(window) == window.maxlen:
            old = window[0]
            idx = bisect.bisect_left(sorted_list, old)
            if idx < len(sorted_list) and sorted_list[idx] == old:
                del sorted_list[idx]
        window.append(value)
        bisect.insort(sorted_list, value)

    def _write_actuals_to_shm(self) -> None:
        shm = self._actuals_shm
        if shm is None:
            return
        shm[SHM.SLOT_CYCLE_COUNT] = int(self.cycle_count)
        shm[SHM.SLOT_TIMESTAMP_NS] = time.time_ns()
        shm[SHM.SLOT_JITTER_NS] = int(self._last_cycle_jitter_ns or 0)
        shm[SHM.SLOT_MAX_JITTER_PW_NS] = int(self._max_abs_cycle_jitter_post_warmup_ns)
        shm[SHM.SLOT_WORK_NS] = int(self._last_work_ns or 0)
        shm[SHM.SLOT_MAX_WORK_NS] = int(self._max_work_ns)
        shm[SHM.SLOT_OVERRUN_COUNT] = int(self._overrun_count)
        shm[SHM.SLOT_MAX_OVERRUN_NS] = int(self._max_overrun_ns)
        shm[SHM.SLOT_SEND_INTERVAL_NS] = int(self._last_send_interval_ns or 0)
        shm[SHM.SLOT_MIN_SLEEP_BUDGET_NS] = int(self._min_sleep_budget_ns or 0)
        shm[SHM.SLOT_DEADLINE_MISS_COUNT] = int(self._deadline_miss_count)
        shm[SHM.SLOT_DC_SYNC_ERR_NS] = int(self._last_dc_sync_error_ns or 0)
        shm[SHM.SLOT_DC_SYNC_ERR_MAX_NS] = int(self._max_abs_dc_sync_error_ns)
        shm[SHM.SLOT_DOMAIN_WC] = int(self._last_domain_wc or 0)
        shm[SHM.SLOT_DOMAIN_WC_STATE] = int(self._last_domain_wc_state or 0)
        shm[SHM.SLOT_DOMAIN_WC_MIN] = int(self._domain_wc_min or 0)
        shm[SHM.SLOT_DOMAIN_WC_MAX] = int(self._domain_wc_max or 0)
        shm[SHM.SLOT_ALL_OP_FIRST_NS] = int(self._all_op_first_ns or 0)
        shm[SHM.SLOT_ALL_OP_LAST_NS] = int(self._all_op_last_ns or 0)
        shm[SHM.SLOT_ALL_OP_LEFT_LAST_NS] = int(self._all_left_op_last_ns or 0)
        shm[SHM.SLOT_CYCLE_TIME_MS_X1000] = int(self.cfg.cycle_time_ms * 1000)
        for slave_pos, entries in self.offsets.items():
            if slave_pos in self._cia402_positions:
                continue
            dcfg = next((d for d in self.cfg.slaves if d.position == slave_pos), None)
            if dcfg is None:
                continue
            reg_entries = getattr(dcfg, 'register_entries', None) or []
            di_idx = 0
            do_idx = 0
            for key in reg_entries:
                if key not in entries:
                    continue
                idx = key[0] if isinstance(key, tuple) else key
                if 0x6000 <= idx < 0x6040:
                    if di_idx < SHM.SLOT_IO_DI_COUNT:
                        raw = self.master.read_domain(self.domain, entries[key], 1) or b"\x00"
                        shm[SHM.SLOT_IO_DI_BASE + di_idx] = raw[0]
                        di_idx += 1
                elif 0x7040 <= idx < 0x7080:
                    if do_idx < SHM.SLOT_IO_DO_COUNT:
                        raw = self.master.read_domain(self.domain, entries[key], 1) or b"\x00"
                        shm[SHM.SLOT_IO_DO_BASE + do_idx] = raw[0]
                        do_idx += 1
        cache = self._pdo_cache
        for slave_pos, entries in self.offsets.items():
            if slave_pos >= SHM.MAX_DRIVES:
                continue
            base = SHM.DRIVE_BASE + slave_pos * SHM.DRIVE_STRIDE
            c = cache.get(slave_pos)
            shm[base + SHM.DRIVE_STATUSWORD] = c.get('sw', 0) if c else 0
            if (POSITION_ACTUAL_INDEX, 0) in entries:
                raw_p = self.master.read_domain(self.domain, entries[(POSITION_ACTUAL_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                shm[base + SHM.DRIVE_POSITION] = int.from_bytes(raw_p, 'little', signed=True)
            else:
                shm[base + SHM.DRIVE_POSITION] = 0
            if (VELOCITY_ACTUAL_INDEX, 0) in entries:
                raw_v = self.master.read_domain(self.domain, entries[(VELOCITY_ACTUAL_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                shm[base + SHM.DRIVE_VELOCITY] = int.from_bytes(raw_v, 'little', signed=True)
            else:
                shm[base + SHM.DRIVE_VELOCITY] = 0
            if (TORQUE_ACTUAL_INDEX, 0) in entries:
                raw = self.master.read_domain(self.domain, entries[(TORQUE_ACTUAL_INDEX, 0)], 2) or b"\x00\x00"
                shm[base + SHM.DRIVE_TORQUE] = int.from_bytes(raw, 'little', signed=True)
            if (MODES_OP_DISPLAY_INDEX, 0) in entries:
                raw = self.master.read_domain(self.domain, entries[(MODES_OP_DISPLAY_INDEX, 0)], 1) or b"\x00"
                shm[base + SHM.DRIVE_MODE] = raw[0]
            if (ERROR_CODE_INDEX, 0) in entries:
                raw = self.master.read_domain(self.domain, entries[(ERROR_CODE_INDEX, 0)], 2) or b"\x00\x00"
                shm[base + SHM.DRIVE_ERROR_CODE] = int.from_bytes(raw, 'little')
            if (DIGITAL_INPUTS_INDEX, 0) in entries:
                raw = self.master.read_domain(self.domain, entries[(DIGITAL_INPUTS_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                shm[base + SHM.DRIVE_DIGITAL_INPUTS] = int.from_bytes(raw, 'little')
            if (DIP_IN_STATE_INDEX, 1) in entries:
                raw = self.master.read_domain(self.domain, entries[(DIP_IN_STATE_INDEX, 1)], 4) or b"\x00\x00\x00\x00"
                shm[base + SHM.DRIVE_DIP_IN_STATE] = int.from_bytes(raw, 'little')
            shm[base + SHM.DRIVE_IN_OP] = 1 if self.slave_in_op.get(slave_pos, False) else 0
            shm[base + SHM.DRIVE_ENABLED] = 1 if (self.drive_enabled.get(slave_pos, False) and not self._manual_disable.get(slave_pos, False)) else 0
        shm[SHM.SLOT_SEQUENCE] = int(self.cycle_count)

    def _read_targets_from_shm(self) -> None:
        shm = self._targets_shm
        if shm is None:
            return
        for pos in range(SHM.MAX_DRIVES):
            base = SHM.TARGET_BASE + pos * SHM.TARGET_STRIDE
            if shm[base + SHM.TARGET_VALID] == 1:
                self._csp_target_next[pos] = int(shm[base + SHM.TARGET_CSP])
                self.last_mode_cmd[pos] = int(shm[base + SHM.TARGET_MODE])
                shm[base + SHM.TARGET_VALID] = 0

    @staticmethod
    def _percentile_value(values: List[int], percentile: float) -> Optional[int]:
        if not values:
            return None
        if percentile <= 0:
            return values[0]
        if percentile >= 100:
            return values[-1]
        idx = int(round((percentile / 100.0) * (len(values) - 1)))
        if idx < 0:
            idx = 0
        if idx >= len(values):
            idx = len(values) - 1
        return values[idx]

    @staticmethod
    def _fault_code_label(error_code: Optional[int]) -> str:
        if error_code is None:
            return "Unknown"
        return FAULT_CODE_LABELS.get(int(error_code), "Unknown")

    def _read_fault_error_code(self, slave_pos: int, entries: Dict[Tuple[int, int], int]) -> Tuple[Optional[int], str]:
        error_code: Optional[int] = None
        
        # Try PDO first
        if (ERROR_CODE_INDEX, 0) in entries:
            raw_ec = self.master.read_domain(self.domain, entries[(ERROR_CODE_INDEX, 0)], 2)
            if raw_ec:
                error_code = int.from_bytes(raw_ec, "little")
                if error_code not in (None, 0):
                    return error_code, "pdo"

        # If we don't have a runtime SDO request object registered, we cannot use SDO-request-object fallback.
        if not self._sdo_mgr.has(int(slave_pos), int(ERROR_CODE_INDEX), 0):
            return None, "unavailable"

        # Try cached SDO-request-object result (serviced via cyclic send/receive).
        cache_age_cycles = int(getattr(self.cfg, "fault_error_code_cache_max_age_cycles", 500) or 500)
        cached = self._sdo_mgr.get_cached_read(
            slave_pos, ERROR_CODE_INDEX, 0,
            cycle_count=int(self.cycle_count),
            max_age_cycles=cache_age_cycles,
        )
        if cached and len(cached) >= 2:
            ec = int.from_bytes(cached[:2], "little")
            if ec != 0:
                return ec, "sdo_req_cached"

        # Queue (deduped) SDO read via request object.
        if bool(getattr(self.cfg, "fault_error_code_sdo_fallback", True)):
            if not self._sdo_queue_allowed_now():
                return None, "sdo_req_suppressed"
            min_interval = int(getattr(self.cfg, "fault_error_code_read_min_interval_cycles", 500) or 500)
            self._sdo_mgr.set_desired_read(
                slave_pos, ERROR_CODE_INDEX, 0,
                min_interval_cycles=min_interval,
            )
            return None, "sdo_req_pending"

        return None, "unavailable"

    def _configure_sync_manager_watchdog(self, dcfg: DriveConfig, s) -> None:
        """
        Best-effort configure the ESC SyncManager watchdog for a slave.

        This targets AL 0x001B "Sync manager watchdog" by adjusting the slave's
        watchdog divider (reg 0x0400) and intervals (reg 0x0420) using the IgH
        configuration API (ecrt_slave_config_watchdog). This is NOT a CoE SDO.
        """
        timeout_ms = getattr(dcfg, "sm_watchdog_timeout_ms", None)
        if timeout_ms is None:
            timeout_ms = getattr(self.cfg, "sm_watchdog_timeout_ms", None)
        if timeout_ms is None:
            env = os.getenv("ETHERCAT_SM_WATCHDOG_TIMEOUT_MS")
            if env is not None and env.strip():
                try:
                    timeout_ms = float(env.strip())
                except ValueError:
                    timeout_ms = None
        if timeout_ms is None:
            return

        try:
            timeout_ms_f = float(timeout_ms)
        except Exception:
            return
        if timeout_ms_f <= 0.0:
            return

        divider = getattr(dcfg, "sm_watchdog_divider", None)
        if divider is None:
            divider = getattr(self.cfg, "sm_watchdog_divider", None)
        if divider is None:
            divider = _SM_WD_DEFAULT_DIVIDER_40NS

        try:
            divider_i = int(divider)
        except Exception:
            divider_i = _SM_WD_DEFAULT_DIVIDER_40NS
        divider_i = max(1, min(0xFFFF, divider_i))

        base_ns = divider_i * 40  # per ecrt.h
        intervals = int(round((timeout_ms_f * 1_000_000.0) / float(base_ns)))
        intervals = max(1, min(0xFFFF, intervals))

        try:
            self.master.slave_config_watchdog(s, divider_i, intervals)
            approx_ms = (intervals * base_ns) / 1_000_000.0
            logger.info(
                f"Slave {dcfg.position}: SM watchdog configured "
                f"(divider={divider_i}, intervals={intervals}, approx={approx_ms:.1f}ms)"
            )
        except Exception as e:
            logger.warning(f"Slave {dcfg.position}: SM watchdog config failed: {e}")

    def _setup(self) -> bool:
        # Open/request the master with or without PDO based on config
        if not self.master.request(
            sdo_only=self.cfg.sdo_only,
            force_release_device=bool(getattr(self.cfg, "force_release_master_on_startup", False)),
            device_path=str(getattr(self.cfg, "ethercat_device_path", "/dev/EtherCAT0")),
            sigterm_first=bool(getattr(self.cfg, "force_release_sigterm_first", True)),
            retry_delay_s=float(getattr(self.cfg, "force_release_retry_delay_s", 1.0)),
            force_release_attempts=int(getattr(self.cfg, "force_release_attempts", 3)),
            force_release_debug_owners=bool(getattr(self.cfg, "force_release_debug_owners", True)),
        ):
            logger.error("Failed to request master")
            return False

        # Create domain only if PDO is desired
        if not self.cfg.sdo_only:
            self.domain = self.master.create_domain()

        for dcfg in self.cfg.slaves:
            if dcfg.vendor_id is None or dcfg.product_code is None:
                logger.error(f"Slave {dcfg.position} missing vendor_id/product_code")
                return False

            s = self.master.config_slave(dcfg.alias, dcfg.position, dcfg.vendor_id, dcfg.product_code)
            self.slave_handles[dcfg.position] = s
            self._default_mode[dcfg.position] = dcfg.operation_mode
            if dcfg.operation_mode is not None and dcfg.position not in self.last_mode_cmd:
                self.last_mode_cmd[dcfg.position] = int(dcfg.operation_mode)

            if getattr(dcfg, 'cia402', True):
                self._cia402_positions.add(dcfg.position)

            if not self._setup_runtime_sdo_requests_for_slave(dcfg.position, s, is_cia402=bool(getattr(dcfg, "cia402", True))):
                return False

            self._configure_sync_manager_watchdog(dcfg, s)

            for (idx, sub, data) in getattr(dcfg, 'startup_sdos', []):
                self.master.slave_config_sdo(s, idx, sub, data)

            if dcfg.xml:
                decoded = decode_esi(
                    dcfg.xml.xml_file,
                    vendor_id=dcfg.vendor_id,
                    product_code=dcfg.product_code,
                )
                rx_pdos = dcfg.pdo.rx_pdos if dcfg.pdo and dcfg.pdo.rx_pdos else decoded.rx_pdos
                tx_pdos = dcfg.pdo.tx_pdos if dcfg.pdo and dcfg.pdo.tx_pdos else decoded.tx_pdos

                rx_pdo_map = decoded.pdo_map_rx
                tx_pdo_map = decoded.pdo_map_tx
                if dcfg.pdo and dcfg.pdo.custom_pdo_config:
                    custom = dcfg.pdo.custom_pdo_config
                    if rx_pdos and custom.get('rx_entries'):
                        rx_pdo_map = {rx_pdos[0]: custom['rx_entries']}
                    if tx_pdos and custom.get('tx_entries'):
                        tx_pdo_map = {tx_pdos[0]: custom['tx_entries']}

                flat_rx = [e for entries in rx_pdo_map.values() for e in entries]
                flat_tx = [e for entries in tx_pdo_map.values() for e in entries]
                supports = dict(decoded.supports or {})
                supports.update(
                    {
                        "controlword": any(e[0] == CW_INDEX for e in flat_rx),
                        "statusword": any(e[0] == SW_INDEX for e in flat_tx),
                        "mode_command": any(e[0] == MODES_OP_INDEX for e in flat_rx),
                        "mode_display": any(e[0] == MODES_OP_DISPLAY_INDEX for e in flat_tx),
                        "touch_probe": any(
                            e[0]
                            in (
                                PROBE_FUNCTION_INDEX,
                                PROBE_STATUS_INDEX,
                                PROBE_POS1_INDEX,
                                PROBE_POS2_INDEX,
                                PROBE_POS2_ALT_INDEX,
                            )
                            for e in (flat_rx + flat_tx)
                        ),
                    }
                )

                sync_configs = []
                if rx_pdos:
                    sync_configs.append((2, 1, [(p, rx_pdo_map.get(p, [])) for p in rx_pdos]))
                if tx_pdos:
                    sync_configs.append((3, 2, [(p, tx_pdo_map.get(p, [])) for p in tx_pdos]))

                register_list = []
                for sm_idx, direction, pdos in sync_configs:
                    for pdo_idx, pdo_entries in pdos:
                        for idx, sub, _bits in pdo_entries:
                            register_list.append((idx, sub))

            elif getattr(dcfg, 'sync_configs', None) is not None:
                sync_configs = dcfg.sync_configs
                register_list = list(getattr(dcfg, 'register_entries', None) or [])

                rx_pdo_map = {}
                tx_pdo_map = {}
                for sm_idx, direction, pdos in sync_configs:
                    for pdo_idx, pdo_entries in pdos:
                        if direction == 1:
                            rx_pdo_map[pdo_idx] = pdo_entries
                        elif direction == 2:
                            tx_pdo_map[pdo_idx] = pdo_entries

                flat_rx = [e for ent in rx_pdo_map.values() for e in ent]
                flat_tx = [e for ent in tx_pdo_map.values() for e in ent]
                supports = {
                    "controlword": any(e[0] == CW_INDEX for e in flat_rx),
                    "statusword": any(e[0] == SW_INDEX for e in flat_tx),
                    "mode_command": any(e[0] == MODES_OP_INDEX for e in flat_rx),
                    "mode_display": any(e[0] == MODES_OP_DISPLAY_INDEX for e in flat_tx),
                }
            else:
                logger.error(f"Slave {dcfg.position}: requires either xml or sync_configs")
                return False

            if getattr(dcfg, "features_overrides", None):
                supports.update(dcfg.features_overrides)

            if not self.cfg.sdo_only:
                self.master.configure_slave_pdos(s, sync_configs)

                offsets = self.master.register_pdo_entry_list(
                    self.domain, dcfg.alias, dcfg.position, dcfg.vendor_id, dcfg.product_code, register_list
                )
                self.offsets[dcfg.position] = offsets
                self.pdo_maps[dcfg.position] = {
                    'rx': rx_pdo_map,
                    'tx': tx_pdo_map,
                }
                
                sys.stdout.write(f"[SETUP] Slave {dcfg.position}: Registered {len(offsets)} PDO entries\n")
                if (DIP_IN_STATE_INDEX, 1) in offsets:
                    sys.stdout.write(f"[SETUP] Slave {dcfg.position}: 0x4020:1 registered at offset {offsets[(DIP_IN_STATE_INDEX, 1)]}\n")
                else:
                    sys.stdout.write(f"[SETUP] Slave {dcfg.position}: 0x4020:1 NOT registered!\n")
                    sys.stdout.write(f"[SETUP] Slave {dcfg.position}: Available entries: {[(hex(k[0]), k[1]) for k in list(offsets.keys())[:10]]}\n")
                sys.stdout.flush()

            self.features[dcfg.position] = supports
            auto_en = getattr(self.cfg, "auto_enable_cia402", True)
            self._enable_requested.setdefault(dcfg.position, auto_en and dcfg.position in self._cia402_positions)
            self._manual_disable.setdefault(dcfg.position, False)
            self._op_dropout_count.setdefault(dcfg.position, 0)

            if dcfg.enable_dc and not self.cfg.sdo_only:
                cycle_time_ns = int(self.cfg.cycle_time_ms * 1_000_000)
                dc_assign = dcfg.dc_assign_activate if dcfg.dc_assign_activate is not None else 0x0300
                sync0_cycle = int(dcfg.dc_sync0_cycle_time_ns) if dcfg.dc_sync0_cycle_time_ns is not None else cycle_time_ns
                sync0_shift = dcfg.dc_sync0_shift_ns
                sync1_cycle = dcfg.dc_sync1_cycle_time_ns
                sync1_shift = dcfg.dc_sync1_shift_ns

                s.config_dc(
                    assign_activate=dc_assign,
                    sync0_cycle_time_ns=sync0_cycle,
                    sync0_shift_ns=sync0_shift,
                    sync1_cycle_time_ns=sync1_cycle,
                    sync1_shift_ns=sync1_shift
                )
                logger.info(f"DC configured for slave {dcfg.position}")

        if any(bool(getattr(d, "ruckig", None) and getattr(d.ruckig, "enabled", False)) for d in self.cfg.slaves):
            self._ruckig_planner = RuckigCspPlanner()
            if not self._ruckig_planner.available():
                logger.warning("Ruckig is enabled in config but the ruckig package is not available.")

        if not self.cfg.sdo_only:
            for dcfg in self.cfg.slaves:
                if dcfg.position not in self._cia402_positions:
                    continue
                entries = self.offsets.get(dcfg.position, {})
                missing = []
                if (CW_INDEX, 0) not in entries:
                    missing.append(f"0x{CW_INDEX:04X}:0 (controlword)")
                if (SW_INDEX, 0) not in entries:
                    missing.append(f"0x{SW_INDEX:04X}:0 (statusword)")
                if missing:
                    raise RuntimeError(
                        f"CiA402 slave {dcfg.position} requires 0x6040/0x6041 mapped. Missing: {', '.join(missing)}"
                    )

            # Select DC reference clock first
            dc_ref_pos = getattr(self.cfg, 'dc_reference_slave', None)
            selected_dc_ref = None
            
            if dc_ref_pos is not None:
                s = self.slave_handles.get(dc_ref_pos)
                if s:
                    self.master.select_reference_clock(s)
                    logger.info(f"[DC] Selected slave {dc_ref_pos} as DC reference clock (explicit)")
                    selected_dc_ref = dc_ref_pos
                else:
                    logger.error(f"dc_reference_slave={dc_ref_pos} not found in slave_handles")
            else:
                for dcfg in self.cfg.slaves:
                    if dcfg.enable_dc:
                        s = self.slave_handles.get(dcfg.position)
                        if s:
                            self.master.select_reference_clock(s)
                            logger.info(f"[DC] Selected slave {dcfg.position} as DC reference clock (auto, first DC-enabled)")
                            selected_dc_ref = dcfg.position
                            break

            if not self._m2s_logged_mode:
                if self._dc_master_sync_mode == "m2s":
                    self._emit_process_log(
                        f"[DC] Master clock mode=M2S (app_time <- reference_clock_time_32, sync_reference_clock DISABLED), "
                        f"dc_reference_slave={selected_dc_ref}",
                        stderr=True,
                    )
                else:
                    self._emit_process_log(
                        f"[DC] Master clock mode=S2M (app_time <- monotonic_ns, sync_reference_clock ENABLED), "
                        f"dc_reference_slave={selected_dc_ref}",
                        stderr=True,
                    )
                self._m2s_logged_mode = True

            # Seed app_time once before activation. For pure M2S we will quickly
            # overwrite this with the reference clock time once sync datagrams flow.
            try:
                self.master.set_application_time(int(time.monotonic_ns()))
            except Exception:
                pass

            for dcfg in self.cfg.slaves:
                if dcfg.position not in self._cia402_positions:
                    continue
                entries = self.offsets.get(dcfg.position, {})
                mode = self.last_mode_cmd.get(dcfg.position)
                if mode is not None and (MODES_OP_INDEX, 0) not in entries:
                    s = self.slave_handles.get(dcfg.position)
                    if s:
                        self.master.slave_config_sdo(s, MODES_OP_INDEX, 0, bytes([mode]))
                        logger.info(f"Slave {dcfg.position}: mode 0x{mode:02X} registered as startup SDO")
                    self._last_mode_sdo[dcfg.position] = mode

            for dcfg in self.cfg.slaves:
                if dcfg.position not in self._cia402_positions:
                    continue
                entries = self.offsets.get(dcfg.position, {})
                if (TARGET_TORQUE_INDEX, 0) not in entries:
                    continue
                s = self.slave_handles.get(dcfg.position)
                if s:
                    try:
                        self.master.slave_config_sdo(s, MAX_TORQUE_INDEX, 0, (1000).to_bytes(2, 'little', signed=True))
                        logger.info(f"Slave {dcfg.position}: 0x6072 (max torque) set to 1000 (100%% per-mille)")
                    except Exception as e:
                        logger.warning(f"Slave {dcfg.position}: could not set 0x6072: {e}")

            self.master.activate()
            logger.info("Master activated")
            
            # Boost IGH master thread AFTER activation (when EtherCAT-OP spawns)
            self._boost_igh_master_priority()
            self._activated_mono_ns = time.monotonic_ns()

            for dcfg in self.cfg.slaves:
                self.slave_in_op[dcfg.position] = False
                self.slave_in_safeop[dcfg.position] = False
                self.last_al_state[dcfg.position] = None
                self._pp_pulse_active[dcfg.position] = False
                self._pp_pulse_start_ns[dcfg.position] = None
                self._pp_pulse_pending[dcfg.position] = False
                self.enable_last_action_ns[dcfg.position] = 0
                self.enable_step[dcfg.position] = 0
            if getattr(self.cfg, "force_disable_on_start", False):
                for pos in self._cia402_positions:
                    self._manual_disable[pos] = True
                    self._enable_requested[pos] = False
                    self.desired_controlword[pos] = 0x0000
                    self.drive_enabled[pos] = False

        return True

    def _teardown(self):
        self._graceful_drive_shutdown()
        try:
            self.master.deactivate()
        except Exception:
            pass
        try:
            self.master.release()
        except Exception:
            pass
        if self._log_q is not None:
            try:
                self._log_q.put_nowait(None)
            except Exception:
                pass
        if self._log_thread is not None:
            self._log_thread.join(timeout=2.0)
            self._log_thread = None
        self._log_q = None
        if self._log_fh is not None:
            try:
                self._log_fh.flush()
                self._log_fh.close()
            except Exception:
                pass
            self._log_fh = None

    def _graceful_drive_shutdown(self):
        if self.cfg.sdo_only or self.domain is None:
            return
        cycle_ns = int(self.cfg.cycle_time_ms * 1_000_000)
        shutdown_cycles = max(int(500_000_000 / cycle_ns), 50)
        logger.info(f"Graceful shutdown: disabling all drives over {shutdown_cycles} cycles...")
        for pos in self._cia402_positions:
            entries = self.offsets.get(pos, {})
            if (TARGET_VELOCITY_INDEX, 0) in entries:
                try:
                    self.master.write_domain(
                        self.domain, entries[(TARGET_VELOCITY_INDEX, 0)],
                        int(0).to_bytes(4, byteorder='little', signed=True))
                except Exception:
                    pass
            if (TARGET_POSITION_INDEX, 0) in entries:
                raw = self.master.read_domain(self.domain, entries[(TARGET_POSITION_INDEX, 0)], 4)
                if raw:
                    try:
                        self.master.write_domain(
                            self.domain, entries[(TARGET_POSITION_INDEX, 0)], raw)
                    except Exception:
                        pass
            self.desired_controlword[pos] = 0x0000
        for i in range(shutdown_cycles):
            try:
                self.master.receive()
                self.master.process_domain(self.domain)
                for pos in self._cia402_positions:
                    entries = self.offsets.get(pos, {})
                    if (CW_INDEX, 0) in entries:
                        self.master.write_domain(
                            self.domain, entries[(CW_INDEX, 0)],
                            int(0x0000).to_bytes(2, byteorder='little'))
                self.master.queue_domain(self.domain)
                self.master.send()
            except Exception:
                break
            time.sleep(cycle_ns / 1_000_000_000.0)
        logger.info("Graceful shutdown complete - drives disabled")

    def _handle_command(self, cmd: Command):
        if getattr(self.cfg, "forbid_motion_commands", False):
            blocked = {
                CommandType.SET_VELOCITY_MODE,
                CommandType.SET_POSITION_MODE,
                CommandType.SET_CSP_MODE,
                CommandType.SET_TORQUE_MODE,
                CommandType.SET_VELOCITY,
                CommandType.SET_POSITION,
                CommandType.SET_POSITION_CSP,
                CommandType.START_HOMING,
                CommandType.SET_TORQUE,
                CommandType.START_RUCKIG_POSITION,
                CommandType.START_RUCKIG_VELOCITY,
                CommandType.START_SEMI_ROTARY_RT,
                CommandType.UPDATE_SEMI_ROTARY_RT,
                CommandType.ENABLE_DRIVE,
            }
            if cmd.type in blocked:
                self._motion_command_block_count += 1
                logger.warning(f"Blocked command in no-motion mode: {cmd.type.name} target={cmd.target_id}")
                return
        # Store intent; cyclic writer will realize it via PDO or SDO
        if cmd.type == CommandType.SET_VELOCITY_MODE:
            self.last_mode_cmd[cmd.target_id] = MODE_PV
        elif cmd.type == CommandType.SET_POSITION_MODE:
            self.last_mode_cmd[cmd.target_id] = MODE_PP
        elif cmd.type == CommandType.SET_CSP_MODE:
            self.last_mode_cmd[cmd.target_id] = MODE_CSP
        elif cmd.type == CommandType.SET_TORQUE_MODE:
            self.last_velocity_cmd[cmd.target_id] = 0.0
            self.last_mode_cmd[cmd.target_id] = MODE_PT
        elif cmd.type == CommandType.SET_VELOCITY:
            raw = float(cmd.value or 0.0)
            dcfg = next((d for d in self.cfg.slaves if d.position == cmd.target_id), None)
            cap = getattr(dcfg, 'max_velocity', None)
            if cap is None:
                cap = 500000.0
            if abs(raw) > cap:
                raw = cap if raw > 0 else -cap
            self.last_velocity_cmd[cmd.target_id] = raw
            # If this drive requires a set-point pulse to latch PV targets, schedule it.
            dcfg = next((d for d in self.cfg.slaves if d.position == cmd.target_id), None)
            if dcfg and getattr(dcfg, 'pv_requires_setpoint_toggle', False):
                # If a pulse is already active, force a one-cycle clear to guarantee a fresh edge.
                if self._pv_pulse_active.get(cmd.target_id, False):
                    self._pv_force_clear_cycles[cmd.target_id] = 1
                self._pv_pulse_pending[cmd.target_id] = True
        elif cmd.type in (CommandType.SET_POSITION, CommandType.SET_POSITION_CSP):
            if cmd.type == CommandType.SET_POSITION_CSP:
                # CSP: do NOT mix with PP latching semantics; maintain a streaming target (atomic swap per cycle).
                self._csp_target_next[cmd.target_id] = int(float(cmd.value or 0.0))
                # Manual CSP streaming overrides any internal trajectory generator for safety.
                self._ruckig_requests.pop(cmd.target_id, None)
                if self._ruckig_planner:
                    self._ruckig_planner.stop(cmd.target_id)
            else:
                # PP: store target and generate a "new set-point" pulse (controlword bit 4) to latch it.
                self.last_position_cmd[cmd.target_id] = float(cmd.value or 0.0)
                # If a pulse is already active, force a one-cycle clear so the next cycle has a clean 0→1 edge.
                if self._pp_pulse_active.get(cmd.target_id, False):
                    self._pp_force_clear_cycles[cmd.target_id] = 1
                self._pp_pulse_pending[cmd.target_id] = True
        elif cmd.type == CommandType.START_HOMING:
            # Switch to HM mode and issue one-cycle start
            self.last_mode_cmd[cmd.target_id] = MODE_HM
            # Reuse position strobe to trigger new-set-point in HM
            self.last_position_cmd[cmd.target_id] = float(0)
            # CiA402 homing start is typically triggered by controlword bit 4 in HM mode.
            if self._pp_pulse_active.get(cmd.target_id, False):
                self._pp_force_clear_cycles[cmd.target_id] = 1
            self._pp_pulse_pending[cmd.target_id] = True
        elif cmd.type == CommandType.ARM_PROBE:
            probe_value = cmd.params.get('probe_function') if cmd.params else None
            if probe_value is not None:
                self.last_probe_arm[cmd.target_id] = int(probe_value)
        elif cmd.type == CommandType.SET_TORQUE:
            raw = float(cmd.value or 0.0)
            dcfg = next((d for d in self.cfg.slaves if d.position == cmd.target_id), None)
            cap = getattr(dcfg, 'max_torque', None)
            if cap is None:
                cap = 800.0
            if abs(raw) > cap:
                raw = cap if raw > 0 else -cap
            self.last_torque_cmd[cmd.target_id] = raw
        elif cmd.type == CommandType.STOP_MOTION:
            self.last_velocity_cmd[cmd.target_id] = 0.0
            self.last_torque_cmd[cmd.target_id] = 0.0
            # Stop any internal trajectory as well.
            self._ruckig_requests.pop(cmd.target_id, None)
            if self._ruckig_planner:
                self._ruckig_planner.stop(cmd.target_id)
        elif cmd.type == CommandType.START_RUCKIG_POSITION:
            # Force CSP when starting Ruckig motion
            self.last_mode_cmd[cmd.target_id] = MODE_CSP
            self._ruckig_requests[cmd.target_id] = {
                "kind": "position",
                "target": int(float(cmd.value or 0.0)),
                "params": cmd.params or {},
            }
        elif cmd.type == CommandType.START_RUCKIG_VELOCITY:
            self.last_mode_cmd[cmd.target_id] = MODE_CSP
            self._ruckig_requests[cmd.target_id] = {
                "kind": "velocity",
                "target": float(cmd.value or 0.0),
                "params": cmd.params or {},
            }
        elif cmd.type == CommandType.STOP_RUCKIG:
            self._ruckig_requests.pop(cmd.target_id, None)
            if self._ruckig_planner:
                self._ruckig_planner.stop(cmd.target_id)
        elif cmd.type == CommandType.START_SEMI_ROTARY_RT:
            params = dict(cmd.params or {})
            try:
                comp_counts = params.get("comp_counts") or []
                n_samples = int(params.get("n_samples") or len(comp_counts))
                if not comp_counts or n_samples <= 0:
                    raise ValueError("comp_counts/n_samples required")
                die_pos = int(params["die_pos"])
                shuttle_pos = int(params["shuttle_pos"])
                
                use_ruckig = params.get("use_ruckig_velocity_interface", False)
                ruckig_integrator = None
                nip_in_integrator = None
                nip_out_integrator = None
                nip_in_target_velocity = 0.0
                nip_out_target_velocity = 0.0
                
                if use_ruckig and RUCKIG_AVAILABLE_CORE:
                    try:
                        dt_s = float(self.cfg.cycle_time_ms) / 1000.0
                        max_velocity = float(params.get("max_velocity", 655360.0))
                        max_acceleration = float(params.get("max_acceleration", 5000000.0))
                        max_jerk = float(params.get("max_jerk", 20000000.0))
                        
                        ruckig_integrator = RuckigVelocityIntegrator(
                            dt_s=dt_s,
                            max_velocity=max_velocity,
                            max_acceleration=max_acceleration,
                            max_jerk=max_jerk
                        )
                        print(f"[RT-INIT] Ruckig velocity interface enabled for die: max_v={max_velocity:.0f}, max_a={max_acceleration:.0f}, max_j={max_jerk:.0f}", flush=True)
                        
                        if params.get("enable_nips", False):
                            line_speed_mps = float(params.get("line_speed_mps", 0.0))
                            nip_in_counts_per_rev = float(params.get("nip_in_counts_per_rev", 0.0))
                            nip_out_counts_per_rev = float(params.get("nip_out_counts_per_rev", 0.0))
                            nip_in_diameter_m = float(params.get("nip_in_roller_diameter_m", 0.1))
                            nip_out_diameter_m = float(params.get("nip_out_roller_diameter_m", 0.1))
                            
                            if line_speed_mps > 0:
                                nip_in_target_velocity = ContinuousRotationTrajectory.calculate_nip_target_velocity(
                                    line_speed_mps=line_speed_mps,
                                    roller_diameter_m=nip_in_diameter_m,
                                    counts_per_rev=nip_in_counts_per_rev
                                )
                                nip_out_target_velocity = ContinuousRotationTrajectory.calculate_nip_target_velocity(
                                    line_speed_mps=line_speed_mps,
                                    roller_diameter_m=nip_out_diameter_m,
                                    counts_per_rev=nip_out_counts_per_rev
                                )
                                
                                nip_in_integrator = RuckigVelocityIntegrator(
                                    dt_s=dt_s,
                                    max_velocity=max_velocity,
                                    max_acceleration=max_acceleration,
                                    max_jerk=max_jerk
                                )
                                nip_out_integrator = RuckigVelocityIntegrator(
                                    dt_s=dt_s,
                                    max_velocity=max_velocity,
                                    max_acceleration=max_acceleration,
                                    max_jerk=max_jerk
                                )
                                print(f"[RT-INIT] Ruckig velocity interface enabled for nips: in_target={nip_in_target_velocity:.0f}, out_target={nip_out_target_velocity:.0f}", flush=True)
                    except Exception as e:
                        print(f"[RT-INIT] Ruckig init failed, falling back to linear ramp: {e}", flush=True)
                        ruckig_integrator = None
                        nip_in_integrator = None
                        nip_out_integrator = None
                
                self._semi_rotary_rt = {
                    "active": True,
                    "die_pos": die_pos,
                    "shuttle_pos": shuttle_pos,
                    "nip_in_pos": params.get("nip_in_pos"),
                    "nip_out_pos": params.get("nip_out_pos"),
                    "die_start": int(params["die_start"]),
                    "shuttle_center": int(params["shuttle_center"]),
                    "nip_in_start": int(params.get("nip_in_start") or 0),
                    "nip_out_start": int(params.get("nip_out_start") or 0),
                    "die_counts_per_rev": float(params["die_counts_per_rev"]),
                    "nip_in_counts_per_rev": float(params.get("nip_in_counts_per_rev") or 0.0),
                    "nip_out_counts_per_rev": float(params.get("nip_out_counts_per_rev") or 0.0),
                    "comp_counts": [int(v) for v in comp_counts],
                    "n_samples": n_samples,
                    "blend_cycles": int(params.get("blend_cycles") or 0),
                    "cycle_count": 0,
                    "max_shuttle_delta_per_cycle": params.get("max_shuttle_delta_per_cycle"),
                    "max_shuttle_excursion": params.get("max_shuttle_excursion"),
                    "enable_nips": bool(params.get("enable_nips", False)),
                    "error": None,
                    "die_phase_rt": None,
                    "comp_target_rt": None,
                    "comp_target_delta_rt": None,
                    "rev_count_rt": None,
                    "blend_rt": None,
                    "comp_raw_rt": None,
                    "ruckig_integrator": ruckig_integrator,
                    "use_ruckig": use_ruckig and ruckig_integrator is not None,
                    "nip_in_integrator": nip_in_integrator,
                    "nip_out_integrator": nip_out_integrator,
                    "nip_in_target_velocity": nip_in_target_velocity if nip_in_integrator is not None else 0.0,
                    "nip_out_target_velocity": nip_out_target_velocity if nip_out_integrator is not None else 0.0,
                    "nip_in_actual": int(params.get("nip_in_start") or 0),
                    "nip_out_actual": int(params.get("nip_out_start") or 0),
                    "active": True,
                }
                self.last_mode_cmd[die_pos] = MODE_CSP
                self.last_mode_cmd[shuttle_pos] = MODE_CSP
                shuttle_center = int(params["shuttle_center"])
                self._csp_target_next[shuttle_pos] = shuttle_center
                self._csp_target_cur[shuttle_pos] = shuttle_center
                if self._ruckig_planner:
                    self._ruckig_planner.stop(shuttle_pos)
                self._ruckig_last_error[die_pos] = None
                self._ruckig_last_error[shuttle_pos] = None
            except Exception as e:
                self._semi_rotary_rt["active"] = False
                self._semi_rotary_rt["error"] = str(e)
        elif cmd.type == CommandType.UPDATE_SEMI_ROTARY_RT:
            params = dict(cmd.params or {})
            if self._semi_rotary_rt.get("active"):
                if "comp_counts" in params and params["comp_counts"]:
                    self._semi_rotary_rt["comp_counts"] = [int(v) for v in params["comp_counts"]]
                    self._semi_rotary_rt["n_samples"] = int(params.get("n_samples") or len(self._semi_rotary_rt["comp_counts"]))
                if "die_counts_per_rev" in params:
                    self._semi_rotary_rt["target_counts_per_rev"] = float(params["die_counts_per_rev"])
                if "die_velocity_counts_per_s" in params:
                    self._semi_rotary_rt["target_velocity_counts_per_s"] = float(params["die_velocity_counts_per_s"])
                if "phase_offset" in params:
                    self._semi_rotary_rt["phase_offset"] = float(params["phase_offset"])
                for key in (
                    "blend_cycles",
                    "max_shuttle_delta_per_cycle",
                    "max_shuttle_excursion",
                    "enable_nips",
                    "nip_in_counts_per_rev",
                    "nip_out_counts_per_rev",
                    "velocity_ramp_rate",
                ):
                    if key in params:
                        self._semi_rotary_rt[key] = params[key]
        elif cmd.type == CommandType.STOP_SEMI_ROTARY_RT:
            self._semi_rotary_rt["active"] = False
        elif cmd.type == CommandType.START_DIE_VELOCITY_TEST:
            params = dict(cmd.params or {})
            try:
                die_pos = int(params["die_pos"])
                target_rpm = float(params.get("target_rpm", 5.0))
                counts_per_rev = float(params["die_counts_per_rev"])
                gear_ratio = float(params.get("die_gear_ratio", 1.0))
                direction = float(params.get("die_direction", 1.0))
                max_velocity = float(params["max_velocity"])
                max_acceleration = float(params["max_acceleration"])
                max_jerk = float(params["max_jerk"])
                dt_s = float(self.cfg.cycle_time_ms) / 1000.0
                
                target_velocity = direction * (target_rpm / 60.0) * counts_per_rev * gear_ratio
                
                if not RUCKIG_AVAILABLE_CORE:
                    logger.error("Ruckig not available - cannot start die velocity test")
                    return
                
                entries = self.offsets.get(die_pos, {})
                if (POSITION_ACTUAL_INDEX, 0) not in entries:
                    logger.error(f"Die velocity test requires 0x6064 (position actual) mapped in PDO for slave {die_pos}")
                    return
                if (VELOCITY_ACTUAL_INDEX, 0) not in entries:
                    logger.error(f"Die velocity test requires 0x606C (velocity actual) mapped in PDO for slave {die_pos}")
                    return
                
                raw_p = self.master.read_domain(self.domain, entries[(POSITION_ACTUAL_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                raw_v = self.master.read_domain(self.domain, entries[(VELOCITY_ACTUAL_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                current_pos = int.from_bytes(raw_p, "little", signed=True)
                current_vel = float(int.from_bytes(raw_v, "little", signed=True))
                
                self._die_velocity_integrator = RuckigVelocityIntegrator(
                    dt_s, max_velocity, max_acceleration, max_jerk
                )
                self._die_velocity_integrator.reset(
                    position=float(current_pos),
                    velocity=float(current_vel),
                    acceleration=0.0
                )
                
                self._die_test_target_rpm = target_rpm
                self._die_test_target_velocity = target_velocity
                self._die_test_pos = die_pos
                self._die_velocity_test_active = True
                
                self.last_mode_cmd[die_pos] = MODE_CSP
                
                logger.info(
                    f"Started die velocity test: pos={die_pos} rpm={target_rpm:.2f} "
                    f"target_vel={target_velocity:.1f} counts/s current_pos={current_pos}"
                )
            except Exception as e:
                logger.error(f"Failed to start die velocity test: {e}")
                self._die_velocity_test_active = False
        elif cmd.type == CommandType.STOP_DIE_VELOCITY_TEST:
            if self._die_velocity_test_active:
                logger.info("Stopping die velocity test")
                self._die_velocity_test_active = False
                self._die_velocity_integrator = None
        elif cmd.type == CommandType.DISABLE_PROBE:
            # Clear probe function on device (will be applied once in cyclic)
            self.last_probe_arm[cmd.target_id] = 0
        elif cmd.type == CommandType.ENABLE_DRIVE:
            self._manual_disable[cmd.target_id] = False
            self._enable_requested[cmd.target_id] = True
            self._emit_process_log(
                f"[EC][CIA402] enable requested slave={cmd.target_id} "
                f"manual_disable={self._manual_disable.get(cmd.target_id, False)} "
                f"enable_requested={self._enable_requested.get(cmd.target_id, False)} "
                f"in_op={self.slave_in_op.get(cmd.target_id, False)}"
            )
        elif cmd.type == CommandType.DISABLE_DRIVE:
            self._manual_disable[cmd.target_id] = True
            self._enable_requested[cmd.target_id] = False
            self._emit_process_log(
                f"[EC][CIA402] disable requested slave={cmd.target_id} "
                f"manual_disable={self._manual_disable.get(cmd.target_id, False)} "
                f"enable_requested={self._enable_requested.get(cmd.target_id, False)}"
            )
            # Force the state machine back toward disabled
            self.desired_controlword[cmd.target_id] = 0x0000  # Disable voltage
            self.drive_enabled[cmd.target_id] = False
            # Clear any in-flight pulses (must re-strobe after re-enable)
            self._pp_pulse_active[cmd.target_id] = False
            self._pp_pulse_start_ns[cmd.target_id] = None
            self._pp_pulse_pending[cmd.target_id] = False
            self._pv_pulse_active[cmd.target_id] = False
            self._pv_pulse_start_ns[cmd.target_id] = None
            self._pv_pulse_pending[cmd.target_id] = False
        elif cmd.type == CommandType.WRITE_RAW_PDO:
            pos = cmd.target_id
            key = (cmd.params["index"], cmd.params["subindex"])
            self._raw_pdo_writes.setdefault(pos, {})[key] = cmd.value
        elif cmd.type == CommandType.READ_SDO:
            pass
        elif cmd.type == CommandType.WRITE_SDO:
            index = cmd.params.get("index")
            subindex = cmd.params.get("subindex", 0)
            try:
                if index is None:
                    raise ValueError("missing index")
                if not self._sdo_mgr.has(int(cmd.target_id), int(index), int(subindex)):
                    raise ValueError("no SDO request object registered for this index/subindex")
                self._sdo_mgr.set_desired_write(int(cmd.target_id), int(index), int(subindex), bytes(cmd.value))
            except Exception as e:
                logger.error(f"SDO write schedule failed: slave={cmd.target_id} 0x{int(index or 0):04X}:{int(subindex)}: {e}")

    def _auto_enable_drives(self):
        """
        Automatic CiA 402 state machine - enables drives through state transitions.
        Called every cycle to monitor and transition drives to Operation Enabled.
        Only runs AFTER slave has reached OP state AND DC settling delay has elapsed.
        """
        if not self._dc_settling_complete:
            return

        period_ns = int(max(self.cfg.enable_transition_period_ms, 1.0) * 1_000_000)
        now_ns = time.monotonic_ns()

        for slave_pos, entries in self.offsets.items():
            if slave_pos not in self._cia402_positions:
                continue
            if not self.slave_in_op.get(slave_pos, False):
                continue
            if self._manual_disable.get(slave_pos, False) or not self._enable_requested.get(slave_pos, True):
                # Keep drive disabled and avoid advancing state machine.
                self.desired_controlword[slave_pos] = 0x0000  # Disable voltage
                self.drive_enabled[slave_pos] = False
                continue
            # Time-based rate limiting (defaults to 100ms)
            last_ns = self.enable_last_action_ns.get(slave_pos, 0)
            if (now_ns - last_ns) < period_ns:
                continue
            
            # Read statusword
            if (SW_INDEX, 0) not in entries:
                continue
            
            raw_sw = self.master.read_domain(self.domain, entries[(SW_INDEX, 0)], 2)
            if not raw_sw:
                continue
            
            statusword = int.from_bytes(raw_sw, 'little')
            if statusword == 0:
                continue
            
            # Decode CiA 402 state from statusword
            # Check for FAULT first (bit 3 set)
            if statusword & 0x0008:
                error_code, error_code_source = self._read_fault_error_code(slave_pos, entries)
                was_fault = bool(self._fault_active_last.get(slave_pos, False))
                prev_code = self._fault_error_code_last.get(slave_pos)
                if (not was_fault) or (prev_code != error_code):
                    if error_code is None:
                        self._emit_process_log(
                            f"Slave {slave_pos}: FAULT (statusword=0x{statusword:04X}) reason=error_code_unavailable source={error_code_source}"
                        )
                    else:
                        self._emit_process_log(
                            f"Slave {slave_pos}: FAULT (statusword=0x{statusword:04X}, error_code=0x{error_code:04X}, reason={self._fault_code_label(error_code)}, source={error_code_source})"
                        )
                self._fault_active_last[slave_pos] = True
                self._fault_error_code_last[slave_pos] = error_code
                # FAULT state - send fault reset
                step = self.enable_step.get(slave_pos, 0)
                if step < 10:  # Limit fault reset attempts
                    # Set FAULT_RESET controlword
                    self.desired_controlword[slave_pos] = 0x0080
                    self.enable_step[slave_pos] = step + 1
                    self.enable_last_action_ns[slave_pos] = now_ns
                    self._emit_process_log(
                        f"[EC][CIA402] slave={slave_pos} sw=0x{statusword:04X} "
                        f"action=FAULT_RESET cw=0x0080 attempt={step + 1}"
                    )
                    if step == 0:
                        logger.info(f"Slave {slave_pos}: sending FAULT_RESET")
                continue  # Move to next slave
            else:
                self._fault_active_last[slave_pos] = False
                # Stop any periodic fault-code SDO reads once out of FAULT.
                try:
                    self._sdo_mgr.clear_desired_read(slave_pos, ERROR_CODE_INDEX, 0)
                except Exception:
                    pass
            
            # Determine current state using CiA 402 standard bit patterns
            state_bits = statusword & 0x006F
            
            # Operation Enabled: xxxx xxxx x01x 0111
            if state_bits == 0x0027:
                # Already enabled - set controlword to maintain enabled state
                self.desired_controlword[slave_pos] = 0x000F  # ENABLE_OPERATION
                if not self.drive_enabled.get(slave_pos, False):
                    self.drive_enabled[slave_pos] = True
                    logger.info(f"✓ Slave {slave_pos}: ENABLED (0x{statusword:04X})")
                    self._emit_process_log(
                        f"[EC][CIA402] slave={slave_pos} sw=0x{statusword:04X} "
                        "state=OPERATION_ENABLED enabled=true"
                    )
                    self.enable_step[slave_pos] = 0
                self.enable_last_action_ns[slave_pos] = now_ns
                continue  # Move to next slave
            
            # Switched On: xxxx xxxx x01x 0011
            elif state_bits == 0x0023:
                # Send ENABLE_OPERATION (0x000F)
                self.desired_controlword[slave_pos] = 0x000F
                self.enable_last_action_ns[slave_pos] = now_ns
                self._emit_process_log(
                    f"[EC][CIA402] slave={slave_pos} sw=0x{statusword:04X} "
                    "state=SWITCHED_ON action=ENABLE_OPERATION cw=0x000F"
                )
                logger.info(f"Slave {slave_pos}: SWITCHED_ON (0x{statusword:04X}) → ENABLE_OPERATION")
            
            # Ready to Switch On: xxxx xxxx x00x 0001
            elif state_bits == 0x0021:
                # Send SWITCH_ON (0x0007)
                self.desired_controlword[slave_pos] = 0x0007
                self.enable_last_action_ns[slave_pos] = now_ns
                self._emit_process_log(
                    f"[EC][CIA402] slave={slave_pos} sw=0x{statusword:04X} "
                    "state=READY_TO_SWITCH_ON action=SWITCH_ON cw=0x0007"
                )
                logger.info(f"Slave {slave_pos}: READY_TO_SWITCH_ON (0x{statusword:04X}) → SWITCH_ON")
            
            # Switch On Disabled: xxxx xxxx x1xx 0000
            elif (statusword & 0x004F) == 0x0040:
                # Send SHUTDOWN (0x0006)
                self.desired_controlword[slave_pos] = 0x0006
                self.enable_last_action_ns[slave_pos] = now_ns
                self._emit_process_log(
                    f"[EC][CIA402] slave={slave_pos} sw=0x{statusword:04X} "
                    "state=SWITCH_ON_DISABLED action=SHUTDOWN cw=0x0006"
                )
                logger.info(f"Slave {slave_pos}: SWITCH_ON_DISABLED (0x{statusword:04X}) → SHUTDOWN")
            else:
                # Unknown state - log for debugging
                if self.cycle_count % 200 == 0:  # Log occasionally
                    logger.warning(f"Slave {slave_pos}: Unknown state - statusword 0x{statusword:04X}, state_bits 0x{state_bits:04X}")
    
    def _publish_status(self):
        status = NetworkStatus(drives={})
        status.timestamp_ns = int(time.time() * 1_000_000_000)
        status.cycle_time_ms_config = self.cfg.cycle_time_ms
        status.last_cycle_time_ns = self._last_cycle_time_ns
        status.last_cycle_time_us = self._last_cycle_time_us
        status.last_cycle_jitter_ns = self._last_cycle_jitter_ns
        status.last_cycle_jitter_us = self._last_cycle_jitter_us
        status.max_abs_cycle_jitter_ns = self._max_abs_cycle_jitter_ns
        status.max_abs_cycle_jitter_us = self._max_abs_cycle_jitter_us
        status.max_abs_cycle_jitter_post_warmup_ns = self._max_abs_cycle_jitter_post_warmup_ns
        status.max_abs_cycle_jitter_post_warmup_us = self._max_abs_cycle_jitter_post_warmup_us
        status.jitter_p95_ns = self._percentile_value(self._jitter_sorted_ns, 95.0)
        status.jitter_p99_ns = self._percentile_value(self._jitter_sorted_ns, 99.0)
        status.jitter_p999_ns = self._percentile_value(self._jitter_sorted_ns, 99.9)
        status.jitter_p95_us = self._percentile_value(self._jitter_sorted_us, 95.0)
        status.jitter_p99_us = self._percentile_value(self._jitter_sorted_us, 99.0)
        status.jitter_p999_us = self._percentile_value(self._jitter_sorted_us, 99.9)
        status.deadline_miss_count = self._deadline_miss_count
        status.overrun_count = self._overrun_count
        status.max_overrun_ns = self._max_overrun_ns
        status.last_work_ns = self._last_work_ns
        status.max_work_ns = self._max_work_ns
        status.work_p99_ns = self._percentile_value(self._work_sorted_ns, 99.0)
        status.last_send_interval_ns = self._last_send_interval_ns
        status.min_sleep_budget_ns = self._min_sleep_budget_ns
        status.domain_wc = self._last_domain_wc
        status.domain_wc_state = self._last_domain_wc_state
        status.domain_wc_min = self._domain_wc_min
        status.domain_wc_max = self._domain_wc_max
        status.dc_sync_error_ns = self._last_dc_sync_error_ns
        status.dc_sync_error_p95_ns = self._percentile_value(self._dc_sync_sorted_ns, 95.0)
        status.dc_sync_error_p99_ns = self._percentile_value(self._dc_sync_sorted_ns, 99.0)
        status.dc_sync_error_p999_ns = self._percentile_value(self._dc_sync_sorted_ns, 99.9)
        status.dc_sync_error_max_ns = self._max_abs_dc_sync_error_ns
        status.dc_sync_samples = len(self._dc_sync_sorted_ns)
        status.motion_command_block_count = self._motion_command_block_count
        status.all_slaves_op_first_ns = self._all_op_first_ns
        status.all_slaves_op_last_ns = self._all_op_last_ns
        status.all_slaves_left_op_last_ns = self._all_left_op_last_ns
        status.sdo_only = self.cfg.sdo_only
        
        # === NON-RT MONITORING: Working Counter Drops ===
        wc = self._last_domain_wc
        if wc is not None and self._wc_expected > 0:
            if wc < self._wc_expected:
                self._domain_wc_drop_count += 1
                
                # Alert every 500 cycles (~1 second at 2ms)
                if (self.cycle_count - self._last_wc_alert_cycle) >= 500:
                    missing = self._wc_expected - wc
                    slaves_missing = [p for p, in_op in self.slave_in_op.items() if not in_op]
                    
                    logger.warning(
                        f"[EC] WC {wc}/{self._wc_expected} (missing {missing} slaves). "
                        f"Not in OP: {slaves_missing}. Check dmesg for 'Synchronization error'."
                    )
                    self._last_wc_alert_cycle = self.cycle_count
        
        status.domain_wc_drop_count = self._domain_wc_drop_count
        status.domain_wc_expected = self._wc_expected
        status.slaves_not_in_op = [p for p, in_op in self.slave_in_op.items() if not in_op]
        
        # === NON-RT MONITORING: RT Budget Usage ===
        work_ns = self._last_work_ns
        if work_ns is not None:
            cycle_budget_ns = int(self.cfg.cycle_time_ms * 1_000_000)
            usage_percent = (work_ns / cycle_budget_ns) * 100.0
            
            # Alert if using >85% of budget
            if usage_percent > 85.0:
                self._rt_budget_warnings += 1
                
                if (self.cycle_count - self._last_rt_alert_cycle) >= 1000:
                    logger.warning(
                        f"[EC] RT budget {usage_percent:.1f}% "
                        f"(work={work_ns/1000:.1f}us / budget={cycle_budget_ns/1000:.1f}us). "
                        f"Datagrams may be skipped!"
                    )
                    self._last_rt_alert_cycle = self.cycle_count
            
            status.rt_budget_usage_percent = usage_percent
            status.rt_budget_warnings = self._rt_budget_warnings
        
        # Add detailed phase timing to status
        status.phase_recv_ns = self._phase_recv
        status.phase_state_ns = self._phase_state
        status.phase_cia_ns = self._phase_cia
        status.phase_ruckig_ns = self._phase_ruckig
        status.phase_write_ns = self._phase_write
        status.phase_dc_send_ns = self._phase_dc_send
        
        for slave_pos, entries in self.offsets.items():
            drive = {}
            drive['in_op'] = self.slave_in_op.get(slave_pos, False)
            sc = self.slave_handles.get(slave_pos)
            if sc is not None:
                sc_state = sc.get_state()
                if sc_state:
                    drive['al_state'] = sc_state.get('al_state', 0)
                    drive['online'] = sc_state.get('online', False)
                    drive['operational'] = sc_state.get('operational', False)

            if slave_pos not in self._cia402_positions:
                dcfg = next((d for d in self.cfg.slaves if d.position == slave_pos), None)
                sizes = getattr(dcfg, 'pdo_read_sizes', {}) if dcfg else {}
                raw_data = {}
                for key, offset in entries.items():
                    sz = sizes.get(key, 1)
                    raw = self.master.read_domain(self.domain, offset, sz) or bytes(sz)
                    raw_data[key] = int.from_bytes(raw, 'little')
                drive['raw_pdo'] = raw_data
                status.drives[slave_pos] = drive
                continue

            if (SW_INDEX, 0) in entries:
                raw = self.master.read_domain(self.domain, entries[(SW_INDEX, 0)], 2) or b"\x00\x00"
                sw = int.from_bytes(raw, 'little')
                drive['statusword'] = sw
                if slave_pos == 3 and self.cycle_count % 500 == 0:
                    sys.stdout.write(f"[SHM-DEBUG] Slave 3 cycle {self.cycle_count}: statusword offset={entries[(SW_INDEX, 0)]} raw={raw.hex()} value=0x{sw:04X}\n")
                    sys.stdout.flush()
                # Derived semantics (no extra bus access)
                state_bits = sw & 0x006F
                drive['enabled'] = (state_bits == 0x0027) and not self._manual_disable.get(slave_pos, False)
                drive['fault'] = bool(sw & 0x0008)
                drive['warning'] = bool(sw & 0x0080)
                drive['target_reached'] = bool(sw & 0x0400)
                drive['internal_limit_active'] = bool(sw & 0x0800)
                drive['setpoint_ack'] = bool(sw & 0x1000)
                # Intent flags
                drive['enable_requested'] = self._enable_requested.get(slave_pos, True)
                drive['manual_disable'] = self._manual_disable.get(slave_pos, False)
                drive['op_entered_first_ns'] = self._op_entered_first_ns.get(slave_pos)
                drive['op_entered_last_ns'] = self._op_entered_last_ns.get(slave_pos)
                drive['op_left_last_ns'] = self._op_left_last_ns.get(slave_pos)
                drive['op_dropout_count'] = self._op_dropout_count.get(slave_pos, 0)
            if (MODES_OP_DISPLAY_INDEX, 0) in entries:
                raw = self.master.read_domain(self.domain, entries[(MODES_OP_DISPLAY_INDEX, 0)], 1) or b"\x00"
                drive['mode_display'] = raw[0]
            if (POSITION_ACTUAL_INDEX, 0) in entries:
                raw = self.master.read_domain(self.domain, entries[(POSITION_ACTUAL_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                drive['position_actual'] = int.from_bytes(raw, 'little', signed=True)
            if (VELOCITY_ACTUAL_INDEX, 0) in entries:
                raw = self.master.read_domain(self.domain, entries[(VELOCITY_ACTUAL_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                drive['velocity_actual'] = int.from_bytes(raw, 'little', signed=True)
            if (TORQUE_ACTUAL_INDEX, 0) in entries:
                raw = self.master.read_domain(self.domain, entries[(TORQUE_ACTUAL_INDEX, 0)], 2) or b"\x00\x00"
                drive['torque_actual'] = int.from_bytes(raw, 'little', signed=True)

            # Probe status
            try:
                if (PROBE_STATUS_INDEX, 0) in entries:
                    raw = self.master.read_domain(self.domain, entries[(PROBE_STATUS_INDEX, 0)], 2) or b"\x00\x00"
                    drive['probe_active'] = bool(int.from_bytes(raw, 'little') & 0x0001)
                    drive['probe_enabled'] = True
                if (PROBE_POS1_INDEX, 0) in entries:
                    raw = self.master.read_domain(self.domain, entries[(PROBE_POS1_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                    drive['probe_pos1'] = int.from_bytes(raw, 'little', signed=True)
                if (PROBE_POS2_INDEX, 0) in entries:
                    raw = self.master.read_domain(self.domain, entries[(PROBE_POS2_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                    drive['probe_pos2'] = int.from_bytes(raw, 'little', signed=True)
                elif (PROBE_POS2_ALT_INDEX, 0) in entries:
                    raw = self.master.read_domain(self.domain, entries[(PROBE_POS2_ALT_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                    drive['probe_pos2'] = int.from_bytes(raw, 'little', signed=True)
            except Exception as e:
                if slave_pos == 3:
                    sys.stdout.write(f"[ERROR] Slave 3 probe read exception: {e}\n")
                    sys.stdout.flush()
            if (DIGITAL_INPUTS_INDEX, 0) in entries:
                raw = self.master.read_domain(self.domain, entries[(DIGITAL_INPUTS_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                drive['digital_inputs'] = int.from_bytes(raw, 'little')
            if (DIP_IN_STATE_INDEX, 1) in entries:
                raw = self.master.read_domain(self.domain, entries[(DIP_IN_STATE_INDEX, 1)], 4) or b"\x00\x00\x00\x00"
                drive['dip_in_state'] = int.from_bytes(raw, 'little')
            elif slave_pos == 3 and self.cycle_count < 5:
                sys.stdout.write(f"[PROC-DEBUG] Slave 3 cycle {self.cycle_count}: DIP_IN_STATE_INDEX={(DIP_IN_STATE_INDEX, 1)} NOT in entries!\n")
                sys.stdout.write(f"[PROC-DEBUG] entries keys: {list(entries.keys())}\n")
                sys.stdout.flush()
            if (ERROR_CODE_INDEX, 0) in entries:
                raw = self.master.read_domain(self.domain, entries[(ERROR_CODE_INDEX, 0)], 2) or b"\x00\x00"
                drive['error_code'] = int.from_bytes(raw, 'little')
            # Publish static feature capabilities (from XML-derived supports)
            drive['features'] = self.features.get(slave_pos, {})
            # Publish Ruckig planner state
            if self._ruckig_planner is not None:
                r = self._ruckig_planner.describe(slave_pos)
                r["last_error"] = self._ruckig_last_error.get(slave_pos)
                drive["ruckig"] = r
                rt = self._ruckig_trace.get(slave_pos)
                if rt is not None:
                    drive["ruckig_trace"] = dict(rt)
            semi_rt = self._semi_rotary_rt
            if slave_pos == semi_rt.get("shuttle_pos") or slave_pos == semi_rt.get("die_pos"):
                drive["semi_rotary_rt"] = {
                    "active": bool(semi_rt.get("active")),
                    "error": semi_rt.get("error"),
                    "die_phase_rt": semi_rt.get("die_phase_rt"),
                    "comp_target_rt": semi_rt.get("comp_target_rt"),
                    "comp_target_delta_rt": semi_rt.get("comp_target_delta_rt"),
                    "rev_count_rt": semi_rt.get("rev_count_rt"),
                    "blend_rt": semi_rt.get("blend_rt"),
                    "comp_raw_rt": semi_rt.get("comp_raw_rt"),
                    "cycle_count_rt": semi_rt.get("cycle_count"),
                }
            # Publish PDO map health (presence)
            pdo_health = {}
            for key_idx in [MODES_OP_INDEX, MODES_OP_DISPLAY_INDEX, CW_INDEX, SW_INDEX, TARGET_VELOCITY_INDEX, TARGET_POSITION_INDEX]:
                state = 'missing'
                if (key_idx, 0) in entries:
                    state = 'pdo'
                pdo_health[f"0x{key_idx:04X}:0"] = state
            drive['pdo_health'] = pdo_health
            if slave_pos == 3 and self.cycle_count in [15000, 15001, 15002]:
                sys.stdout.write(f"[PROC-DEBUG2] Slave 3 cycle {self.cycle_count}: About to publish drive dict keys: {list(drive.keys())}\n")
                if 'dip_in_state' in drive:
                    sys.stdout.write(f"[PROC-DEBUG2] Slave 3: dip_in_state=0x{drive['dip_in_state']:08X}\n")
                else:
                    sys.stdout.write(f"[PROC-DEBUG2] Slave 3: dip_in_state NOT in drive dict!\n")
                sys.stdout.flush()
            status.drives[slave_pos] = drive
        try:
            self.status_q.put_nowait(status)
            if bool(getattr(self.cfg, "process_status_debug_log", True)) and self._status_publish_count < 3:
                self._status_publish_count += 1
                self._emit_process_log(
                    f"[EC] status published cycle={self.cycle_count} drives={list(status.drives.keys())} qsize={self.status_q.qsize()}"
                )
        except queue.Full:
            pass
        except Exception as e:
            self._emit_process_log(f"[EC] status_q.put FAILED: {e}", stderr=True)
            traceback.print_exc(file=sys.stderr)
            sys.stderr.flush()

    def _cyclic_write(self):
        """
        Apply desired intents to PDO or SDO each cycle. Maintains mode (0x6060),
        controlword (0x6040), targets (0x607A, 0x60FF), and device features like
        probe function (0x60B8) if mapped.
        """
        if self.cfg.sdo_only or self.domain is None:
            return

        for pos, writes in self._raw_pdo_writes.items():
            entries = self.offsets.get(pos, {})
            for key, data in writes.items():
                if key in entries:
                    self.master.write_domain(self.domain, entries[key], data)

        for slave_pos, entries in self.offsets.items():
            if slave_pos not in self._cia402_positions:
                continue
            statusword = None
            if (SW_INDEX, 0) in entries:
                raw_sw = self.master.read_domain(self.domain, entries[(SW_INDEX, 0)], 2)
                if raw_sw:
                    statusword = int.from_bytes(raw_sw, 'little')

            c = self._pdo_cache.setdefault(slave_pos, {})
            c['sw'] = statusword or 0

            mode = self.last_mode_cmd.get(slave_pos)
            if mode is not None:
                if (MODES_OP_INDEX, 0) in entries:
                    self.master.write_domain(self.domain, entries[(MODES_OP_INDEX, 0)], bytes([mode]))
                else:
                    prev = self._last_mode_sdo.get(slave_pos)
                    if mode != prev:
                        logger.warning(
                            f"Slave {slave_pos}: {hex(MODES_OP_INDEX)} not in PDO; scheduling mode={mode} via SDO request object"
                        )
                        self._last_mode_sdo[slave_pos] = mode
                        if self._sdo_mgr.has(slave_pos, MODES_OP_INDEX, 0):
                            self._sdo_mgr.set_desired_write(slave_pos, MODES_OP_INDEX, 0, bytes([int(mode) & 0xFF]))
                        else:
                            logger.error(f"Slave {slave_pos}: no SDO request object for {hex(MODES_OP_INDEX)}:0")

            motion_ok = self.drive_enabled.get(slave_pos, False) and not self._manual_disable.get(slave_pos, False)
            mode_eff = mode if mode is not None else self._default_mode.get(slave_pos)

            if motion_ok:
                if mode_eff == MODE_PT:
                    v = 0
                else:
                    vel = self.last_velocity_cmd.get(slave_pos)
                    v = int(vel) if vel is not None else None
                if v is not None:
                    if (TARGET_VELOCITY_INDEX, 0) in entries:
                        self.master.write_domain(self.domain, entries[(TARGET_VELOCITY_INDEX, 0)], v.to_bytes(4, byteorder='little', signed=True))
                    else:
                        prev = self._last_velocity_sdo.get(slave_pos)
                        if v != prev:
                            logger.info(
                                f"Slave {slave_pos}: {hex(TARGET_VELOCITY_INDEX)} not in PDO; scheduling vel={v} via SDO request object"
                            )
                            self._last_velocity_sdo[slave_pos] = v
                            if self._sdo_mgr.has(slave_pos, TARGET_VELOCITY_INDEX, 0):
                                self._sdo_mgr.set_desired_write(
                                    slave_pos,
                                    TARGET_VELOCITY_INDEX,
                                    0,
                                    v.to_bytes(4, 'little', signed=True),
                                )
                            else:
                                logger.error(f"Slave {slave_pos}: no SDO request object for {hex(TARGET_VELOCITY_INDEX)}:0")
                    if mode_eff != MODE_PT and self._pv_pulse_pending.get(slave_pos, False) and self._pv_pulse_active.get(slave_pos, False):
                        self._pv_pulse_pending[slave_pos] = False

            torque = self.last_torque_cmd.get(slave_pos)
            if motion_ok and torque is not None:
                t = int(torque)
                if (TARGET_TORQUE_INDEX, 0) in entries:
                    self.master.write_domain(self.domain, entries[(TARGET_TORQUE_INDEX, 0)], t.to_bytes(2, byteorder='little', signed=True))
                else:
                    prev = self._last_torque_sdo.get(slave_pos)
                    if t != prev:
                        logger.info(
                            f"Slave {slave_pos}: {hex(TARGET_TORQUE_INDEX)} not in PDO; scheduling torque={t} via SDO request object"
                        )
                        self._last_torque_sdo[slave_pos] = t
                        if self._sdo_mgr.has(slave_pos, TARGET_TORQUE_INDEX, 0):
                            self._sdo_mgr.set_desired_write(
                                slave_pos,
                                TARGET_TORQUE_INDEX,
                                0,
                                t.to_bytes(2, 'little', signed=True),
                            )
                        else:
                            logger.error(f"Slave {slave_pos}: no SDO request object for {hex(TARGET_TORQUE_INDEX)}:0")

            # Maintain target position (0x607A)
            # - PP/HM: write the last commanded target (latch with bit4 pulse below)
            # - CSP: stream the buffered position every cycle (atomic swap)
            pos = None
            mode_eff = mode if mode is not None else self._default_mode.get(slave_pos)
            if mode_eff == MODE_CSP:
                if slave_pos in self._csp_target_next:
                    self._csp_target_cur[slave_pos] = self._csp_target_next.get(slave_pos)
                pos = self._csp_target_cur.get(slave_pos)
                if pos is None and (POSITION_ACTUAL_INDEX, 0) in entries:
                    raw_p = self.master.read_domain(self.domain, entries[(POSITION_ACTUAL_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                    pos = int.from_bytes(raw_p, "little", signed=True)
            else:
                pos = self.last_position_cmd.get(slave_pos)

            if mode_eff == MODE_CSP and pos is not None and (TARGET_POSITION_INDEX, 0) in entries:
                p = _wrap_i32(int(pos))
                self.master.write_domain(self.domain, entries[(TARGET_POSITION_INDEX, 0)], p.to_bytes(4, byteorder='little', signed=True))
            elif motion_ok and pos is not None and mode_eff != MODE_PT:
                p = int(pos)
                if (TARGET_POSITION_INDEX, 0) in entries:
                    self.master.write_domain(self.domain, entries[(TARGET_POSITION_INDEX, 0)], p.to_bytes(4, byteorder='little', signed=True))
                else:
                    warned = self.warned_missing_pdo.setdefault(slave_pos, set())
                    key = (TARGET_POSITION_INDEX, 0)
                    if key not in warned:
                        logger.warning(
                            f"Slave {slave_pos}: {hex(TARGET_POSITION_INDEX)} not in PDO; scheduling via SDO request object"
                        )
                        warned.add(key)
                    prev = self._last_position_sdo.get(slave_pos)
                    if p != prev:
                        self._last_position_sdo[slave_pos] = p
                        if self._sdo_mgr.has(slave_pos, TARGET_POSITION_INDEX, 0):
                            self._sdo_mgr.set_desired_write(
                                slave_pos,
                                TARGET_POSITION_INDEX,
                                0,
                                p.to_bytes(4, "little", signed=True),
                            )
                        else:
                            logger.error(f"Slave {slave_pos}: no SDO request object for {hex(TARGET_POSITION_INDEX)}:0")

            # Controlword bit maintenance (0x6040):
            # Build controlword from desired state machine value + motion bits
            # CRITICAL: Must write EVERY cycle or slave may ignore commands
            if (CW_INDEX, 0) in entries:
                cw_offset = entries[(CW_INDEX, 0)]
                
                # Start with state machine controlword (or default to enabled if already enabled)
                if slave_pos in self.desired_controlword:
                    cw = self.desired_controlword[slave_pos]
                elif self.drive_enabled.get(slave_pos, False):
                    # Drive is enabled - use ENABLE_OPERATION as base
                    cw = CW_ENABLE_OP_SIMPLIFIED
                else:
                    # Not enabled yet and no state machine command - skip this cycle
                    cw = None
                
                # Write controlword if we have one
                if cw is not None:
                    # --- CiA402 controlword behavior by mode ---
                    # PV / Profile Torque / CSP style modes: continuous targets, no PP strobe.
                    # PP: strobe bit 4 ("new set-point") as a pulse, then clear on ack/timeout.
                    is_enabled = self.drive_enabled.get(slave_pos, False)
                    mode_eff = mode_eff

                    # Default: clear HALT so it doesn't linger across moves
                    cw &= ~(1 << CW_BIT_HALT)

                    # Update/clear PP pulse state (only after we've asserted bit 4 at least once).
                    if self._pp_pulse_active.get(slave_pos, False):
                        ack_mask = int(self.cfg.pp_ack_mask)
                        timeout_ns = int(max(self.cfg.pp_ack_timeout_ms, 1.0) * 1_000_000)
                        start_ns = self._pp_pulse_start_ns.get(slave_pos) or 0
                        now_ns = time.monotonic_ns()
                        acked = bool(statusword is not None and (statusword & ack_mask))
                        timed_out = (now_ns - start_ns) >= timeout_ns
                        if acked or timed_out:
                            # Stop asserting bit 4 from this cycle onward.
                            self._pp_pulse_active[slave_pos] = False
                            self._pp_pulse_start_ns[slave_pos] = None
                            self._pp_pulse_pending[slave_pos] = False

                    # Update/clear PV pulse state (optional; mirrors PP ack/timeout semantics).
                    if self._pv_pulse_active.get(slave_pos, False):
                        ack_mask = int(self.cfg.pp_ack_mask)
                        timeout_ns = int(max(self.cfg.pp_ack_timeout_ms, 1.0) * 1_000_000)
                        start_ns = self._pv_pulse_start_ns.get(slave_pos) or 0
                        now_ns = time.monotonic_ns()
                        acked = bool(statusword is not None and (statusword & ack_mask))
                        timed_out = (now_ns - start_ns) >= timeout_ns
                        if acked or timed_out:
                            self._pv_pulse_active[slave_pos] = False
                            self._pv_pulse_start_ns[slave_pos] = None
                            self._pv_pulse_pending[slave_pos] = False

                    # Assert bit 4 only when a pulse is pending/active and a strobe-driven mode is selected.
                    # CiA402: PP uses bit 4 "new set-point". HM commonly uses bit 4 as the homing start trigger.
                    if is_enabled and (mode_eff in (MODE_PP, MODE_HM)):
                        # Absolute/relative semantics: default to ABSOLUTE for PP (bit6=0).
                        if mode_eff == MODE_PP:
                            cw &= ~(1 << CW_BIT_ABS_REL)  # absolute
                            cw |= (1 << CW_BIT_CHANGE_IMMEDIATELY)

                        # Force-clear cycle to guarantee a fresh 0→1 edge if a new command arrived mid-pulse.
                        fc = int(self._pp_force_clear_cycles.get(slave_pos, 0))
                        if fc > 0:
                            cw &= ~(1 << CW_BIT_NEW_SET_POINT)
                            self._pp_force_clear_cycles[slave_pos] = fc - 1
                        elif self._pp_pulse_pending.get(slave_pos, False) or self._pp_pulse_active.get(slave_pos, False):
                            cw |= (1 << CW_BIT_NEW_SET_POINT)
                            # Mark that we actually asserted bit 4 (required to later clear on ack/timeout).
                            if not self._pp_pulse_active.get(slave_pos, False):
                                self._pp_pulse_active[slave_pos] = True
                                self._pp_pulse_start_ns[slave_pos] = time.monotonic_ns()
                                # Pending is a one-shot request; once we've started the pulse, rely on
                                # active+ack/timeout to clear it deterministically.
                                self._pp_pulse_pending[slave_pos] = False

                    # Optional PV set-point pulse behavior (device-specific)
                    if is_enabled and (mode_eff == MODE_PV):
                        dcfg = next((d for d in self.cfg.slaves if d.position == slave_pos), None)
                        pv_requires = bool(dcfg and getattr(dcfg, 'pv_requires_setpoint_toggle', False))
                        if pv_requires:
                            fc = int(self._pv_force_clear_cycles.get(slave_pos, 0))
                            if fc > 0:
                                cw &= ~(1 << CW_BIT_NEW_SET_POINT)
                                self._pv_force_clear_cycles[slave_pos] = fc - 1
                            elif self._pv_pulse_pending.get(slave_pos, False) or self._pv_pulse_active.get(slave_pos, False):
                                cw |= (1 << CW_BIT_NEW_SET_POINT)
                                if not self._pv_pulse_active.get(slave_pos, False):
                                    self._pv_pulse_active[slave_pos] = True
                                    self._pv_pulse_start_ns[slave_pos] = time.monotonic_ns()
                    
                    self.master.write_domain(self.domain, cw_offset, cw.to_bytes(2, byteorder='little'))

            if (PROBE_FUNCTION_INDEX, 0) in entries:
                probe_val = self.last_probe_arm.get(slave_pos)
                val = (int(probe_val) if probe_val is not None else 0).to_bytes(2, 'little')
                self.master.write_domain(self.domain, entries[(PROBE_FUNCTION_INDEX, 0)], val)
                if probe_val is not None:
                    self.last_probe_arm[slave_pos] = None
            else:
                probe_val = self.last_probe_arm.get(slave_pos)
                if probe_val is not None:
                    warned = self.warned_missing_pdo.setdefault(slave_pos, set())
                    key = (PROBE_FUNCTION_INDEX, 0)
                    if key not in warned:
                        logger.warning(
                            f"Slave {slave_pos}: {hex(PROBE_FUNCTION_INDEX)} not in PDO; scheduling via SDO request object"
                        )
                        warned.add(key)
                    if self._sdo_mgr.has(slave_pos, PROBE_FUNCTION_INDEX, 0):
                        self._sdo_mgr.set_desired_write(
                            slave_pos,
                            PROBE_FUNCTION_INDEX,
                            0,
                            int(probe_val).to_bytes(2, "little"),
                        )
                    else:
                        logger.error(f"Slave {slave_pos}: no SDO request object for {hex(PROBE_FUNCTION_INDEX)}:0")
                    self.last_probe_arm[slave_pos] = None

    def _update_die_velocity_test(self) -> None:
        """
        RT update for die velocity test mode using Ruckig velocity interface.
        Generates smooth CSP position setpoints from velocity profile.
        """
        if not self._die_velocity_test_active or self._die_velocity_integrator is None:
            return
        
        if self.cfg.sdo_only or self.domain is None:
            return
        
        die_pos = self._die_test_pos
        if die_pos is None:
            return
        
        if not self.drive_enabled.get(die_pos, False) or self._manual_disable.get(die_pos, False):
            return
        
        try:
            position, velocity, acceleration = self._die_velocity_integrator.step(
                self._die_test_target_velocity
            )
            
            self._csp_target_next[die_pos] = int(round(position))
        except Exception as e:
            logger.error(f"Die velocity test update failed: {e}")
            self._die_velocity_test_active = False
    
    def _update_semi_rotary_rt(self) -> None:
        rt = self._semi_rotary_rt
        if not rt.get("active"):
            return
        if self.cfg.sdo_only or self.domain is None:
            return

        die_pos = rt.get("die_pos")
        shuttle_pos = rt.get("shuttle_pos")
        if die_pos is None or shuttle_pos is None:
            rt["error"] = "die_pos/shuttle_pos not configured"
            rt["active"] = False
            return

        if not self.drive_enabled.get(die_pos, False) or self._manual_disable.get(die_pos, False):
            return
        if not self.drive_enabled.get(shuttle_pos, False) or self._manual_disable.get(shuttle_pos, False):
            return

        die_entries = self.offsets.get(die_pos, {})
        if (POSITION_ACTUAL_INDEX, 0) not in die_entries:
            rt["error"] = "die position_actual PDO missing"
            rt["active"] = False
            return

        raw = self.master.read_domain(self.domain, die_entries[(POSITION_ACTUAL_INDEX, 0)], 4)
        if not raw:
            return
        die_actual = int.from_bytes(raw, "little", signed=True)
        
        counts_per_rev = float(rt.get("die_counts_per_rev", 0.0))
        abs_counts_per_rev = abs(counts_per_rev)
        if abs_counts_per_rev <= 0.0:
            rt["error"] = "die_counts_per_rev must be non-zero"
            rt["active"] = False
            return
        
        cycle_count = int(rt.get("cycle_count", 0))
        die_start = int(rt.get("die_start", 0))
        
        use_ruckig = rt.get("use_ruckig", False)
        ruckig_integrator = rt.get("ruckig_integrator")
        target_velocity_counts_per_s = float(rt.get("die_velocity_counts_per_s", 0.0))
        
        if cycle_count == 0:
            rt["die_commanded_position"] = die_actual
            rt["target_counts_per_rev"] = counts_per_rev
            rt["die_velocity_counts_per_s"] = target_velocity_counts_per_s
            rt["target_velocity_counts_per_s"] = target_velocity_counts_per_s
            
            if use_ruckig and ruckig_integrator is not None:
                ruckig_integrator.reset(position=0.0, velocity=0.0, acceleration=0.0)
                print(f"[RT-INIT] Using Ruckig velocity interface, target_velocity={target_velocity_counts_per_s:.0f} counts/s", flush=True)
            else:
                rt["die_velocity_counts_per_s"] = 0.0
                print(f"[RT-INIT] Using linear ramp, target_velocity={target_velocity_counts_per_s:.0f} counts/s", flush=True)
            
            print(f"[RT-INIT] die_actual={die_actual}, die_commanded_position={die_actual}", flush=True)
        
        die_commanded = rt.get("die_commanded_position", die_actual)
        dt_s = float(self.cfg.cycle_time_ms) / 1000.0
        
        if use_ruckig and ruckig_integrator is not None:
            position, velocity, acceleration = ruckig_integrator.step(target_velocity_counts_per_s)
            die_commanded = int(die_actual + position)
            
            if cycle_count % 500 == 0:
                print(f"[RT-RUCKIG] cycle={cycle_count}, target_vel={target_velocity_counts_per_s:.1f}, "
                      f"current_vel={velocity:.1f}, acc={acceleration:.1f}, commanded={die_commanded}", flush=True)
        else:
            die_velocity_counts_per_s = rt.get("die_velocity_counts_per_s", 0.0)
            
            if abs(target_velocity_counts_per_s - die_velocity_counts_per_s) > 0.01:
                ramp_rate = float(rt.get("velocity_ramp_rate", 100000.0))
                max_change_per_cycle = ramp_rate * dt_s
                delta = target_velocity_counts_per_s - die_velocity_counts_per_s
                if abs(delta) <= max_change_per_cycle:
                    die_velocity_counts_per_s = target_velocity_counts_per_s
                else:
                    die_velocity_counts_per_s += max_change_per_cycle if delta > 0 else -max_change_per_cycle
                rt["die_velocity_counts_per_s"] = die_velocity_counts_per_s
            
            die_velocity_counts_per_cycle = die_velocity_counts_per_s * dt_s
            die_commanded = int(die_commanded + die_velocity_counts_per_cycle)
            
            if cycle_count % 500 == 0:
                print(f"[RT-LINEAR] cycle={cycle_count}, vel/s={die_velocity_counts_per_s:.1f}, "
                      f"vel/cyc={die_velocity_counts_per_cycle:.3f}, commanded={die_commanded}", flush=True)
        
        self._csp_target_next[die_pos] = int(die_commanded)
        self.last_mode_cmd[die_pos] = MODE_CSP
        rt["die_commanded_position"] = die_commanded
        
        if cycle_count % 500 == 0:
            print(f"[RT-SEND] pos={die_pos}, csp_target={int(die_commanded)}", flush=True)
            actual_die_pos = self.position_actual.get(die_pos, 0)
            error = actual_die_pos - die_commanded
            print(f"[RT-FEEDBACK] cmd={die_commanded}, actual={actual_die_pos}, error={error}", flush=True)
        
        die_elapsed = die_actual - die_start
        if abs_counts_per_rev <= 0.0:
            rt["error"] = "die_counts_per_rev must be non-zero"
            rt["active"] = False
            return

        if counts_per_rev < 0:
            die_phase = ((-die_elapsed) % abs_counts_per_rev) / abs_counts_per_rev
        else:
            die_phase = (die_elapsed % abs_counts_per_rev) / abs_counts_per_rev
        
        phase_offset = float(rt.get("phase_offset", 0.0))
        die_phase = (die_phase + phase_offset) % 1.0

        comp_counts = rt.get("comp_counts") or []
        n_samples = int(rt.get("n_samples") or len(comp_counts))
        table_len = len(comp_counts)
        if table_len <= 0 or n_samples <= 0:
            rt["error"] = "empty comp_counts"
            rt["active"] = False
            return
        if n_samples > table_len:
            n_samples = table_len
        phase_idx = die_phase * n_samples
        i0 = int(phase_idx)
        if i0 >= n_samples:
            i0 = n_samples - 1
        frac = phase_idx - float(i0)
        i1 = i0 + 1
        if i1 >= n_samples:
            i1 = 0
        v0 = float(comp_counts[i0])
        v1 = float(comp_counts[i1])
        comp_raw = int(round(v0 + (v1 - v0) * frac))

        cycle_count = int(rt.get("cycle_count", 0))
        blend_cycles = int(rt.get("blend_cycles", 0))
        blend = 1.0 if blend_cycles <= 0 else min(1.0, float(cycle_count) / float(blend_cycles))

        shuttle_center = int(rt.get("shuttle_center", 0))
        comp_target = shuttle_center + int(comp_raw * blend)

        max_excursion = rt.get("max_shuttle_excursion")
        if max_excursion is not None:
            max_exc = abs(int(max_excursion))
            lo = shuttle_center - max_exc
            hi = shuttle_center + max_exc
            if comp_target < lo:
                comp_target = lo
            elif comp_target > hi:
                comp_target = hi

        prev_target = rt.get("comp_target_rt")
        target_delta = None
        if prev_target is not None:
            target_delta = int(comp_target) - int(prev_target)
            max_delta = rt.get("max_shuttle_delta_per_cycle")
            if max_delta is not None:
                lim = abs(int(max_delta))
                if abs(target_delta) > lim:
                    comp_target = int(prev_target) + (lim if target_delta > 0 else -lim)
                    target_delta = int(comp_target) - int(prev_target)

        rev_count = die_elapsed / counts_per_rev
        self._csp_target_next[shuttle_pos] = int(comp_target)
        self.last_mode_cmd[shuttle_pos] = MODE_CSP

        if bool(rt.get("enable_nips", False)):
            nip_in_pos = rt.get("nip_in_pos")
            nip_in_integrator = rt.get("nip_in_integrator")
            nip_in_target_velocity = rt.get("nip_in_target_velocity", 0.0)
            
            if nip_in_pos is not None and self.drive_enabled.get(nip_in_pos, False) and not self._manual_disable.get(nip_in_pos, False):
                if nip_in_integrator is not None and rt.get("use_ruckig", False):
                    nip_in_position, nip_in_velocity, nip_in_acceleration = nip_in_integrator.step(nip_in_target_velocity)
                    nip_in_actual = int(rt.get("nip_in_actual", rt.get("nip_in_start", 0)))
                    nip_in_target = nip_in_actual + int(nip_in_position)
                    rt["nip_in_actual"] = int(nip_in_target)
                else:
                    nip_in_target = int(rt.get("nip_in_start", 0)) + int(rev_count * float(rt.get("nip_in_counts_per_rev", 0.0)))
                self._csp_target_next[int(nip_in_pos)] = int(nip_in_target)
                self.last_mode_cmd[int(nip_in_pos)] = MODE_CSP
            
            nip_out_pos = rt.get("nip_out_pos")
            nip_out_integrator = rt.get("nip_out_integrator")
            nip_out_target_velocity = rt.get("nip_out_target_velocity", 0.0)
            
            if nip_out_pos is not None and self.drive_enabled.get(nip_out_pos, False) and not self._manual_disable.get(nip_out_pos, False):
                if nip_out_integrator is not None and rt.get("use_ruckig", False):
                    nip_out_position, nip_out_velocity, nip_out_acceleration = nip_out_integrator.step(nip_out_target_velocity)
                    nip_out_actual = int(rt.get("nip_out_actual", rt.get("nip_out_start", 0)))
                    nip_out_target = nip_out_actual + int(nip_out_position)
                    rt["nip_out_actual"] = int(nip_out_target)
                else:
                    nip_out_target = int(rt.get("nip_out_start", 0)) + int(rev_count * float(rt.get("nip_out_counts_per_rev", 0.0)))
                self._csp_target_next[int(nip_out_pos)] = int(nip_out_target)
                self.last_mode_cmd[int(nip_out_pos)] = MODE_CSP

        rt["die_phase_rt"] = float(die_phase)
        rt["comp_target_rt"] = int(comp_target)
        rt["comp_target_delta_rt"] = int(target_delta) if target_delta is not None else None
        rt["rev_count_rt"] = float(rev_count)
        rt["blend_rt"] = float(blend)
        rt["comp_raw_rt"] = int(comp_raw)
        rt["cycle_count"] = cycle_count + 1

    def _update_ruckig(self) -> None:
        """
        If enabled and active, update Ruckig once per cycle and feed the CSP buffer (_csp_target_next).
        """
        if not self._ruckig_planner:
            return
        if self.cfg.sdo_only or self.domain is None:
            return

        dt_s_fallback = float(self.cfg.cycle_time_ms) / 1000.0

        for dcfg in self.cfg.slaves:
            pos = dcfg.position

            # If no request and planner not active, skip
            if pos not in self._ruckig_requests and not self._ruckig_planner.is_active(pos):
                continue

            # Safety: do not generate motion if not enabled
            if not self.drive_enabled.get(pos, False) or self._manual_disable.get(pos, False):
                continue

            entries = self.offsets.get(pos, {})
            if (POSITION_ACTUAL_INDEX, 0) not in entries:
                self._ruckig_last_error[pos] = "Ruckig requires 0x6064 (position actual) mapped in PDO."
                self._ruckig_requests.pop(pos, None)
                self._ruckig_planner.stop(pos)
                continue
            if (VELOCITY_ACTUAL_INDEX, 0) not in entries:
                self._ruckig_last_error[pos] = "Ruckig requires 0x606C (velocity actual) mapped in PDO."
                self._ruckig_requests.pop(pos, None)
                self._ruckig_planner.stop(pos)
                continue

            raw_p = self.master.read_domain(self.domain, entries[(POSITION_ACTUAL_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
            raw_v = self.master.read_domain(self.domain, entries[(VELOCITY_ACTUAL_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
            actual_position = int.from_bytes(raw_p, "little", signed=True)
            actual_velocity_drive = float(int.from_bytes(raw_v, "little", signed=True))
            c = self._pdo_cache.setdefault(pos, {})
            c['pos'] = actual_position
            c['vel'] = int(actual_velocity_drive)
            prev_actual = self._ruckig_prev_actual_position.get(pos)
            measured_delta = None if prev_actual is None else int(actual_position - prev_actual)
            prev_cycle = self._ruckig_prev_sample_cycle.get(pos)
            sample_is_fresh = prev_cycle is not None and (int(self.cycle_count) - int(prev_cycle) == 1)
            self._ruckig_prev_actual_position[pos] = int(actual_position)
            self._ruckig_prev_sample_cycle[pos] = int(self.cycle_count)
            actual_velocity_estimated = (
                float(measured_delta) / dt_s_fallback
                if measured_delta is not None and sample_is_fresh
                else actual_velocity_drive
            )
            prev_velocity = self._ruckig_prev_velocity.get(pos)
            velocity_delta = None if prev_velocity is None else (actual_velocity_estimated - prev_velocity)
            self._ruckig_prev_velocity[pos] = actual_velocity_estimated
            actual_acceleration_estimated = (
                float(velocity_delta) / dt_s_fallback
                if velocity_delta is not None and sample_is_fresh
                else 0.0
            )
            trace = self._ruckig_trace.setdefault(pos, {})
            trace["measured_position_actual"] = int(actual_position)
            trace["measured_velocity_actual"] = float(actual_velocity_drive)
            trace["measured_velocity_estimated"] = float(actual_velocity_estimated)
            trace["measured_acceleration_estimated"] = float(actual_acceleration_estimated)
            trace["measured_velocity_estimate_fresh"] = bool(sample_is_fresh)
            trace["measured_position_delta"] = measured_delta
            trace["cycle_time_s"] = dt_s_fallback
            trace["drive_max_velocity_config"] = getattr(dcfg, "max_velocity", None)
            trace["cycle_idx"] = int(self.cycle_count)

            # Start request (initialize planner using measured actuals)
            req = self._ruckig_requests.get(pos)
            if req is not None:
                cfg = getattr(dcfg, "ruckig", None)
                if not cfg or not getattr(cfg, "enabled", False):
                    self._ruckig_last_error[pos] = "Ruckig request rejected: DriveConfig.ruckig.enabled is False."
                    self._ruckig_requests.pop(pos, None)
                    self._ruckig_planner.stop(pos)
                    continue

                params = req.get("params") or {}
                unit = (params.get("unit") or "native").lower()
                if unit != "native":
                    self._ruckig_last_error[pos] = "Ruckig currently supports only unit='native' (drive pulses)."
                    self._ruckig_requests.pop(pos, None)
                    self._ruckig_planner.stop(pos)
                    continue

                overrides = {
                    "max_velocity": params.get("max_velocity"),
                    "max_acceleration": params.get("max_acceleration"),
                    "max_jerk": params.get("max_jerk"),
                }

                velocity_for_init = actual_velocity_estimated
                velocity_drive_for_init = actual_velocity_drive
                acceleration_for_init = actual_acceleration_estimated

                try:
                    if req.get("kind") == "position":
                        trace["request_kind"] = "position"
                        trace["request_target_position"] = int(req.get("target"))
                        self._ruckig_planner.start_position(
                            pos,
                            actual_position=actual_position,
                            actual_velocity=velocity_drive_for_init,
                            target_position=int(req.get("target")),
                            cfg=cfg,
                            dt_s_fallback=dt_s_fallback,
                            overrides=overrides,
                            actual_acceleration=acceleration_for_init,
                        )
                    else:
                        trace["request_kind"] = "velocity"
                        trace["request_target_velocity"] = float(req.get("target"))
                        trace["request_max_velocity"] = params.get("max_velocity")
                        trace["request_max_acceleration"] = params.get("max_acceleration")
                        trace["request_max_jerk"] = params.get("max_jerk")
                        self._ruckig_planner.start_velocity(
                            pos,
                            actual_position=actual_position,
                            actual_velocity=velocity_for_init,
                            target_velocity=float(req.get("target")),
                            cfg=cfg,
                            dt_s_fallback=dt_s_fallback,
                            overrides=overrides,
                            actual_acceleration=acceleration_for_init,
                        )

                    # Seed CSP buffer to measured position to prevent a step on the first cycle.
                    self._csp_target_next[pos] = int(actual_position)
                    self._csp_target_cur[pos] = int(actual_position)
                    self._ruckig_last_error[pos] = None
                except (RuckigUnavailable, ValueError) as e:
                    self._ruckig_last_error[pos] = str(e)
                    self._ruckig_planner.stop(pos)
                finally:
                    self._ruckig_requests.pop(pos, None)

            # Step active planner
            step = self._ruckig_planner.step(pos, actual_position=actual_position, actual_velocity=actual_velocity_drive)
            if step is not None:
                csp_target = _wrap_i32(int(step.position))
                prev_csp = self._csp_target_cur.get(pos)
                self._csp_target_next[pos] = csp_target
                trace["generated_csp_target"] = int(csp_target)
                trace["generated_csp_delta"] = None if prev_csp is None else int(csp_target - int(prev_csp))
                trace["generated_velocity"] = float(step.velocity)
                trace["generated_acceleration"] = float(step.acceleration)
                trace["planner_done"] = bool(step.done)
            else:
                step_error = self._ruckig_planner.consume_last_error(pos)
                if step_error:
                    self._ruckig_last_error[pos] = f"trajectory solve failed: {step_error}"
                    trace["step_error"] = step_error

    def run(self):
        try:
            self._run_inner()
        except SystemExit:
            raise
        except Exception:
            self._emit_process_log("ETHERCAT PROCESS FATAL:", stderr=True)
            traceback.print_exc(file=sys.stderr)
            sys.stderr.flush()
            raise

    def _run_inner(self):
        self._install_signal_handlers()
        self._apply_irq_affinity()

        if self.cfg.rt_priority is not None:
            SCHED_FIFO = 1
            param = os.sched_param(self.cfg.rt_priority)
            os.sched_setscheduler(0, SCHED_FIFO, param)

        if self.cfg.cpu_core is not None:
            os.sched_setaffinity(0, {self.cfg.cpu_core})

        # Lock all current and future memory pages to prevent page faults
        try:
            _libc = ctypes.CDLL("libc.so.6", use_errno=True)
            _MCL_CURRENT, _MCL_FUTURE = 1, 2
            if _libc.mlockall(_MCL_CURRENT | _MCL_FUTURE) != 0:
                self._emit_process_log("[EC] mlockall failed", stderr=True)
            else:
                self._emit_process_log("[EC] mlockall succeeded")
        except Exception as e:
            self._emit_process_log(f"[EC] mlockall unavailable: {e}", stderr=True)

        # Set up clock_nanosleep for absolute monotonic sleep
        _use_clock_nanosleep = False
        try:
            _libc_ns = ctypes.CDLL("libc.so.6", use_errno=True)
            _CLOCK_MONOTONIC = 1
            _TIMER_ABSTIME = 1

            class _timespec(ctypes.Structure):
                _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]

            _clock_nanosleep = _libc_ns.clock_nanosleep
            _clock_nanosleep.argtypes = [
                ctypes.c_int, ctypes.c_int,
                ctypes.POINTER(_timespec), ctypes.POINTER(_timespec),
            ]
            _clock_nanosleep.restype = ctypes.c_int
            _ts_sleep = _timespec()
            _use_clock_nanosleep = True
            self._emit_process_log("[EC] clock_nanosleep available (absolute monotonic sleep)")
        except Exception as e:
            self._emit_process_log(f"[EC] clock_nanosleep unavailable, falling back to time.sleep: {e}", stderr=True)

        import threading
        self._log_q = queue.Queue(maxsize=256)

        def _log_writer():
            fh = None
            if self._process_log_file:
                try:
                    fh = open(self._process_log_file, "a", encoding="utf-8")
                except Exception:
                    pass
            while True:
                try:
                    item = self._log_q.get(timeout=1.0)
                except queue.Empty:
                    if self.stop_event.is_set():
                        break
                    continue
                if item is None:
                    break
                line, to_file, to_stderr = item
                if to_file and fh:
                    try:
                        fh.write(line)
                        fh.flush()
                    except Exception:
                        pass
                if to_stderr:
                    try:
                        sys.stderr.write(line)
                        sys.stderr.flush()
                    except Exception:
                        pass
            if fh:
                try:
                    fh.flush()
                    fh.close()
                except Exception:
                    pass

        self._log_thread = threading.Thread(target=_log_writer, daemon=True)
        self._log_thread.start()

        ok = self._setup()
        if not ok:
            self._teardown()
            raise SystemExit(1)

        cycle_ns = int(self.cfg.cycle_time_ms * 1_000_000)
        if cycle_ns <= 0:
            raise ValueError("cycle_time_ms must be > 0")

        next_cycle_mono = time.monotonic_ns()
        prev_cycle_start_mono = time.monotonic_ns()
        last_timing_log_mono = prev_cycle_start_mono
        try:
            while not self.stop_event.is_set():
                cycle_start_mono = time.monotonic_ns()
                actual_cycle_ns = cycle_start_mono - prev_cycle_start_mono
                prev_cycle_start_mono = cycle_start_mono
                self._last_cycle_time_ns = int(actual_cycle_ns)
                self._last_cycle_time_us = int(actual_cycle_ns / 1000)
                cycle_jitter_ns = actual_cycle_ns - cycle_ns
                self._last_cycle_jitter_ns = int(cycle_jitter_ns)
                self._last_cycle_jitter_us = int(cycle_jitter_ns / 1000)
                self._window_append(self._jitter_window_ns, self._jitter_sorted_ns, self._last_cycle_jitter_ns)
                self._window_append(self._jitter_window, self._jitter_sorted_us, self._last_cycle_jitter_us)
                max_abs_cycle_jitter_ns = int(abs(cycle_jitter_ns))
                if max_abs_cycle_jitter_ns > self._max_abs_cycle_jitter_ns:
                    self._max_abs_cycle_jitter_ns = max_abs_cycle_jitter_ns
                max_abs_cycle_jitter_us = int(abs(cycle_jitter_ns) / 1000)
                if max_abs_cycle_jitter_us > self._max_abs_cycle_jitter_us:
                    self._max_abs_cycle_jitter_us = max_abs_cycle_jitter_us
                if self.cycle_count >= self._jitter_warmup_cycles:
                    if max_abs_cycle_jitter_ns > self._max_abs_cycle_jitter_post_warmup_ns:
                        self._max_abs_cycle_jitter_post_warmup_ns = max_abs_cycle_jitter_ns
                    if max_abs_cycle_jitter_us > self._max_abs_cycle_jitter_post_warmup_us:
                        self._max_abs_cycle_jitter_post_warmup_us = max_abs_cycle_jitter_us
                lateness_ns = cycle_start_mono - next_cycle_mono
                if lateness_ns > self._deadline_miss_threshold_ns:
                    self._deadline_miss_count += 1
                self._log_actual_sum += int(actual_cycle_ns)
                self._log_jitter_sum += int(abs(cycle_jitter_ns))
                if lateness_ns > self._log_lateness_max:
                    self._log_lateness_max = lateness_ns
                self._log_cycle_count += 1
                timing_log_period_ns = int(float(getattr(self.cfg, "process_timing_log_period_s", 1.0)) * 1_000_000_000)
                if timing_log_period_ns <= 0:
                    timing_log_period_ns = 1_000_000_000
                if bool(getattr(self.cfg, "process_timing_log", True)) and cycle_start_mono - last_timing_log_mono >= timing_log_period_ns:
                    _n = max(1, self._log_cycle_count)
                    self._emit_process_log(
                        f"[EC] avg:actual={self._log_actual_sum // _n} "
                        f"avg:jitter={self._log_jitter_sum // _n} "
                        f"max:lateness={self._log_lateness_max} "
                        f"max:work={self._log_work_max} "
                        f"min:sleep_win={self._min_sleep_budget_window_ns} "
                        f"min:sleep={self._min_sleep_budget_ns} "
                        f"overruns={self._overrun_count} "
                        f"max:recv={self._phase_recv_max} "
                        f"max:state={self._phase_state_max} "
                        f"max:cia={self._phase_cia_max} "
                        f"max:ruckig={self._phase_ruckig_max} "
                        f"max:pdo_w={self._phase_write_max} "
                        f"max:dc_send={self._phase_dc_send_max} "
                        f"max:tail={self._phase_tail_max} "
                        f"dc_err={self._last_dc_sync_error_ns} "
                        f"cycles={_n}"
                    )
                    self._log_actual_sum = 0
                    self._log_jitter_sum = 0
                    self._log_lateness_max = 0
                    self._log_work_max = 0
                    self._log_cycle_count = 0
                    self._phase_recv_max = 0
                    self._phase_state_max = 0
                    self._phase_cia_max = 0
                    self._phase_ruckig_max = 0
                    self._phase_write_max = 0
                    self._phase_dc_send_max = 0
                    self._phase_tail_max = 0
                    self._min_sleep_budget_window_ns = None
                    last_timing_log_mono = cycle_start_mono
                self.cycle_count += 1
                next_cycle_mono += cycle_ns

                for _ in range(16):
                    try:
                        cmd = self.cmd_q.get_nowait()
                        self._handle_command(cmd)
                    except queue.Empty:
                        break

                if not self.cfg.sdo_only and self.domain is not None:
                    _t0 = time.monotonic_ns()
                    self.master.receive()
                    self.master.process_domain(self.domain)
                    _t_recv = time.monotonic_ns()

                    # Poll SDO request objects after receive/process so completions are observed.
                    try:
                        self._sdo_mgr.poll(int(self.cycle_count))
                    except Exception:
                        pass

                    if self._dc_master_sync_mode == "m2s":
                        # Pure M2S: discipline application_time from the reference slave DC clock.
                        # The 32-bit ref time is only valid after a prior sync_slave_clocks() datagram
                        # has been received, so we gracefully fall back until then.
                        ref32 = None
                        try:
                            ref32 = self.master.reference_clock_time_32()
                        except Exception:
                            ref32 = None
                        if ref32 is not None:
                            if self._m2s_last_app_time_ns is None:
                                self._m2s_last_app_time_ns = int(time.monotonic_ns())

                            if getattr(self._m2s_ref_ext, "_last32", None) is None:
                                self._m2s_ref_time_ns = int(self._m2s_ref_ext.seed_near(self._m2s_last_app_time_ns, int(ref32)))
                            else:
                                self._m2s_ref_time_ns = int(self._m2s_ref_ext.update(int(ref32)))

                            if not self._m2s_logged_lock:
                                self._emit_process_log(
                                    f"[DC] M2S locked: ref32=0x{int(ref32) & 0xFFFF_FFFF:08X} "
                                    f"ref_time_ns={int(self._m2s_ref_time_ns)} app_time_ns={int(self._m2s_last_app_time_ns)}",
                                    stderr=True,
                                )
                                self._m2s_logged_lock = True

                            if not self._m2s_logged_pll:
                                self._emit_process_log(
                                    f"[DC] M2S PLL: k={self._m2s_k:g} window={getattr(self._m2s_phase_err_window, 'maxlen', None)} "
                                    f"max_corr_ns={self._m2s_max_correction_ns}",
                                    stderr=True,
                                )
                                self._m2s_logged_pll = True

                            # Predict next app_time by nominal cycle, then apply a bounded correction toward ref_time.
                            prev_app = int(self._m2s_last_app_time_ns)
                            predicted = prev_app + int(cycle_ns)
                            phase_err = int(self._m2s_ref_time_ns) - int(predicted)
                            self._m2s_phase_err_window.append(phase_err)
                            filt_err = _median_i64(list(self._m2s_phase_err_window))
                            corr = int(self._m2s_k * float(filt_err))
                            if corr > self._m2s_max_correction_ns:
                                corr = self._m2s_max_correction_ns
                            elif corr < -self._m2s_max_correction_ns:
                                corr = -self._m2s_max_correction_ns

                            new_app = predicted + corr
                            if new_app <= prev_app:
                                new_app = prev_app + 1
                            self._m2s_last_app_time_ns = int(new_app)
                        else:
                            if self._m2s_last_app_time_ns is None:
                                self._m2s_last_app_time_ns = int(time.monotonic_ns())
                            else:
                                self._m2s_last_app_time_ns += int(cycle_ns)
                            if not self._m2s_logged_fallback:
                                self._emit_process_log(
                                    "[DC] M2S waiting for reference clock time (fallback app_time active)",
                                    stderr=True,
                                )
                                self._m2s_logged_fallback = True
                        try:
                            self.master.set_application_time(int(self._m2s_last_app_time_ns))
                        except Exception:
                            pass
                    else:
                        try:
                            # Use monotonic time for application_time to avoid wall-clock jumps (NTP/RTC adjustments)
                            # destabilizing DC sync and triggering AL 0x001A sync errors.
                            self.master.set_application_time(time.monotonic_ns())
                        except Exception:
                            pass

                    wc, wc_state = self.master.domain_state(self.domain)
                    self._last_domain_wc = wc
                    self._last_domain_wc_state = wc_state
                    if self._domain_wc_min is None or wc < self._domain_wc_min:
                        self._domain_wc_min = wc
                    if self._domain_wc_max is None or wc > self._domain_wc_max:
                        self._domain_wc_max = wc
                    for pos in self.slave_in_op:
                        sc = self.slave_handles.get(pos)
                        if sc is None:
                            continue
                        sc_state = sc.get_state()
                        al_state = sc_state.get('al_state', 0) if sc_state else 0

                        is_safeop = (al_state >= AL_STATE_SAFEOP)
                        was_safeop = self.slave_in_safeop.get(pos, False)
                        if is_safeop and not was_safeop:
                            self.slave_in_safeop[pos] = True
                            if bool(getattr(self.cfg, "process_op_transition_log", True)):
                                self._emit_process_log(f"[DC] Slave {pos} entered SAFEOP (al_state=0x{al_state:02X})")

                        is_op = sc_state is not None and sc_state.get('operational', False)
                        was_op = self.slave_in_op[pos]
                        if is_op and not was_op:
                            self.slave_in_op[pos] = True
                            if bool(getattr(self.cfg, "process_op_transition_log", True)):
                                self._emit_process_log(f"[DC] Slave {pos} entered OP (al_state=0x{al_state:02X})")
                            if pos not in self._op_entered_first_ns:
                                self._op_entered_first_ns[pos] = cycle_start_mono
                            self._op_entered_last_ns[pos] = cycle_start_mono
                        elif not is_op and was_op:
                            self.slave_in_op[pos] = False
                            self.drive_enabled[pos] = False
                            self.desired_controlword.pop(pos, None)
                            self._pp_pulse_active[pos] = False
                            self._pp_pulse_start_ns[pos] = None
                            self._pp_pulse_pending[pos] = False
                            if bool(getattr(self.cfg, "process_op_transition_log", True)):
                                self._emit_process_log(f"[DC] Slave {pos} left OP (al_state=0x{al_state:02X}, domain WC={wc})")
                            self._op_left_last_ns[pos] = cycle_start_mono
                            self._op_dropout_count[pos] = int(self._op_dropout_count.get(pos, 0)) + 1
                            self._dc_settling_complete = False
                            self._dc_settling_logged = False

                    all_in_safeop = bool(self.slave_in_safeop) and all(self.slave_in_safeop.values())
                    if all_in_safeop and self._all_safeop_first_ns is None:
                        self._all_safeop_first_ns = cycle_start_mono
                        if not self._all_safeop_logged:
                            self._emit_process_log(f"[DC] === ALL {len(self.slave_in_safeop)} SLAVES IN SAFEOP ===")
                            self._all_safeop_logged = True

                    all_in_op = bool(self.slave_in_op) and all(self.slave_in_op.values())
                    if all_in_op:
                        if self._all_op_first_ns is None:
                            self._all_op_first_ns = cycle_start_mono
                            self._emit_process_log(f"[DC] === ALL {len(self.slave_in_op)} SLAVES IN OP ===")
                            
                            # Set expected working counter for non-RT monitoring
                            if not self.cfg.sdo_only and self.domain is not None:
                                if self._wc_expected == 0:
                                    self._wc_expected = self._last_domain_wc
                                    self._emit_process_log(f"[EC] Expected WC set to {self._wc_expected}")
                        self._all_op_last_ns = cycle_start_mono

                        if not self._dc_settling_complete:
                            settling_delay_ns = int(getattr(self.cfg, 'dc_settling_delay_s', 2.0) * 1_000_000_000)
                            elapsed_ns = cycle_start_mono - self._all_op_first_ns
                            if elapsed_ns >= settling_delay_ns:
                                self._dc_settling_complete = True
                                self._emit_process_log(f"[DC] DC settling complete after {elapsed_ns / 1_000_000_000:.2f}s - CiA402 enables allowed")
                            elif not self._dc_settling_logged:
                                self._emit_process_log(f"[DC] Waiting for DC settling ({settling_delay_ns / 1_000_000_000:.1f}s)...")
                                self._dc_settling_logged = True
                    else:
                        self._all_left_op_last_ns = cycle_start_mono

                    if self._activated_mono_ns is not None:
                        elapsed_s = (time.monotonic_ns() - self._activated_mono_ns) / 1_000_000_000.0
                        if elapsed_s >= float(self.cfg.op_timeout_s):
                            not_op = [p for p in self.slave_in_op.keys() if not self.slave_in_op.get(p, False)]
                            if not_op:
                                if not self._op_timeout_active:
                                    self._emit_process_log(
                                        f"[EC] OP timeout after {self.cfg.op_timeout_s}s. Not in OP: {not_op}. Domain WC={wc}, state={wc_state}",
                                        stderr=True,
                                    )
                                    self._op_timeout_active = True
                            else:
                                if self._op_timeout_active:
                                    self._emit_process_log("[EC] OP timeout condition cleared; all slaves back in OP")
                                self._op_timeout_active = False

                _t_state = time.monotonic_ns()
                self._auto_enable_drives()
                _t_cia = time.monotonic_ns()
                self._update_ruckig()
                if self._die_velocity_test_active:
                    self._update_die_velocity_test()
                else:
                    self._update_semi_rotary_rt()
                _t_ruckig = time.monotonic_ns()
                self._read_targets_from_shm()
                self._cyclic_write()
                _t_write = time.monotonic_ns()

                if self._dc_master_sync_mode == "m2s":
                    self.master.sync_slave_clocks()
                else:
                    self.master.sync_reference_clock()
                    self.master.sync_slave_clocks()
                    # DC sync monitor: process last result, then queue next cycle's monitor datagram.
                    try:
                        dc_err_ns = self.master.sync_monitor_process()
                        if dc_err_ns is not None:
                            dc_err_ns = int(dc_err_ns)
                            self._last_dc_sync_error_ns = dc_err_ns
                            self._window_append(self._dc_sync_error_window_ns, self._dc_sync_sorted_ns, dc_err_ns)
                            abs_dc = abs(dc_err_ns)
                            if abs_dc > self._max_abs_dc_sync_error_ns:
                                self._max_abs_dc_sync_error_ns = abs_dc
                        self.master.sync_monitor_queue()
                    except Exception:
                        pass

                    # Queue any due SDO request-object operations before sending datagrams.
                    try:
                        if self._sdo_queue_allowed_now():
                            self._sdo_mgr.queue_due(int(self.cycle_count))
                        else:
                            # Prevent backlog / repeated queue attempts during unhealthy OP/WC/DC windows.
                            self._sdo_mgr.clear_all_desired()
                    except Exception:
                        pass

                    self.master.queue_domain(self.domain)
                    self.master.send()

                    post_send_mono = time.monotonic_ns()
                    self._phase_recv = int(_t_recv - _t0)
                    self._phase_state = int(_t_state - _t_recv)
                    self._phase_cia = int(_t_cia - _t_state)
                    self._phase_ruckig = int(_t_ruckig - _t_cia)
                    self._phase_write = int(_t_write - _t_ruckig)
                    self._phase_dc_send = int(post_send_mono - _t_write)
                    if self._phase_recv > self._phase_recv_max:
                        self._phase_recv_max = self._phase_recv
                    if self._phase_state > self._phase_state_max:
                        self._phase_state_max = self._phase_state
                    if self._phase_cia > self._phase_cia_max:
                        self._phase_cia_max = self._phase_cia
                    if self._phase_ruckig > self._phase_ruckig_max:
                        self._phase_ruckig_max = self._phase_ruckig
                    if self._phase_write > self._phase_write_max:
                        self._phase_write_max = self._phase_write
                    if self._phase_dc_send > self._phase_dc_send_max:
                        self._phase_dc_send_max = self._phase_dc_send
                    work_ns = int(post_send_mono - cycle_start_mono)
                    self._last_work_ns = work_ns
                    if work_ns > self._max_work_ns:
                        self._max_work_ns = work_ns
                    if work_ns > self._log_work_max:
                        self._log_work_max = work_ns
                    self._window_append(self._work_window_ns, self._work_sorted_ns, work_ns)
                    if self._prev_send_mono is not None:
                        self._last_send_interval_ns = int(post_send_mono - self._prev_send_mono)
                    self._prev_send_mono = post_send_mono

                    self._write_actuals_to_shm()

                sleep_ns = next_cycle_mono - time.monotonic_ns()
                if self._min_sleep_budget_ns is None or sleep_ns < self._min_sleep_budget_ns:
                    self._min_sleep_budget_ns = sleep_ns
                # Windowed min sleep budget for timing logs (reset after each timing log emission).
                if self._min_sleep_budget_window_ns is None or sleep_ns < self._min_sleep_budget_window_ns:
                    self._min_sleep_budget_window_ns = sleep_ns
                if sleep_ns <= 0:
                    self._overrun_count += 1
                    overrun_magnitude = abs(sleep_ns)
                    if overrun_magnitude > self._max_overrun_ns:
                        self._max_overrun_ns = overrun_magnitude
                    now_ns = time.monotonic_ns()
                    while next_cycle_mono <= now_ns:
                        next_cycle_mono += cycle_ns

                if _use_clock_nanosleep:
                    _ts_sleep.tv_sec = next_cycle_mono // 1_000_000_000
                    _ts_sleep.tv_nsec = next_cycle_mono % 1_000_000_000
                    _clock_nanosleep(_CLOCK_MONOTONIC, _TIMER_ABSTIME, ctypes.byref(_ts_sleep), None)
                else:
                    remaining = next_cycle_mono - time.monotonic_ns()
                    if remaining > 0:
                        time.sleep(remaining / 1_000_000_000.0)
        finally:
            self._teardown()


class EtherCATProcessManager:
    """
    Spawns EtherCATProcess and provides enqueue/read handles for the application.
    """

    def __init__(self, cfg: EthercatNetworkConfig):
        self.cfg = cfg
        self._cmd_q: mp.Queue = mp.Queue(maxsize=1024)
        self._status_q: mp.Queue = mp.Queue(maxsize=64)
        self._stop_event: mp.Event = mp.Event()
        self._proc: Optional[mp.Process] = None
        self._latest_status: Optional[NetworkStatus] = None
        self._actuals_shm = mp.Array('l', SHM.ACTUALS_SIZE, lock=False)
        self._targets_shm = mp.Array('l', SHM.TARGETS_SIZE, lock=False)
        self._semi_rotary_proc: Optional[mp.Process] = None
        self._semi_rotary_stop: Optional[mp.Event] = None
        self._semi_rotary_cfg_shm = mp.Array('l', 8192, lock=False)

    def start(self):
        if self._proc and self._proc.is_alive():
            return
        self._stop_event.clear()
        target = EtherCATProcess(
            self.cfg, self._cmd_q, self._status_q, self._stop_event,
            actuals_shm=self._actuals_shm, targets_shm=self._targets_shm,
        )
        self._proc = mp.Process(target=target.run, daemon=False)
        self._proc.start()

    def is_alive(self) -> bool:
        return bool(self._proc and self._proc.is_alive())

    def exitcode(self) -> Optional[int]:
        if not self._proc:
            return None
        return self._proc.exitcode

    def stop(self):
        self._stop_semi_rotary()
        if not self._proc:
            return
        if not self._proc.is_alive():
            self._proc = None
            return
        logger.info("Stopping EtherCAT process...")
        self._stop_event.set()
        self._proc.join(timeout=5.0)
        if self._proc.is_alive():
            logger.warning("Graceful stop timed out, sending SIGTERM...")
            self._proc.terminate()
            self._proc.join(timeout=2.0)
        if self._proc.is_alive():
            logger.warning("SIGTERM timed out, sending SIGKILL...")
            self._proc.kill()
            self._proc.join(timeout=2.0)
        self._proc = None
        logger.info("EtherCAT process stopped")

    def _start_semi_rotary(self, params: dict) -> None:
        self._stop_semi_rotary()
        from .semi_rotary_process import _semi_rotary_loop
        comp_counts = params.get("comp_counts") or []
        n_samples = int(params.get("n_samples") or len(comp_counts))
        cfg = self._semi_rotary_cfg_shm
        cfg[0] = 0
        cfg[1] = 1  # START
        cfg[2] = int(params["die_pos"])
        cfg[3] = int(params["shuttle_pos"])
        cfg[4] = int(params["nip_in_pos"]) if params.get("nip_in_pos") is not None else -1
        cfg[5] = int(params["nip_out_pos"]) if params.get("nip_out_pos") is not None else -1
        cfg[6] = int(params["die_start"])
        cfg[7] = int(params["die_counts_per_rev"])
        cfg[8] = int(params["shuttle_center"])
        cfg[9] = int(params.get("blend_cycles") or 0)
        cfg[10] = int(params.get("max_shuttle_excursion") or 0)
        cfg[11] = int(params.get("max_shuttle_delta_per_cycle") or 0)
        cfg[12] = 1 if params.get("enable_nips") else 0
        cfg[13] = int(params.get("nip_in_counts_per_rev") or 0)
        cfg[14] = int(params.get("nip_out_counts_per_rev") or 0)
        cfg[15] = int(params.get("nip_in_start") or 0)
        cfg[16] = int(params.get("nip_out_start") or 0)
        cfg[17] = n_samples
        for i, v in enumerate(comp_counts[:n_samples]):
            cfg[64 + i] = int(v)
        self._semi_rotary_stop = mp.Event()
        cfg[0] = 1  # signal config ready
        self._semi_rotary_proc = mp.Process(
            target=_semi_rotary_loop,
            args=(self._actuals_shm, self._targets_shm, cfg, self._semi_rotary_stop, self.cfg.cycle_time_ms),
            daemon=True,
        )
        self._semi_rotary_proc.start()
        logger.info("Semi-rotary cam process started")

    def _update_semi_rotary(self, params: dict) -> None:
        if self._semi_rotary_proc is None or not self._semi_rotary_proc.is_alive():
            return
        cfg = self._semi_rotary_cfg_shm
        comp_counts = params.get("comp_counts") or []
        n_samples = int(params.get("n_samples") or len(comp_counts))
        cfg[1] = 2  # UPDATE
        cfg[9] = int(params.get("blend_cycles") or 0)
        cfg[10] = int(params.get("max_shuttle_excursion") or 0)
        cfg[11] = int(params.get("max_shuttle_delta_per_cycle") or 0)
        cfg[12] = 1 if params.get("enable_nips") else 0
        cfg[13] = int(params.get("nip_in_counts_per_rev") or 0)
        cfg[14] = int(params.get("nip_out_counts_per_rev") or 0)
        cfg[17] = n_samples
        for i, v in enumerate(comp_counts[:n_samples]):
            cfg[64 + i] = int(v)
        cfg[0] += 1  # bump sequence to signal update

    def _stop_semi_rotary(self) -> None:
        if self._semi_rotary_proc is not None:
            cfg = self._semi_rotary_cfg_shm
            cfg[1] = 0  # STOP
            cfg[0] += 1
            if self._semi_rotary_stop is not None:
                self._semi_rotary_stop.set()
            self._semi_rotary_proc.join(timeout=2.0)
            if self._semi_rotary_proc.is_alive():
                self._semi_rotary_proc.terminate()
                self._semi_rotary_proc.join(timeout=1.0)
            self._semi_rotary_proc = None
            self._semi_rotary_stop = None
            logger.info("Semi-rotary cam process stopped")

    # Application API
    def send_command(self, cmd: Command):
        if cmd.type == CommandType.START_SEMI_ROTARY_RT:
            self._start_semi_rotary(cmd.params or {})
            return True
        elif cmd.type == CommandType.UPDATE_SEMI_ROTARY_RT:
            self._update_semi_rotary(cmd.params or {})
            return True
        elif cmd.type == CommandType.STOP_SEMI_ROTARY_RT:
            self._stop_semi_rotary()
            return True
        try:
            self._cmd_q.put_nowait(cmd)
            return True
        except queue.Full:
            return False

    def _build_status_from_shm(self) -> Optional[NetworkStatus]:
        shm = self._actuals_shm
        if shm[SHM.SLOT_SEQUENCE] == 0:
            return None
        status = NetworkStatus(drives={})
        status.timestamp_ns = int(shm[SHM.SLOT_TIMESTAMP_NS])
        cycle_time_ms_x1000 = int(shm[SHM.SLOT_CYCLE_TIME_MS_X1000])
        status.cycle_time_ms_config = cycle_time_ms_x1000 / 1000.0 if cycle_time_ms_x1000 else None
        status.last_cycle_jitter_ns = int(shm[SHM.SLOT_JITTER_NS])
        status.last_cycle_jitter_us = int(shm[SHM.SLOT_JITTER_NS]) // 1000
        status.max_abs_cycle_jitter_post_warmup_ns = int(shm[SHM.SLOT_MAX_JITTER_PW_NS])
        status.max_abs_cycle_jitter_post_warmup_us = int(shm[SHM.SLOT_MAX_JITTER_PW_NS]) // 1000
        status.last_work_ns = int(shm[SHM.SLOT_WORK_NS])
        status.max_work_ns = int(shm[SHM.SLOT_MAX_WORK_NS])
        status.overrun_count = int(shm[SHM.SLOT_OVERRUN_COUNT])
        status.max_overrun_ns = int(shm[SHM.SLOT_MAX_OVERRUN_NS])
        status.last_send_interval_ns = int(shm[SHM.SLOT_SEND_INTERVAL_NS])
        status.min_sleep_budget_ns = int(shm[SHM.SLOT_MIN_SLEEP_BUDGET_NS])
        status.deadline_miss_count = int(shm[SHM.SLOT_DEADLINE_MISS_COUNT])
        status.dc_sync_error_ns = int(shm[SHM.SLOT_DC_SYNC_ERR_NS])
        status.dc_sync_error_max_ns = int(shm[SHM.SLOT_DC_SYNC_ERR_MAX_NS])
        status.domain_wc = int(shm[SHM.SLOT_DOMAIN_WC])
        status.domain_wc_state = int(shm[SHM.SLOT_DOMAIN_WC_STATE])
        status.domain_wc_min = int(shm[SHM.SLOT_DOMAIN_WC_MIN])
        status.domain_wc_max = int(shm[SHM.SLOT_DOMAIN_WC_MAX])
        status.all_slaves_op_first_ns = int(shm[SHM.SLOT_ALL_OP_FIRST_NS]) or None
        status.all_slaves_op_last_ns = int(shm[SHM.SLOT_ALL_OP_LAST_NS]) or None
        status.all_slaves_left_op_last_ns = int(shm[SHM.SLOT_ALL_OP_LEFT_LAST_NS]) or None
        for dcfg in self.cfg.slaves:
            pos = dcfg.position
            if pos >= SHM.MAX_DRIVES:
                continue
            base = SHM.DRIVE_BASE + pos * SHM.DRIVE_STRIDE
            drive = {}
            drive['in_op'] = bool(shm[base + SHM.DRIVE_IN_OP])
            drive['enabled'] = bool(shm[base + SHM.DRIVE_ENABLED])
            if not getattr(dcfg, 'cia402', True):
                raw_pdo = {}
                reg_entries = getattr(dcfg, 'register_entries', None) or []
                di_idx = 0
                do_idx = 0
                for key in reg_entries:
                    idx = key[0] if isinstance(key, tuple) else key
                    if 0x6000 <= idx < 0x6040:
                        if di_idx < SHM.SLOT_IO_DI_COUNT:
                            raw_pdo[key] = int(shm[SHM.SLOT_IO_DI_BASE + di_idx])
                            di_idx += 1
                    elif 0x7040 <= idx < 0x7080:
                        if do_idx < SHM.SLOT_IO_DO_COUNT:
                            raw_pdo[key] = int(shm[SHM.SLOT_IO_DO_BASE + do_idx])
                            do_idx += 1
                drive['raw_pdo'] = raw_pdo
                status.drives[pos] = drive
                continue
            drive['statusword'] = int(shm[base + SHM.DRIVE_STATUSWORD])
            drive['position_actual'] = int(shm[base + SHM.DRIVE_POSITION])
            drive['velocity_actual'] = int(shm[base + SHM.DRIVE_VELOCITY])
            drive['torque_actual'] = int(shm[base + SHM.DRIVE_TORQUE])
            drive['mode_display'] = int(shm[base + SHM.DRIVE_MODE])
            drive['error_code'] = int(shm[base + SHM.DRIVE_ERROR_CODE])
            drive['digital_inputs'] = int(shm[base + SHM.DRIVE_DIGITAL_INPUTS])
            drive['dip_in_state'] = int(shm[base + SHM.DRIVE_DIP_IN_STATE])
            sw = drive['statusword']
            state_bits = sw & 0x006F
            drive['fault'] = bool(sw & 0x0008)
            drive['warning'] = bool(sw & 0x0080)
            drive['target_reached'] = bool(sw & 0x0400)
            status.drives[pos] = drive
        return status

    def get_latest_status(self) -> Optional[NetworkStatus]:
        status = self._build_status_from_shm()
        if status is not None:
            self._latest_status = status
        return status

    def get_status_snapshot(self) -> Optional[NetworkStatus]:
        if self._latest_status is None:
            return self._build_status_from_shm()
        return self._latest_status


