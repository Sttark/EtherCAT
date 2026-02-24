"""
Separate process for semi-rotary cam interpolation.

Reads die position from actuals_shm, computes cam compensation targets,
writes shuttle/nip CSP targets to targets_shm. Runs off the RT core.
"""

import multiprocessing as mp
import time
from typing import Any, Dict, Optional

from . import shm_layout as SHM
from .constants import MODE_CSP


def _semi_rotary_loop(
    actuals_shm,
    targets_shm,
    cfg_shm,
    stop_event: mp.Event,
    cycle_time_ms: float,
):
    cycle_ns = int(cycle_time_ms * 1_000_000)
    cycle_s = cycle_ns / 1_000_000_000.0

    die_pos = 0
    shuttle_pos = 0
    nip_in_pos = None
    nip_out_pos = None
    die_start = 0
    die_counts_per_rev = 0.0
    comp_counts = []
    n_samples = 0
    shuttle_center = 0
    blend_cycles = 0
    max_shuttle_excursion = None
    max_shuttle_delta_per_cycle = None
    enable_nips = False
    nip_in_counts_per_rev = 0.0
    nip_out_counts_per_rev = 0.0
    nip_in_start = 0
    nip_out_start = 0

    active = False
    cycle_count = 0
    prev_comp_target = None

    last_seq = 0

    while not stop_event.is_set():
        # Check for config updates from cfg_shm
        cfg_seq = int(cfg_shm[0])
        if cfg_seq != last_seq and cfg_seq > 0:
            last_seq = cfg_seq
            cmd = int(cfg_shm[1])
            if cmd == 1:  # START
                active = True
                cycle_count = 0
                prev_comp_target = None
                die_pos = int(cfg_shm[2])
                shuttle_pos = int(cfg_shm[3])
                nip_in_pos = int(cfg_shm[4]) if cfg_shm[4] != -1 else None
                nip_out_pos = int(cfg_shm[5]) if cfg_shm[5] != -1 else None
                die_start = int(cfg_shm[6])
                die_counts_per_rev = float(cfg_shm[7])
                shuttle_center = int(cfg_shm[8])
                blend_cycles = int(cfg_shm[9])
                max_shuttle_excursion = int(cfg_shm[10]) if cfg_shm[10] != 0 else None
                max_shuttle_delta_per_cycle = int(cfg_shm[11]) if cfg_shm[11] != 0 else None
                enable_nips = bool(cfg_shm[12])
                nip_in_counts_per_rev = float(cfg_shm[13])
                nip_out_counts_per_rev = float(cfg_shm[14])
                nip_in_start = int(cfg_shm[15])
                nip_out_start = int(cfg_shm[16])
                n_samples = int(cfg_shm[17])
                comp_counts = [int(cfg_shm[64 + i]) for i in range(n_samples)]
            elif cmd == 2:  # UPDATE
                n_new = int(cfg_shm[17])
                if n_new > 0:
                    n_samples = n_new
                    comp_counts = [int(cfg_shm[64 + i]) for i in range(n_samples)]
                if cfg_shm[9] != 0:
                    blend_cycles = int(cfg_shm[9])
                if cfg_shm[11] != 0:
                    max_shuttle_delta_per_cycle = int(cfg_shm[11])
                if cfg_shm[10] != 0:
                    max_shuttle_excursion = int(cfg_shm[10])
                enable_nips = bool(cfg_shm[12])
                nip_in_counts_per_rev = float(cfg_shm[13])
                nip_out_counts_per_rev = float(cfg_shm[14])
            elif cmd == 0:  # STOP
                active = False

        if not active:
            time.sleep(cycle_s)
            continue

        abs_counts_per_rev = abs(die_counts_per_rev)
        if abs_counts_per_rev <= 0.0 or n_samples <= 0:
            time.sleep(cycle_s)
            continue

        # Read die actual position from actuals_shm
        if die_pos >= SHM.MAX_DRIVES:
            time.sleep(cycle_s)
            continue
        die_base = SHM.DRIVE_BASE + die_pos * SHM.DRIVE_STRIDE
        die_enabled = bool(actuals_shm[die_base + SHM.DRIVE_ENABLED])
        shuttle_base = SHM.DRIVE_BASE + shuttle_pos * SHM.DRIVE_STRIDE
        shuttle_enabled = bool(actuals_shm[shuttle_base + SHM.DRIVE_ENABLED])

        if not die_enabled or not shuttle_enabled:
            time.sleep(cycle_s)
            continue

        die_actual = int(actuals_shm[die_base + SHM.DRIVE_POSITION])
        die_elapsed = die_actual - die_start

        if die_counts_per_rev < 0:
            die_phase = ((-die_elapsed) % abs_counts_per_rev) / abs_counts_per_rev
        else:
            die_phase = (die_elapsed % abs_counts_per_rev) / abs_counts_per_rev

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

        blend = 1.0 if blend_cycles <= 0 else min(1.0, float(cycle_count) / float(blend_cycles))
        comp_target = shuttle_center + int(comp_raw * blend)

        if max_shuttle_excursion is not None:
            max_exc = abs(max_shuttle_excursion)
            lo = shuttle_center - max_exc
            hi = shuttle_center + max_exc
            if comp_target < lo:
                comp_target = lo
            elif comp_target > hi:
                comp_target = hi

        if prev_comp_target is not None and max_shuttle_delta_per_cycle is not None:
            delta = comp_target - prev_comp_target
            lim = abs(max_shuttle_delta_per_cycle)
            if abs(delta) > lim:
                comp_target = prev_comp_target + (lim if delta > 0 else -lim)

        prev_comp_target = comp_target
        rev_count = die_elapsed / die_counts_per_rev

        # Write shuttle target
        shuttle_tgt_base = SHM.TARGET_BASE + shuttle_pos * SHM.TARGET_STRIDE
        targets_shm[shuttle_tgt_base + SHM.TARGET_CSP] = int(comp_target)
        targets_shm[shuttle_tgt_base + SHM.TARGET_MODE] = MODE_CSP
        targets_shm[shuttle_tgt_base + SHM.TARGET_VALID] = 1

        # Write nip targets
        if enable_nips:
            if nip_in_pos is not None:
                nip_in_base = SHM.DRIVE_BASE + nip_in_pos * SHM.DRIVE_STRIDE
                if bool(actuals_shm[nip_in_base + SHM.DRIVE_ENABLED]):
                    nip_in_target = nip_in_start + int(rev_count * nip_in_counts_per_rev)
                    tgt_base = SHM.TARGET_BASE + nip_in_pos * SHM.TARGET_STRIDE
                    targets_shm[tgt_base + SHM.TARGET_CSP] = int(nip_in_target)
                    targets_shm[tgt_base + SHM.TARGET_MODE] = MODE_CSP
                    targets_shm[tgt_base + SHM.TARGET_VALID] = 1
            if nip_out_pos is not None:
                nip_out_base = SHM.DRIVE_BASE + nip_out_pos * SHM.DRIVE_STRIDE
                if bool(actuals_shm[nip_out_base + SHM.DRIVE_ENABLED]):
                    nip_out_target = nip_out_start + int(rev_count * nip_out_counts_per_rev)
                    tgt_base = SHM.TARGET_BASE + nip_out_pos * SHM.TARGET_STRIDE
                    targets_shm[tgt_base + SHM.TARGET_CSP] = int(nip_out_target)
                    targets_shm[tgt_base + SHM.TARGET_MODE] = MODE_CSP
                    targets_shm[tgt_base + SHM.TARGET_VALID] = 1

        targets_shm[SHM.TARGET_SEQUENCE] = cycle_count
        cycle_count += 1

        time.sleep(cycle_s)
