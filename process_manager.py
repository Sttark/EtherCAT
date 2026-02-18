import logging
import multiprocessing as mp
import os
import queue
import signal
import sys
import time
import traceback
from typing import Any, Dict, List, Optional, Tuple

from .commands import Command, CommandType
from .status_model import NetworkStatus
from .constants import (
    CW_INDEX, SW_INDEX,
    MODES_OP_INDEX, MODES_OP_DISPLAY_INDEX,
    TARGET_POSITION_INDEX, TARGET_VELOCITY_INDEX, TARGET_TORQUE_INDEX,
    POSITION_ACTUAL_INDEX, VELOCITY_ACTUAL_INDEX, TORQUE_ACTUAL_INDEX,
    PROBE_FUNCTION_INDEX, PROBE_STATUS_INDEX, PROBE_POS1_INDEX, PROBE_POS2_INDEX, PROBE_POS2_ALT_INDEX, DIGITAL_INPUTS_INDEX,
    MODE_PP, MODE_PV, MODE_PT, MODE_CSP, MODE_HM,
    CW_BIT_NEW_SET_POINT, CW_BIT_CHANGE_IMMEDIATELY, CW_BIT_ABS_REL, CW_BIT_HALT, CW_ENABLE_OP_SIMPLIFIED,
    PROBE_FUNC_ENABLE_PROBE1, PROBE_FUNC_PROBE1_POS_EDGE, PROBE_FUNC_PROBE1_NEG_EDGE,
)
from .config_schema import EthercatNetworkConfig, DriveConfig
from .igh_master import Master
from .xml_decoder import decode_esi
from .ruckig_planner import RuckigCspPlanner, RuckigUnavailable


logger = logging.getLogger(__name__)

AL_STATE_OP = 0x08


class EtherCATProcess:
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

    def __init__(self, cfg: EthercatNetworkConfig, cmd_q: mp.Queue, status_q: mp.Queue, stop_event: mp.Event):
        self.cfg = cfg
        self.cmd_q = cmd_q
        self.status_q = status_q
        self.stop_event = stop_event
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
        self.last_probe_arm: Dict[int, Optional[int]] = {}
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

        # Ruckig (optional CSP trajectory generation)
        self._ruckig_planner: Optional[RuckigCspPlanner] = None
        self._ruckig_requests: Dict[int, Dict[str, Any]] = {}  # slave_pos -> request dict
        self._ruckig_last_error: Dict[int, Optional[str]] = {}

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

                register_pairs = []
                for p, entries in rx_pdo_map.items():
                    register_pairs.extend([(idx, sub) for (idx, sub, _bits) in entries])
                for p, entries in tx_pdo_map.items():
                    register_pairs.extend([(idx, sub) for (idx, sub, _bits) in entries])
                register_list = list(set(register_pairs))

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

            self.features[dcfg.position] = supports
            self._enable_requested.setdefault(dcfg.position, dcfg.position in self._cia402_positions)
            self._manual_disable.setdefault(dcfg.position, False)

            if dcfg.enable_dc and not self.cfg.sdo_only:
                cycle_time_ns = int(self.cfg.cycle_time_ms * 1_000_000)
                dc_assign = dcfg.dc_assign_activate if dcfg.dc_assign_activate is not None else 0x0300
                sync0_shift = dcfg.dc_sync0_shift_ns
                sync1_cycle = dcfg.dc_sync1_cycle_time_ns
                sync1_shift = dcfg.dc_sync1_shift_ns

                s.config_dc(
                    assign_activate=dc_assign,
                    sync0_cycle_time_ns=cycle_time_ns,
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

            dc_ref_pos = getattr(self.cfg, 'dc_reference_slave', None)
            if dc_ref_pos is not None:
                s = self.slave_handles.get(dc_ref_pos)
                if s:
                    self.master.select_reference_clock(s)
                    logger.info(f"Selected slave {dc_ref_pos} as DC reference clock (explicit config)")
                else:
                    logger.error(f"dc_reference_slave={dc_ref_pos} not found in slave_handles")
            else:
                for dcfg in self.cfg.slaves:
                    if dcfg.enable_dc:
                        s = self.slave_handles.get(dcfg.position)
                        if s:
                            self.master.select_reference_clock(s)
                            logger.info(f"Selected slave {dcfg.position} as DC reference clock (first DC-enabled)")
                            break

            initial_time_ns = int(time.time() * 1_000_000_000)
            try:
                self.master.set_application_time(initial_time_ns)
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

            self.master.activate()
            logger.info("Master activated")
            self._activated_mono_ns = time.monotonic_ns()

            for dcfg in self.cfg.slaves:
                self.slave_in_op[dcfg.position] = False
                self.last_al_state[dcfg.position] = None
                self._pp_pulse_active[dcfg.position] = False
                self._pp_pulse_start_ns[dcfg.position] = None
                self._pp_pulse_pending[dcfg.position] = False
                self.enable_last_action_ns[dcfg.position] = 0
                self.enable_step[dcfg.position] = 0

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
        # Store intent; cyclic writer will realize it via PDO or SDO
        if cmd.type == CommandType.SET_VELOCITY_MODE:
            self.last_mode_cmd[cmd.target_id] = MODE_PV
        elif cmd.type == CommandType.SET_POSITION_MODE:
            self.last_mode_cmd[cmd.target_id] = MODE_PP
        elif cmd.type == CommandType.SET_CSP_MODE:
            self.last_mode_cmd[cmd.target_id] = MODE_CSP
        elif cmd.type == CommandType.SET_TORQUE_MODE:
            self.last_mode_cmd[cmd.target_id] = MODE_PT
        elif cmd.type == CommandType.SET_VELOCITY:
            self.last_velocity_cmd[cmd.target_id] = float(cmd.value or 0.0)
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
            self.last_torque_cmd[cmd.target_id] = float(cmd.value or 0.0)
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
        elif cmd.type == CommandType.DISABLE_PROBE:
            # Clear probe function on device (will be applied once in cyclic)
            self.last_probe_arm[cmd.target_id] = 0
        elif cmd.type == CommandType.ENABLE_DRIVE:
            self._manual_disable[cmd.target_id] = False
            self._enable_requested[cmd.target_id] = True
        elif cmd.type == CommandType.DISABLE_DRIVE:
            self._manual_disable[cmd.target_id] = True
            self._enable_requested[cmd.target_id] = False
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
                self.master.sdo_download(cmd.target_id, index, subindex, cmd.value)
            except Exception as e:
                logger.error(f"SDO write failed: slave={cmd.target_id} 0x{index:04X}:{subindex}: {e}")

    def _auto_enable_drives(self):
        """
        Automatic CiA 402 state machine - enables drives through state transitions.
        Called every cycle to monitor and transition drives to Operation Enabled.
        Only runs AFTER slave has reached OP state.
        """
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
                # FAULT state - send fault reset
                step = self.enable_step.get(slave_pos, 0)
                if step < 10:  # Limit fault reset attempts
                    # Set FAULT_RESET controlword
                    self.desired_controlword[slave_pos] = 0x0080
                    self.enable_step[slave_pos] = step + 1
                    self.enable_last_action_ns[slave_pos] = now_ns
                    if step == 0:  # Log once
                        logger.info(f"Slave {slave_pos}: FAULT (0x{statusword:04X}) → sending FAULT_RESET")
                continue  # Move to next slave
            
            # Determine current state using CiA 402 standard bit patterns
            state_bits = statusword & 0x006F
            
            # Operation Enabled: xxxx xxxx x01x 0111
            if state_bits == 0x0027:
                # Already enabled - set controlword to maintain enabled state
                self.desired_controlword[slave_pos] = 0x000F  # ENABLE_OPERATION
                if not self.drive_enabled.get(slave_pos, False):
                    self.drive_enabled[slave_pos] = True
                    logger.info(f"✓ Slave {slave_pos}: ENABLED (0x{statusword:04X})")
                    self.enable_step[slave_pos] = 0
                self.enable_last_action_ns[slave_pos] = now_ns
                continue  # Move to next slave
            
            # Switched On: xxxx xxxx x01x 0011
            elif state_bits == 0x0023:
                # Send ENABLE_OPERATION (0x000F)
                self.desired_controlword[slave_pos] = 0x000F
                self.enable_last_action_ns[slave_pos] = now_ns
                logger.info(f"Slave {slave_pos}: SWITCHED_ON (0x{statusword:04X}) → ENABLE_OPERATION")
            
            # Ready to Switch On: xxxx xxxx x00x 0001
            elif state_bits == 0x0021:
                # Send SWITCH_ON (0x0007)
                self.desired_controlword[slave_pos] = 0x0007
                self.enable_last_action_ns[slave_pos] = now_ns
                logger.info(f"Slave {slave_pos}: READY_TO_SWITCH_ON (0x{statusword:04X}) → SWITCH_ON")
            
            # Switch On Disabled: xxxx xxxx x1xx 0000
            elif (statusword & 0x004F) == 0x0040:
                # Send SHUTDOWN (0x0006)
                self.desired_controlword[slave_pos] = 0x0006
                self.enable_last_action_ns[slave_pos] = now_ns
                logger.info(f"Slave {slave_pos}: SWITCH_ON_DISABLED (0x{statusword:04X}) → SHUTDOWN")
            else:
                # Unknown state - log for debugging
                if self.cycle_count % 200 == 0:  # Log occasionally
                    logger.warning(f"Slave {slave_pos}: Unknown state - statusword 0x{statusword:04X}, state_bits 0x{state_bits:04X}")
    
    def _publish_status(self):
        status = NetworkStatus(drives={})
        status.timestamp_ns = int(time.time() * 1_000_000_000)
        status.cycle_time_ms_config = self.cfg.cycle_time_ms
        status.sdo_only = self.cfg.sdo_only
        for slave_pos, entries in self.offsets.items():
            drive = {}
            drive['in_op'] = self.slave_in_op.get(slave_pos, False)

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
                # Some devices map Probe-2 position at 0x60BB instead of 0x60BC.
                raw = self.master.read_domain(self.domain, entries[(PROBE_POS2_ALT_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                drive['probe_pos2'] = int.from_bytes(raw, 'little', signed=True)
            if (DIGITAL_INPUTS_INDEX, 0) in entries:
                raw = self.master.read_domain(self.domain, entries[(DIGITAL_INPUTS_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                drive['digital_inputs'] = int.from_bytes(raw, 'little')
            # Publish static feature capabilities (from XML-derived supports)
            drive['features'] = self.features.get(slave_pos, {})
            # Publish Ruckig planner state
            if self._ruckig_planner is not None:
                r = self._ruckig_planner.describe(slave_pos)
                r["last_error"] = self._ruckig_last_error.get(slave_pos)
                drive["ruckig"] = r
            # Publish PDO map health (presence)
            pdo_health = {}
            for key_idx in [MODES_OP_INDEX, MODES_OP_DISPLAY_INDEX, CW_INDEX, SW_INDEX, TARGET_VELOCITY_INDEX, TARGET_POSITION_INDEX]:
                state = 'missing'
                if (key_idx, 0) in entries:
                    state = 'pdo'
                pdo_health[f"0x{key_idx:04X}:0"] = state
            drive['pdo_health'] = pdo_health
            status.drives[slave_pos] = drive
        try:
            self.status_q.put_nowait(status)
            if self._status_publish_count < 3:
                self._status_publish_count += 1
                sys.stderr.write(f"[EC] status published cycle={self.cycle_count} drives={list(status.drives.keys())} qsize={self.status_q.qsize()}\n")
                sys.stderr.flush()
        except queue.Full:
            pass
        except Exception as e:
            sys.stderr.write(f"[EC] status_q.put FAILED: {e}\n")
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

            mode = self.last_mode_cmd.get(slave_pos)
            if mode is not None:
                if (MODES_OP_INDEX, 0) in entries:
                    self.master.write_domain(self.domain, entries[(MODES_OP_INDEX, 0)], bytes([mode]))
                else:
                    prev_sdo_mode = self._last_mode_sdo.get(slave_pos)
                    if mode != prev_sdo_mode:
                        logger.warning(f"Slave {slave_pos}: {hex(MODES_OP_INDEX)} not in PDO; writing mode={mode} via SDO")
                        try:
                            self.master.sdo_download(slave_pos, MODES_OP_INDEX, 0, bytes([mode]))
                            self._last_mode_sdo[slave_pos] = mode
                        except Exception as e:
                            logger.error(f"SDO write {hex(MODES_OP_INDEX)} failed: {e}")

            # Safety: do not write motion targets until drive is enabled.
            # Some drives fault (e.g., following error / excessive reference) if targets stream before enable.
            motion_ok = self.drive_enabled.get(slave_pos, False) and not self._manual_disable.get(slave_pos, False)

            # Maintain target velocity (0x60FF) for PV
            vel = self.last_velocity_cmd.get(slave_pos)
            if motion_ok and vel is not None:
                v = int(vel)
                if (TARGET_VELOCITY_INDEX, 0) in entries:
                    self.master.write_domain(self.domain, entries[(TARGET_VELOCITY_INDEX, 0)], v.to_bytes(4, byteorder='little', signed=True))
                else:
                    prev_sdo_vel = self._last_velocity_sdo.get(slave_pos)
                    if v != prev_sdo_vel:
                        logger.info(f"Slave {slave_pos}: {hex(TARGET_VELOCITY_INDEX)} not in PDO; writing vel={v} via SDO")
                        try:
                            self.master.sdo_download(slave_pos, TARGET_VELOCITY_INDEX, 0, v.to_bytes(4, 'little', signed=True))
                            self._last_velocity_sdo[slave_pos] = v
                        except Exception as e:
                            logger.error(f"SDO write {hex(TARGET_VELOCITY_INDEX)} failed: {e}")
                # PV pulse is "edge-ish": if we asserted bit4 at least once, clear pending so it becomes
                # a bounded pulse (active clears on ack/timeout). This prevents indefinite re-triggering.
                if self._pv_pulse_pending.get(slave_pos, False) and self._pv_pulse_active.get(slave_pos, False):
                    self._pv_pulse_pending[slave_pos] = False

            torque = self.last_torque_cmd.get(slave_pos)
            if motion_ok and torque is not None:
                t = int(torque)
                if (TARGET_TORQUE_INDEX, 0) in entries:
                    self.master.write_domain(self.domain, entries[(TARGET_TORQUE_INDEX, 0)], t.to_bytes(2, byteorder='little', signed=True))
                else:
                    prev_sdo_torque = self._last_torque_sdo.get(slave_pos)
                    if t != prev_sdo_torque:
                        logger.info(f"Slave {slave_pos}: {hex(TARGET_TORQUE_INDEX)} not in PDO; writing torque={t} via SDO")
                        try:
                            self.master.sdo_download(slave_pos, TARGET_TORQUE_INDEX, 0, t.to_bytes(2, 'little', signed=True))
                            self._last_torque_sdo[slave_pos] = t
                        except Exception as e:
                            logger.error(f"SDO write {hex(TARGET_TORQUE_INDEX)} failed: {e}")

            # Maintain target position (0x607A)
            # - PP/HM: write the last commanded target (latch with bit4 pulse below)
            # - CSP: stream the buffered position every cycle (atomic swap)
            pos = None
            mode_eff = mode if mode is not None else self._default_mode.get(slave_pos)
            if mode_eff == MODE_CSP:
                # Atomic swap: update current target at cycle boundary if a new one arrived
                if slave_pos in self._csp_target_next:
                    self._csp_target_cur[slave_pos] = self._csp_target_next.get(slave_pos)
                pos = self._csp_target_cur.get(slave_pos)
            else:
                pos = self.last_position_cmd.get(slave_pos)

            if motion_ok and pos is not None:
                p = int(pos)
                if (TARGET_POSITION_INDEX, 0) in entries:
                    self.master.write_domain(self.domain, entries[(TARGET_POSITION_INDEX, 0)], p.to_bytes(4, byteorder='little', signed=True))
                else:
                    sdo_done = self.warned_missing_pdo.setdefault(slave_pos, set())
                    key = (TARGET_POSITION_INDEX, 0)
                    if key not in sdo_done:
                        logger.warning(f"Slave {slave_pos}: {hex(TARGET_POSITION_INDEX)} not in PDO; writing via SDO (once)")
                        try:
                            self.master.sdo_download(slave_pos, TARGET_POSITION_INDEX, 0, p.to_bytes(4, 'little', signed=True))
                        except Exception as e:
                            logger.error(f"SDO write {hex(TARGET_POSITION_INDEX)} failed: {e}")
                        sdo_done.add(key)

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

            # Touch probe arming maintenance (write-once behavior)
            probe_val = self.last_probe_arm.get(slave_pos)
            if probe_val is not None:
                if (PROBE_FUNCTION_INDEX, 0) in entries:
                    self.master.write_domain(self.domain, entries[(PROBE_FUNCTION_INDEX, 0)], probe_val.to_bytes(2, 'little'))
                else:
                    warned = self.warned_missing_pdo.setdefault(slave_pos, set())
                    key = (PROBE_FUNCTION_INDEX, 0)
                    if key not in warned:
                        logger.warning(f"Slave {slave_pos}: {hex(PROBE_FUNCTION_INDEX)} not in PDO; writing via SDO")
                        warned.add(key)
                    try:
                        self.master.sdo_download(slave_pos, PROBE_FUNCTION_INDEX, 0, probe_val.to_bytes(2, 'little'))
                    except Exception as e:
                        logger.error(f"SDO write {hex(PROBE_FUNCTION_INDEX)} failed: {e}")
                # Clear after one write
                self.last_probe_arm[slave_pos] = None

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
            actual_velocity = float(int.from_bytes(raw_v, "little", signed=True))

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

                try:
                    if req.get("kind") == "position":
                        self._ruckig_planner.start_position(
                            pos,
                            actual_position=actual_position,
                            actual_velocity=actual_velocity,
                            target_position=int(req.get("target")),
                            cfg=cfg,
                            dt_s_fallback=dt_s_fallback,
                            overrides=overrides,
                        )
                    else:
                        self._ruckig_planner.start_velocity(
                            pos,
                            actual_position=actual_position,
                            actual_velocity=actual_velocity,
                            target_velocity=float(req.get("target")),
                            cfg=cfg,
                            dt_s_fallback=dt_s_fallback,
                            overrides=overrides,
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
            step = self._ruckig_planner.step(pos, actual_position=actual_position, actual_velocity=actual_velocity)
            if step is not None:
                self._csp_target_next[pos] = int(step.position)

    def run(self):
        try:
            self._run_inner()
        except SystemExit:
            raise
        except Exception:
            sys.stderr.write("ETHERCAT PROCESS FATAL:\n")
            traceback.print_exc(file=sys.stderr)
            sys.stderr.flush()
            raise

    def _run_inner(self):
        self._install_signal_handlers()

        if self.cfg.rt_priority is not None:
            SCHED_FIFO = 1
            param = os.sched_param(self.cfg.rt_priority)
            os.sched_setscheduler(0, SCHED_FIFO, param)

        if self.cfg.cpu_core is not None:
            os.sched_setaffinity(0, {self.cfg.cpu_core})

        ok = self._setup()
        if not ok:
            self._teardown()
            raise SystemExit(1)

        cycle_ns = int(self.cfg.cycle_time_ms * 1_000_000)
        if cycle_ns <= 0:
            raise ValueError("cycle_time_ms must be > 0")

        mono_base = time.monotonic_ns()
        wall_base = time.time_ns()
        next_cycle_mono = mono_base

        last_status = 0.0
        try:
            while not self.stop_event.is_set():
                self.cycle_count += 1
                next_cycle_mono += cycle_ns
                scheduled_wall_ns = wall_base + (next_cycle_mono - mono_base)

                for _ in range(16):
                    try:
                        cmd = self.cmd_q.get_nowait()
                        self._handle_command(cmd)
                    except queue.Empty:
                        break

                if not self.cfg.sdo_only and self.domain is not None:
                    self.master.receive()
                    self.master.process_domain(self.domain)

                    wc, wc_state = self.master.domain_state(self.domain)
                    all_op_now = (wc_state == 2)
                    for pos in self.slave_in_op:
                        was_op = self.slave_in_op[pos]
                        if all_op_now and not was_op:
                            self.slave_in_op[pos] = True
                            logger.info(f"Slave {pos} entered OP (domain WC={wc}, state=complete)")
                        elif not all_op_now and was_op:
                            self.slave_in_op[pos] = False
                            self.drive_enabled[pos] = False
                            self.desired_controlword.pop(pos, None)
                            self._pp_pulse_active[pos] = False
                            self._pp_pulse_start_ns[pos] = None
                            self._pp_pulse_pending[pos] = False
                            logger.warning(f"Slave {pos} left OP (domain WC={wc}, state={wc_state})")

                    if self._activated_mono_ns is not None:
                        elapsed_s = (time.monotonic_ns() - self._activated_mono_ns) / 1_000_000_000.0
                        if elapsed_s >= float(self.cfg.op_timeout_s):
                            not_op = [p for p in self.slave_in_op.keys() if not self.slave_in_op.get(p, False)]
                            if not_op:
                                raise RuntimeError(f"OP timeout after {self.cfg.op_timeout_s}s. Not in OP: {not_op}. Domain WC={wc}, state={wc_state}")

                    self._auto_enable_drives()
                    self._update_ruckig()
                    self._cyclic_write()

                    try:
                        self.master.set_application_time(int(scheduled_wall_ns))
                    except Exception:
                        pass
                    self.master.sync_reference_clock()
                    self.master.sync_slave_clocks()

                    self.master.queue_domain(self.domain)
                    self.master.send()

                now = time.time()
                if now - last_status > 0.05:
                    self._publish_status()
                    last_status = now

                sleep_ns = next_cycle_mono - time.monotonic_ns()
                if sleep_ns <= 0:
                    mono_base = time.monotonic_ns()
                    wall_base = time.time_ns()
                    next_cycle_mono = mono_base + cycle_ns
                    sleep_ns = cycle_ns
                time.sleep(sleep_ns / 1_000_000_000.0)
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

    def start(self):
        if self._proc and self._proc.is_alive():
            return
        self._stop_event.clear()
        target = EtherCATProcess(self.cfg, self._cmd_q, self._status_q, self._stop_event)
        self._proc = mp.Process(target=target.run, daemon=False)
        self._proc.start()

    def is_alive(self) -> bool:
        return bool(self._proc and self._proc.is_alive())

    def exitcode(self) -> Optional[int]:
        if not self._proc:
            return None
        return self._proc.exitcode

    def stop(self):
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

    # Application API
    def send_command(self, cmd: Command):
        try:
            self._cmd_q.put_nowait(cmd)
            return True
        except queue.Full:
            return False

    def get_latest_status(self) -> Optional[NetworkStatus]:
        latest = None
        while True:
            try:
                latest = self._status_q.get_nowait()
            except queue.Empty:
                break
        return latest


