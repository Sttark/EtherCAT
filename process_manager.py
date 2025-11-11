import logging
import multiprocessing as mp
import queue
import time
from typing import Any, Dict, List, Optional, Tuple

from .commands import Command, CommandType
from .status_model import NetworkStatus
from .constants import (
    CW_INDEX, SW_INDEX,
    MODES_OP_INDEX, MODES_OP_DISPLAY_INDEX,
    TARGET_POSITION_INDEX, TARGET_VELOCITY_INDEX,
    POSITION_ACTUAL_INDEX, VELOCITY_ACTUAL_INDEX,
    PROBE_FUNCTION_INDEX, PROBE_STATUS_INDEX, PROBE_POS1_INDEX, PROBE_POS2_INDEX, DIGITAL_INPUTS_INDEX,
    MODE_PP, MODE_PV, MODE_CSP,
    CW_BIT_NEW_SET_POINT, CW_ENABLE_OP_SIMPLIFIED,
    PROBE_FUNC_ENABLE_PROBE1, PROBE_FUNC_PROBE1_POS_EDGE, PROBE_FUNC_PROBE1_NEG_EDGE,
)
from .config_schema import EthercatNetworkConfig, DriveConfig
from .master_adapter import Master as IGHMasterAdapter
from .xml_decoder import parse_esi_features


logger = logging.getLogger(__name__)


class EtherCATProcess:
    """
    Isolated process that owns the EtherCAT master and cyclic loop.
    Communication via command/status queues. Non-blocking, resilient runtime.
    """

    def __init__(self, cfg: EthercatNetworkConfig, cmd_q: mp.Queue, status_q: mp.Queue):
        self.cfg = cfg
        self.cmd_q = cmd_q
        self.status_q = status_q
        self.adapter = IGHMasterAdapter(master_index=cfg.master_index)
        self.domain = None
        self.slave_handles: Dict[int, Any] = {}
        self.offsets: Dict[int, Dict[Tuple[int, int], int]] = {}
        self.features: Dict[int, Dict[str, Any]] = {}
        self.last_mode_cmd: Dict[int, Optional[int]] = {}
        self.last_velocity_cmd: Dict[int, Optional[float]] = {}
        self.last_position_cmd: Dict[int, Optional[float]] = {}
        self.warned_missing_pdo: Dict[int, set] = {}
        self.last_probe_arm: Dict[int, Optional[int]] = {}
        self.pdo_maps: Dict[int, Dict[str, Dict[int, list]]] = {}

    def _setup(self) -> bool:
        # Open/request the master with or without PDO based on config
        if not self.adapter.request(sdo_only=self.cfg.sdo_only):
            logger.error("Failed to request master")
            return False

        # Create domain only if PDO is desired
        if not self.cfg.sdo_only:
            self.domain = self.adapter.create_domain()

        # Configure each slave
        for dcfg in self.cfg.slaves:
            if dcfg.vendor_id is None or dcfg.product_code is None or not dcfg.xml:
                logger.error(f"Slave {dcfg.position} missing vendor_id/product_code/xml")
                return False

            s = self.adapter.config_slave(dcfg.alias, dcfg.position, dcfg.vendor_id, dcfg.product_code)

            # Build PDO config from XML + overrides
            features = parse_esi_features(dcfg.xml.xml_file)
            rx_pdos = dcfg.pdo.rx_pdos if dcfg.pdo and dcfg.pdo.rx_pdos else features['rx_pdos']
            tx_pdos = dcfg.pdo.tx_pdos if dcfg.pdo and dcfg.pdo.tx_pdos else features['tx_pdos']

            # Build per-PDO entry maps (by-the-book for ecrt_slave_config_pdos)
            rx_pdo_map = features.get('pdo_map_rx', {})
            tx_pdo_map = features.get('pdo_map_tx', {})
            if dcfg.pdo and dcfg.pdo.custom_pdo_config:
                custom = dcfg.pdo.custom_pdo_config
                # Assign all custom RX entries to the first RX PDO (common single-PDO pattern)
                if rx_pdos and custom.get('rx_entries'):
                    rx_pdo_map = {rx_pdos[0]: custom['rx_entries']}
                if tx_pdos and custom.get('tx_entries'):
                    tx_pdo_map = {tx_pdos[0]: custom['tx_entries']}

            sync_configs = []
            if rx_pdos:
                sync_configs.append((2, 0, [(p, rx_pdo_map.get(p, [])) for p in rx_pdos]))
            if tx_pdos:
                sync_configs.append((3, 1, [(p, tx_pdo_map.get(p, [])) for p in tx_pdos]))

            if not self.cfg.sdo_only:
                self.adapter.configure_slave_pdos(s, sync_configs)

                # Bulk register PDO entries to obtain domain offsets (flatten per-PDO maps)
                register_pairs = []
                for p, entries in rx_pdo_map.items():
                    register_pairs.extend([(idx, sub) for (idx, sub, _bits) in entries])
                for p, entries in tx_pdo_map.items():
                    register_pairs.extend([(idx, sub) for (idx, sub, _bits) in entries])
                register_list = list(set(register_pairs))
                offsets = self.adapter.register_pdo_entry_list(
                    self.domain, dcfg.alias, dcfg.position, dcfg.vendor_id, dcfg.product_code, register_list
                )
                self.offsets[dcfg.position] = offsets
                self.pdo_maps[dcfg.position] = {
                    'rx': rx_pdo_map,
                    'tx': tx_pdo_map,
                }

            # Track features for this slave
            self.features[dcfg.position] = features['supports']

        if not self.cfg.sdo_only:
            self.adapter.activate()

        return True

    def _teardown(self):
        try:
            self.adapter.deactivate()
        except Exception:
            pass
        try:
            self.adapter.release()
        except Exception:
            pass

    def _handle_command(self, cmd: Command):
        # Store intent; cyclic writer will realize it via PDO or SDO
        if cmd.type == CommandType.SET_VELOCITY_MODE:
            self.last_mode_cmd[cmd.target_id] = MODE_PV
        elif cmd.type == CommandType.SET_POSITION_MODE:
            self.last_mode_cmd[cmd.target_id] = MODE_PP
        elif cmd.type == CommandType.SET_CSP_MODE:
            self.last_mode_cmd[cmd.target_id] = MODE_CSP
        elif cmd.type == CommandType.SET_VELOCITY:
            self.last_velocity_cmd[cmd.target_id] = float(cmd.value or 0.0)
        elif cmd.type in (CommandType.SET_POSITION, CommandType.SET_POSITION_CSP):
            self.last_position_cmd[cmd.target_id] = float(cmd.value or 0.0)
        elif cmd.type == CommandType.START_HOMING:
            # Switch to HM mode and issue one-cycle start
            self.last_mode_cmd[cmd.target_id] = MODE_HM
            # Reuse position strobe to trigger new-set-point in HM
            self.last_position_cmd[cmd.target_id] = float(0)
        elif cmd.type == CommandType.ARM_PROBE:
            probe_value = cmd.params.get('probe_function') if cmd.params else None
            if probe_value is not None:
                self.last_probe_arm[cmd.target_id] = int(probe_value)
        elif cmd.type == CommandType.STOP_MOTION:
            self.last_velocity_cmd[cmd.target_id] = 0.0
        elif cmd.type == CommandType.DISABLE_PROBE:
            # Clear probe function on device (will be applied once in cyclic)
            self.last_probe_arm[cmd.target_id] = 0
        elif cmd.type == CommandType.ENABLE_DRIVE:
            # Controlword bits handled in cyclic writer
            pass
        elif cmd.type == CommandType.DISABLE_DRIVE:
            # Controlword bits handled in cyclic writer
            pass
        elif cmd.type == CommandType.READ_SDO:
            # Future: perform immediate SDO read and publish asynchronously
            pass
        elif cmd.type == CommandType.WRITE_SDO:
            # Future: perform immediate SDO write
            pass

    def _publish_status(self):
        status = NetworkStatus(drives={})
        status.timestamp_ns = int(time.time() * 1_000_000_000)
        status.cycle_time_ms_config = self.cfg.cycle_time_ms
        status.sdo_only = self.cfg.sdo_only
        # Example: read back status fields when offsets known
        for slave_pos, entries in self.offsets.items():
            drive = {}
            if (SW_INDEX, 0) in entries:
                raw = self.adapter.read_domain(self.domain, entries[(SW_INDEX, 0)], 2) or b"\x00\x00"
                drive['statusword'] = int.from_bytes(raw, 'little')
            if (MODES_OP_DISPLAY_INDEX, 0) in entries:
                raw = self.adapter.read_domain(self.domain, entries[(MODES_OP_DISPLAY_INDEX, 0)], 1) or b"\x00"
                drive['mode_display'] = raw[0]
            if (POSITION_ACTUAL_INDEX, 0) in entries:
                raw = self.adapter.read_domain(self.domain, entries[(POSITION_ACTUAL_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                drive['position_actual'] = int.from_bytes(raw, 'little', signed=True)
            if (VELOCITY_ACTUAL_INDEX, 0) in entries:
                raw = self.adapter.read_domain(self.domain, entries[(VELOCITY_ACTUAL_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                drive['velocity_actual'] = int.from_bytes(raw, 'little', signed=True)

            # Probe status
            if (PROBE_STATUS_INDEX, 0) in entries:
                raw = self.adapter.read_domain(self.domain, entries[(PROBE_STATUS_INDEX, 0)], 2) or b"\x00\x00"
                drive['probe_active'] = bool(int.from_bytes(raw, 'little') & 0x0001)
                drive['probe_enabled'] = True
            if (PROBE_POS1_INDEX, 0) in entries:
                raw = self.adapter.read_domain(self.domain, entries[(PROBE_POS1_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                drive['probe_pos1'] = int.from_bytes(raw, 'little', signed=True)
            if (PROBE_POS2_INDEX, 0) in entries:
                raw = self.adapter.read_domain(self.domain, entries[(PROBE_POS2_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                drive['probe_pos2'] = int.from_bytes(raw, 'little', signed=True)
            if (DIGITAL_INPUTS_INDEX, 0) in entries:
                raw = self.adapter.read_domain(self.domain, entries[(DIGITAL_INPUTS_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
                drive['digital_inputs'] = int.from_bytes(raw, 'little')
            # Publish static feature capabilities (from XML-derived supports)
            drive['features'] = self.features.get(slave_pos, {})
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
        except queue.Full:
            pass

    def _cyclic_write(self):
        """
        Apply desired intents to PDO or SDO each cycle. Maintains mode (0x6060),
        controlword (0x6040), targets (0x607A, 0x60FF), and device features like
        probe function (0x60B8) if mapped.
        """
        if self.cfg.sdo_only or self.domain is None:
            return
        # Example offsets lookup (to be set during registration step):
        # offsets[slave_pos][(index, subindex)] -> byte_offset
        for slave_pos, entries in self.offsets.items():
            # Maintain operation mode (0x6060) if mapped and requested
            mode = self.last_mode_cmd.get(slave_pos)
            if mode is not None:
                if (MODES_OP_INDEX, 0) in entries:
                    self.adapter.write_domain(self.domain, entries[(MODES_OP_INDEX, 0)], bytes([mode]))
                else:
                    # Fallback to SDO and warn once
                    warned = self.warned_missing_pdo.setdefault(slave_pos, set())
                    key = (MODES_OP_INDEX, 0)
                    if key not in warned:
                        logger.warning(f"Slave {slave_pos}: {hex(MODES_OP_INDEX)} not in PDO; writing via SDO")
                        warned.add(key)
                    try:
                        self.adapter.sdo_download(slave_pos, MODES_OP_INDEX, 0, bytes([mode]))
                    except Exception as e:
                        logger.error(f"SDO write {hex(MODES_OP_INDEX)} failed: {e}")

            # Maintain target velocity (0x60FF) for PV
            vel = self.last_velocity_cmd.get(slave_pos)
            if vel is not None:
                v = int(vel)
                if (TARGET_VELOCITY_INDEX, 0) in entries:
                    self.adapter.write_domain(self.domain, entries[(TARGET_VELOCITY_INDEX, 0)], v.to_bytes(4, byteorder='little', signed=True))
                else:
                    warned = self.warned_missing_pdo.setdefault(slave_pos, set())
                    key = (TARGET_VELOCITY_INDEX, 0)
                    if key not in warned:
                        logger.warning(f"Slave {slave_pos}: {hex(TARGET_VELOCITY_INDEX)} not in PDO; writing via SDO")
                        warned.add(key)
                    try:
                        self.adapter.sdo_download(slave_pos, TARGET_VELOCITY_INDEX, 0, v.to_bytes(4, 'little', signed=True))
                    except Exception as e:
                        logger.error(f"SDO write {hex(TARGET_VELOCITY_INDEX)} failed: {e}")

            # Maintain target position (0x607A) for PP and CSP
            pos = self.last_position_cmd.get(slave_pos)
            if pos is not None:
                p = int(pos)
                if (TARGET_POSITION_INDEX, 0) in entries:
                    self.adapter.write_domain(self.domain, entries[(TARGET_POSITION_INDEX, 0)], p.to_bytes(4, byteorder='little', signed=True))
                else:
                    warned = self.warned_missing_pdo.setdefault(slave_pos, set())
                    key = (TARGET_POSITION_INDEX, 0)
                    if key not in warned:
                        logger.warning(f"Slave {slave_pos}: {hex(TARGET_POSITION_INDEX)} not in PDO; writing via SDO")
                        warned.add(key)
                    try:
                        self.adapter.sdo_download(slave_pos, TARGET_POSITION_INDEX, 0, p.to_bytes(4, 'little', signed=True))
                    except Exception as e:
                        logger.error(f"SDO write {hex(TARGET_POSITION_INDEX)} failed: {e}")

            # Controlword bit maintenance (0x6040):
            # - PP: pulse bit 4 (new set-point) when position target changes
            # - PV: ensure enable operation and appropriate state bits
            if (CW_INDEX, 0) in entries:
                cw_offset = entries[(CW_INDEX, 0)]
                cw = 0
                cw |= CW_ENABLE_OP_SIMPLIFIED
                if pos is not None:
                    cw |= (1 << CW_BIT_NEW_SET_POINT)
                self.adapter.write_domain(self.domain, cw_offset, cw.to_bytes(2, byteorder='little'))

            # Touch probe arming maintenance (write-once behavior)
            probe_val = self.last_probe_arm.get(slave_pos)
            if probe_val is not None:
                if (PROBE_FUNCTION_INDEX, 0) in entries:
                    self.adapter.write_domain(self.domain, entries[(PROBE_FUNCTION_INDEX, 0)], probe_val.to_bytes(2, 'little'))
                else:
                    warned = self.warned_missing_pdo.setdefault(slave_pos, set())
                    key = (PROBE_FUNCTION_INDEX, 0)
                    if key not in warned:
                        logger.warning(f"Slave {slave_pos}: {hex(PROBE_FUNCTION_INDEX)} not in PDO; writing via SDO")
                        warned.add(key)
                    try:
                        self.adapter.sdo_download(slave_pos, PROBE_FUNCTION_INDEX, 0, probe_val.to_bytes(2, 'little'))
                    except Exception as e:
                        logger.error(f"SDO write {hex(PROBE_FUNCTION_INDEX)} failed: {e}")
                # Clear after one write
                self.last_probe_arm[slave_pos] = None

    def run(self):
        ok = self._setup()
        if not ok:
            self._teardown()
            return

        cycle_time_s = self.cfg.cycle_time_ms / 1000.0
        last_status = 0.0
        last_cycle_start = time.time()
        try:
            while True:
                # Pump commands
                for _ in range(16):
                    try:
                        cmd = self.cmd_q.get_nowait()
                        self._handle_command(cmd)
                    except queue.Empty:
                        break

                # Cyclic PDO exchange
                if not self.cfg.sdo_only and self.domain is not None:
                    # Maintain application/DC time if needed (optional)
                    try:
                        self.adapter.set_application_time(int(time.time() * 1_000_000_000))
                    except Exception:
                        pass
                    self.adapter.receive()
                    self.adapter.process_domain(self.domain)
                    # Apply maintenance writes
                    self._cyclic_write()
                    self.adapter.queue_domain(self.domain)
                    self.adapter.send()

                # Periodic status publish (decouple from cycle rate)
                now = time.time()
                if now - last_status > 0.05:
                    # compute last cycle time (rough)
                    last_cycle_time_us = int((now - last_cycle_start) * 1_000_000)
                    # Set temporary global last_cycle_time_us via NetworkStatus on publish call
                    # (lightweight: attach to object just before publishing)
                    self._publish_status()
                    last_status = now
                last_cycle_start = now

                time.sleep(cycle_time_s)
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
        self._proc: Optional[mp.Process] = None

    def start(self):
        if self._proc and self._proc.is_alive():
            return
        target = EtherCATProcess(self.cfg, self._cmd_q, self._status_q)
        self._proc = mp.Process(target=target.run, daemon=True)
        self._proc.start()

    def stop(self):
        if self._proc and self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout=2.0)
            self._proc = None

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


