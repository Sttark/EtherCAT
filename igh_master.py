#!/usr/bin/env python3
"""
IgH EtherCAT Master (direct ctypes binding).

This module contains ONE implementation only:
- `Master`: direct wrapper over `libethercat.so.1` (IgH userspace API)
- `SlaveConfig`: thin handle wrapper returned by `Master.config_slave()`

No adapter layer. No dependency on SttarkStandardLibrary / pyethercat.
"""

import ctypes
import logging
import shutil
import subprocess
import time
from typing import Optional, Tuple, Dict


logger = logging.getLogger(__name__)


class MasterException(Exception):
    pass


class SDOException(Exception):
    pass


# Load the EtherCAT library
try:
    _libec = ctypes.CDLL("libethercat.so.1")
except OSError as e:
    logger.error(f"Failed to load libethercat.so.1: {e}")
    _libec = None


# Define C types for EtherCAT structures
class ec_master_t(ctypes.Structure):
    """Opaque master handle."""
    pass


class ec_slave_config_t(ctypes.Structure):
    """Opaque slave configuration handle."""
    pass


class ec_domain_t(ctypes.Structure):
    """Opaque domain handle."""
    pass


class ec_sdo_request_t(ctypes.Structure):
    """Opaque SDO request handle."""
    pass


class ec_pdo_entry_reg_t(ctypes.Structure):
    """PDO entry registration structure."""
    _fields_ = [
        ("alias", ctypes.c_uint16),
        ("position", ctypes.c_uint16),
        ("vendor_id", ctypes.c_uint32),
        ("product_code", ctypes.c_uint32),
        ("index", ctypes.c_uint16),
        ("subindex", ctypes.c_uint8),
        ("offset", ctypes.POINTER(ctypes.c_uint)),
        ("bit_position", ctypes.POINTER(ctypes.c_uint)),
    ]


class ec_pdo_entry_info_t(ctypes.Structure):
    """PDO entry configuration structure."""
    _fields_ = [
        ("index", ctypes.c_uint16),
        ("subindex", ctypes.c_uint8),
        ("bit_length", ctypes.c_uint8),
    ]


class ec_pdo_info_t(ctypes.Structure):
    """PDO configuration structure."""
    _fields_ = [
        ("index", ctypes.c_uint16),
        ("n_entries", ctypes.c_uint),
        ("entries", ctypes.POINTER(ec_pdo_entry_info_t)),
    ]


class ec_sync_info_t(ctypes.Structure):
    """Sync manager configuration structure."""
    _fields_ = [
        ("index", ctypes.c_uint8),
        ("dir", ctypes.c_int),
        ("n_pdos", ctypes.c_uint),
        ("pdos", ctypes.POINTER(ec_pdo_info_t)),
        ("watchdog_mode", ctypes.c_int),
    ]


class ec_domain_state_t(ctypes.Structure):
    _fields_ = [
        ("working_counter", ctypes.c_uint),
        ("wc_state", ctypes.c_uint),
    ]


class ec_master_info_t(ctypes.Structure):
    _fields_ = [
        ("slave_count", ctypes.c_uint),
        ("link_up", ctypes.c_int),
        ("scan_busy", ctypes.c_int),
        ("app_time", ctypes.c_uint64),
    ]


class ec_slave_info_t(ctypes.Structure):
    _fields_ = [
        ("position", ctypes.c_uint16),
        ("vendor_id", ctypes.c_uint32),
        ("product_code", ctypes.c_uint32),
        ("revision_number", ctypes.c_uint32),
        ("serial_number", ctypes.c_uint32),
        ("alias", ctypes.c_uint16),
        ("current_on_ebus", ctypes.c_int16),
        ("al_state", ctypes.c_uint8),
        ("error_flag", ctypes.c_uint8),
        ("sync_count", ctypes.c_uint8),
        ("sdo_count", ctypes.c_uint16),
        ("name", ctypes.c_char * 200),
    ]


# Define function prototypes if library is available
if _libec:
    _libec.ecrt_open_master.argtypes = [ctypes.c_uint]
    _libec.ecrt_open_master.restype = ctypes.POINTER(ec_master_t)

    _libec.ecrt_request_master.argtypes = [ctypes.c_uint]
    _libec.ecrt_request_master.restype = ctypes.POINTER(ec_master_t)

    _libec.ecrt_release_master.argtypes = [ctypes.POINTER(ec_master_t)]
    _libec.ecrt_release_master.restype = None

    _libec.ecrt_master_activate.argtypes = [ctypes.POINTER(ec_master_t)]
    _libec.ecrt_master_activate.restype = ctypes.c_int

    _libec.ecrt_master_deactivate.argtypes = [ctypes.POINTER(ec_master_t)]
    _libec.ecrt_master_deactivate.restype = None

    _libec.ecrt_master_application_time.argtypes = [ctypes.POINTER(ec_master_t), ctypes.c_uint64]
    _libec.ecrt_master_application_time.restype = None

    _libec.ecrt_master.argtypes = [ctypes.POINTER(ec_master_t), ctypes.POINTER(ec_master_info_t)]
    _libec.ecrt_master.restype = ctypes.c_int

    _libec.ecrt_master_get_slave.argtypes = [
        ctypes.POINTER(ec_master_t),
        ctypes.c_uint16,
        ctypes.POINTER(ec_slave_info_t)
    ]
    _libec.ecrt_master_get_slave.restype = ctypes.c_int

    _libec.ecrt_master_sdo_download.argtypes = [
        ctypes.POINTER(ec_master_t),
        ctypes.c_uint16,
        ctypes.c_uint16,
        ctypes.c_uint8,
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_uint32)
    ]
    _libec.ecrt_master_sdo_download.restype = ctypes.c_int

    _libec.ecrt_master_sdo_upload.argtypes = [
        ctypes.POINTER(ec_master_t),
        ctypes.c_uint16,
        ctypes.c_uint16,
        ctypes.c_uint8,
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_size_t),
        ctypes.POINTER(ctypes.c_uint32)
    ]
    _libec.ecrt_master_sdo_upload.restype = ctypes.c_int

    _libec.ecrt_master_send.argtypes = [ctypes.POINTER(ec_master_t)]
    _libec.ecrt_master_send.restype = None

    _libec.ecrt_master_receive.argtypes = [ctypes.POINTER(ec_master_t)]
    _libec.ecrt_master_receive.restype = None

    _libec.ecrt_master_create_domain.argtypes = [ctypes.POINTER(ec_master_t)]
    _libec.ecrt_master_create_domain.restype = ctypes.POINTER(ec_domain_t)

    _libec.ecrt_domain_data.argtypes = [ctypes.POINTER(ec_domain_t)]
    _libec.ecrt_domain_data.restype = ctypes.POINTER(ctypes.c_uint8)

    _libec.ecrt_domain_process.argtypes = [ctypes.POINTER(ec_domain_t)]
    _libec.ecrt_domain_process.restype = ctypes.c_int

    _libec.ecrt_domain_queue.argtypes = [ctypes.POINTER(ec_domain_t)]
    _libec.ecrt_domain_queue.restype = None

    _libec.ecrt_domain_state.argtypes = [
        ctypes.POINTER(ec_domain_t),
        ctypes.POINTER(ec_domain_state_t)
    ]
    _libec.ecrt_domain_state.restype = None

    _libec.ecrt_domain_reg_pdo_entry_list.argtypes = [
        ctypes.POINTER(ec_domain_t),
        ctypes.POINTER(ec_pdo_entry_reg_t)
    ]
    _libec.ecrt_domain_reg_pdo_entry_list.restype = ctypes.c_int

    _libec.ecrt_master_slave_config.argtypes = [
        ctypes.POINTER(ec_master_t),
        ctypes.c_uint16,
        ctypes.c_uint16,
        ctypes.c_uint32,
        ctypes.c_uint32
    ]
    _libec.ecrt_master_slave_config.restype = ctypes.POINTER(ec_slave_config_t)

    _libec.ecrt_slave_config_pdos.argtypes = [
        ctypes.POINTER(ec_slave_config_t),
        ctypes.c_uint,
        ctypes.POINTER(ec_sync_info_t)
    ]
    _libec.ecrt_slave_config_pdos.restype = ctypes.c_int

    _libec.ecrt_slave_config_reg_pdo_entry.argtypes = [
        ctypes.POINTER(ec_slave_config_t),
        ctypes.c_uint16,
        ctypes.c_uint8,
        ctypes.POINTER(ec_domain_t),
        ctypes.POINTER(ctypes.c_uint)
    ]
    _libec.ecrt_slave_config_reg_pdo_entry.restype = ctypes.c_int

    _libec.ecrt_slave_config_dc.argtypes = [
        ctypes.POINTER(ec_slave_config_t),
        ctypes.c_uint16,
        ctypes.c_uint32,
        ctypes.c_int32,
        ctypes.c_uint32,
        ctypes.c_int32
    ]
    # IMPORTANT: in IgH headers (ecrt.h) this returns int (0 success, <0 error).
    _libec.ecrt_slave_config_dc.restype = ctypes.c_int

    _libec.ecrt_slave_config_sdo.argtypes = [
        ctypes.POINTER(ec_slave_config_t),
        ctypes.c_uint16,
        ctypes.c_uint8,
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_size_t
    ]
    _libec.ecrt_slave_config_sdo.restype = ctypes.c_int

    _libec.ecrt_master_select_reference_clock.argtypes = [
        ctypes.POINTER(ec_master_t),
        ctypes.POINTER(ec_slave_config_t)
    ]
    _libec.ecrt_master_select_reference_clock.restype = None

    _libec.ecrt_master_sync_reference_clock.argtypes = [ctypes.POINTER(ec_master_t)]
    _libec.ecrt_master_sync_reference_clock.restype = ctypes.c_int

    _libec.ecrt_master_sync_slave_clocks.argtypes = [ctypes.POINTER(ec_master_t)]
    _libec.ecrt_master_sync_slave_clocks.restype = ctypes.c_int


class SlaveConfig:
    def __init__(self, position: int, vendor_id: int, product_code: int):
        self.position = position
        self.vendor_id = vendor_id
        self.product_code = product_code
        self._config_handle = None
        self._pdo_entries = []
        self._sync_managers = []

    def add_pdo_entry(self, index: int, subindex: int, bit_length: int):
        self._pdo_entries.append({
            'index': index,
            'subindex': subindex,
            'bit_length': bit_length
        })

    def add_sync_manager(self, sm_index: int, direction: int, pdos: list):
        self._sync_managers.append({
            'index': sm_index,
            'direction': direction,
            'pdos': pdos
        })

    def register_pdo_entry(self, index: int, subindex: int, domain) -> Optional[int]:
        if not self._config_handle:
            logger.warning(f"Cannot register 0x{index:04X}:{subindex} - no config handle")
            return None
        bit_pos = ctypes.c_uint()
        offset = _libec.ecrt_slave_config_reg_pdo_entry(
            self._config_handle,
            index,
            subindex,
            domain,
            ctypes.byref(bit_pos)
        )
        if offset < 0:
            logger.error(f"Failed to register 0x{index:04X}:{subindex}, code: {offset}")
            return None
        return offset

    def config_dc(self, assign_activate: int, sync0_cycle_time_ns: int,
                  sync0_shift_ns: int = 0, sync1_cycle_time_ns: int = 0,
                  sync1_shift_ns: int = 0) -> bool:
        if not self._config_handle:
            logger.error("Cannot configure DC - no config handle")
            return False
        try:
            _libec.ecrt_slave_config_dc(
                self._config_handle,
                assign_activate,
                sync0_cycle_time_ns,
                sync0_shift_ns,
                sync1_cycle_time_ns,
                sync1_shift_ns
            )
            return True
        except Exception as e:
            logger.error(f"Failed to configure DC: {e}")
            return False


class Master:
    def __init__(self, master_index: int = 0):
        if _libec is None:
            raise MasterException("EtherCAT library not available")
        self.master_index = master_index
        self._master_handle = None
        self._activated = False
        self._domains = []
        self._slave_configs = {}
        # Keep ctypes-owned storage alive for the lifetime of this Master.
        # IgH is sensitive to pointer lifetimes in registration structs.
        self._pdo_reg_keepalive = []

    def _force_release_device(self, device_path: str, sigterm_first: bool = True) -> None:
        """
        Best-effort release of a busy EtherCAT device node by killing processes using it.

        This mirrors the proven "release master" preflight behavior used in deployment scripts.
        It is intentionally best-effort and never raises; callers decide whether to retry.
        """
        if not device_path:
            return
        if not shutil.which("fuser"):
            logger.warning("Cannot force-release EtherCAT device: `fuser` not found in PATH")
            return

        # `fuser -k` sends a signal to processes using the file. Prefer TERM then KILL.
        cmds = []
        if sigterm_first:
            cmds.append(["fuser", "-k", "-TERM", device_path])
        cmds.append(["fuser", "-k", "-KILL", device_path])

        for cmd in cmds:
            try:
                subprocess.run(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False,
                )
            except Exception as e:
                logger.warning(f"Force-release step failed for {device_path}: {e}")

    def _describe_device_owners(self, device_path: str) -> str:
        """
        Best-effort: return a human-readable list of processes holding `device_path`.
        Uses `fuser -v` when available.
        """
        if not device_path:
            return ""
        if not shutil.which("fuser"):
            return ""
        try:
            cp = subprocess.run(
                ["fuser", "-v", device_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
            )
            out = (cp.stdout or "").strip()
            return out
        except Exception:
            return ""

    def request(
        self,
        sdo_only: bool = True,
        *,
        force_release_device: bool = False,
        device_path: str = "/dev/EtherCAT0",
        sigterm_first: bool = True,
        retry_delay_s: float = 1.0,
        force_release_attempts: int = 1,
        force_release_debug_owners: bool = False,
    ) -> bool:
        try:
            if sdo_only:
                self._master_handle = _libec.ecrt_open_master(self.master_index)
                if not self._master_handle:
                    raise MasterException(f"Failed to open master {self.master_index}")
            else:
                attempts = max(int(force_release_attempts), 1)
                for attempt in range(1, attempts + 1):
                    self._master_handle = _libec.ecrt_request_master(self.master_index)
                    if self._master_handle:
                        break

                    if not force_release_device:
                        break

                    if force_release_debug_owners:
                        owners = self._describe_device_owners(device_path)
                        if owners:
                            logger.warning(f"Device owners before release ({device_path}):\n{owners}")

                    logger.warning(
                        f"Failed to request master {self.master_index}; attempting force-release of {device_path} "
                        f"and retry ({attempt}/{attempts})..."
                    )
                    self._force_release_device(device_path=device_path, sigterm_first=sigterm_first)
                    try:
                        time.sleep(max(float(retry_delay_s), 0.0))
                    except Exception:
                        pass

                    if force_release_debug_owners:
                        owners = self._describe_device_owners(device_path)
                        if owners:
                            logger.warning(f"Device owners after release ({device_path}):\n{owners}")
                if not self._master_handle:
                    raise MasterException(f"Failed to request master {self.master_index}")
            return True
        except Exception as e:
            logger.error(f"Error requesting master: {e}")
            return False

    def release(self):
        if self._master_handle:
            if self._activated:
                self.deactivate()
            _libec.ecrt_release_master(self._master_handle)
            self._master_handle = None

    def create_domain(self):
        if not self._master_handle:
            raise MasterException("Master not requested")
        domain = _libec.ecrt_master_create_domain(self._master_handle)
        if not domain:
            raise MasterException("Failed to create domain")
        self._domains.append(domain)
        return domain

    def config_slave(self, alias: int, position: int, vendor_id: int, product_code: int) -> SlaveConfig:
        if not self._master_handle:
            raise MasterException("Master not requested - call request() first")
        config_handle = _libec.ecrt_master_slave_config(
            self._master_handle,
            alias,
            position,
            vendor_id,
            product_code
        )
        if not config_handle:
            raise MasterException(f"Failed to configure slave at position {position}")
        slave_config = SlaveConfig(position, vendor_id, product_code)
        slave_config._config_handle = config_handle
        self._slave_configs[position] = slave_config
        return slave_config

    def register_pdo_entry(self, domain, alias: int, position: int, vendor_id: int,
                           product_code: int, index: int, subindex: int) -> Optional[int]:
        offset = ctypes.c_uint()
        bit_position = ctypes.c_uint()
        reg = ec_pdo_entry_reg_t()
        reg.alias = alias
        reg.position = position
        reg.vendor_id = vendor_id
        reg.product_code = product_code
        reg.index = index
        reg.subindex = subindex
        reg.offset = ctypes.pointer(offset)
        reg.bit_position = ctypes.pointer(bit_position)
        reg_array = (ec_pdo_entry_reg_t * 2)()
        reg_array[0] = reg
        result = _libec.ecrt_domain_reg_pdo_entry_list(domain, reg_array)
        if result != 0:
            logger.error(f"Failed to register PDO entry 0x{index:04X}:{subindex}, result={result}")
            return None
        # Keepalive (conservative): retain the array + pointed-to vars.
        self._pdo_reg_keepalive.append((reg_array, offset, bit_position))
        return offset.value

    def register_pdo_entry_list(self, domain, alias: int, position: int, vendor_id: int,
                                product_code: int, entries: list) -> Dict[Tuple[int, int], int]:
        if not entries:
            return {}
        reg_array = (ec_pdo_entry_reg_t * (len(entries) + 1))()
        offset_vars = []
        bit_pos_vars = []
        for i, (index, subindex) in enumerate(entries):
            offset_var = ctypes.c_uint()
            bit_pos_var = ctypes.c_uint()
            offset_vars.append(offset_var)
            bit_pos_vars.append(bit_pos_var)
            reg_array[i].alias = alias
            reg_array[i].position = position
            reg_array[i].vendor_id = vendor_id
            reg_array[i].product_code = product_code
            reg_array[i].index = index
            reg_array[i].subindex = subindex
            reg_array[i].offset = ctypes.pointer(offset_var)
            reg_array[i].bit_position = ctypes.pointer(bit_pos_var)
        result = _libec.ecrt_domain_reg_pdo_entry_list(domain, reg_array)
        if result != 0:
            logger.error(f"Bulk PDO registration failed: {result}")
            return {}
        # Keepalive (conservative): retain the array + pointed-to vars.
        self._pdo_reg_keepalive.append((reg_array, offset_vars, bit_pos_vars))
        offsets = {}
        for i, (index, subindex) in enumerate(entries):
            offsets[(index, subindex)] = offset_vars[i].value
        return offsets

    def configure_slave_pdos(self, slave_config: SlaveConfig, sync_configs: list) -> bool:
        if not slave_config._config_handle:
            raise MasterException("Slave not configured")
        sync_infos = []
        for sm_index, direction, pdos in sync_configs:
            pdo_infos = []
            for pdo_index, entries in pdos:
                entry_array = (ec_pdo_entry_info_t * len(entries))()
                for i, (entry_idx, subidx, bits) in enumerate(entries):
                    entry_array[i].index = entry_idx
                    entry_array[i].subindex = subidx
                    entry_array[i].bit_length = bits
                pdo_info = ec_pdo_info_t()
                pdo_info.index = pdo_index
                pdo_info.n_entries = len(entries)
                pdo_info.entries = ctypes.cast(entry_array, ctypes.POINTER(ec_pdo_entry_info_t))
                pdo_infos.append((pdo_info, entry_array))
            pdo_array = (ec_pdo_info_t * len(pdo_infos))()
            for i, (pdo_info, _) in enumerate(pdo_infos):
                pdo_array[i] = pdo_info
            sync_info = ec_sync_info_t()
            sync_info.index = sm_index
            sync_info.dir = direction
            sync_info.n_pdos = len(pdo_infos)
            sync_info.pdos = ctypes.cast(pdo_array, ctypes.POINTER(ec_pdo_info_t))
            sync_info.watchdog_mode = 0
            sync_infos.append((sync_info, pdo_array, pdo_infos))
        sync_array = (ec_sync_info_t * (len(sync_infos) + 1))()
        for i, (sync_info, _, _) in enumerate(sync_infos):
            sync_array[i] = sync_info
        sync_array[len(sync_infos)].index = 0xFF
        sync_array[len(sync_infos)].dir = 0
        sync_array[len(sync_infos)].n_pdos = 0
        sync_array[len(sync_infos)].pdos = None
        sync_array[len(sync_infos)].watchdog_mode = 0
        result = _libec.ecrt_slave_config_pdos(
            slave_config._config_handle,
            len(sync_infos),
            ctypes.cast(sync_array, ctypes.POINTER(ec_sync_info_t))
        )
        if result != 0:
            raise MasterException(
                f"Failed to configure PDOs for slave {slave_config.position}: {result}")
        slave_config._sync_array = sync_array
        slave_config._sync_infos = sync_infos
        return True

    def slave_config_sdo(self, slave_config: SlaveConfig, index: int, subindex: int, data: bytes) -> bool:
        if not slave_config._config_handle:
            raise MasterException("Slave not configured")
        data_array = (ctypes.c_uint8 * len(data))(*data)
        result = _libec.ecrt_slave_config_sdo(
            slave_config._config_handle,
            index,
            subindex,
            data_array,
            len(data)
        )
        if result != 0:
            raise MasterException(
                f"Failed to register SDO config: index=0x{index:04X}, subindex={subindex}, result={result}")
        return True

    def activate(self) -> bool:
        if not self._master_handle:
            raise MasterException("Master not requested")
        if self._activated:
            return True
        result = _libec.ecrt_master_activate(self._master_handle)
        if result != 0:
            raise MasterException(f"Failed to activate master: {result}")
        self._activated = True
        return True

    def deactivate(self):
        if self._master_handle and self._activated:
            _libec.ecrt_master_deactivate(self._master_handle)
            self._activated = False

    def set_application_time(self, time_ns: int):
        if not self._master_handle:
            raise MasterException("Master not requested")
        _libec.ecrt_master_application_time(self._master_handle, ctypes.c_uint64(time_ns))

    def select_reference_clock(self, slave_config: SlaveConfig) -> bool:
        if not self._master_handle:
            raise MasterException("Master not requested")
        if not slave_config._config_handle:
            logger.error("Cannot select reference clock - slave not configured")
            return False
        try:
            _libec.ecrt_master_select_reference_clock(
                self._master_handle,
                slave_config._config_handle
            )
            return True
        except Exception as e:
            logger.error(f"Failed to select reference clock: {e}")
            return False

    def sync_reference_clock(self):
        if self._master_handle and self._activated:
            _libec.ecrt_master_sync_reference_clock(self._master_handle)

    def sync_slave_clocks(self):
        if self._master_handle and self._activated:
            _libec.ecrt_master_sync_slave_clocks(self._master_handle)

    def get_slave_count(self) -> int:
        if not self._master_handle:
            raise MasterException("Master not requested")
        info = ec_master_info_t()
        result = _libec.ecrt_master(self._master_handle, ctypes.byref(info))
        if result != 0:
            raise MasterException(f"Failed to get master info: {result}")
        return info.slave_count

    def get_slave_info(self, position: int) -> Optional[dict]:
        if not self._master_handle:
            raise MasterException("Master not requested")
        info = ec_slave_info_t()
        result = _libec.ecrt_master_get_slave(
            self._master_handle,
            position,
            ctypes.byref(info)
        )
        if result != 0:
            logger.error(f"Failed to get slave {position} info: {result}")
            return None
        return {
            'position': info.position,
            'vendor_id': info.vendor_id,
            'product_code': info.product_code,
            'revision_number': info.revision_number,
            'serial_number': info.serial_number,
            'alias': info.alias,
            'name': info.name.decode('utf-8') if info.name else '',
            'al_state': info.al_state,
            'error_flag': bool(info.error_flag),
        }

    def sdo_download(self, slave_position: int, index: int, subindex: int,
                     data: bytes) -> bool:
        if not self._master_handle:
            raise MasterException("Master not requested")
        abort_code = ctypes.c_uint32(0)
        data_array = (ctypes.c_uint8 * len(data))(*data)
        result = _libec.ecrt_master_sdo_download(
            self._master_handle,
            slave_position,
            index,
            subindex,
            data_array,
            len(data),
            ctypes.byref(abort_code)
        )
        if result != 0:
            raise SDOException(
                f"SDO download failed: slave={slave_position}, "
                f"index=0x{index:04X}, subindex={subindex}, "
                f"abort_code=0x{abort_code.value:08X}"
            )
        return True

    def sdo_upload(self, slave_position: int, index: int, subindex: int,
                   max_size: int = 4) -> Optional[bytes]:
        if not self._master_handle:
            raise MasterException("Master not requested")
        buffer = (ctypes.c_uint8 * max_size)()
        result_size = ctypes.c_size_t(0)
        abort_code = ctypes.c_uint32(0)
        result = _libec.ecrt_master_sdo_upload(
            self._master_handle,
            slave_position,
            index,
            subindex,
            buffer,
            max_size,
            ctypes.byref(result_size),
            ctypes.byref(abort_code)
        )
        if result != 0:
            raise SDOException(
                f"SDO upload failed: slave={slave_position}, "
                f"index=0x{index:04X}, subindex={subindex}, "
                f"abort_code=0x{abort_code.value:08X}"
            )
        return bytes(buffer[:result_size.value])

    def send(self):
        if self._master_handle and self._activated:
            _libec.ecrt_master_send(self._master_handle)

    def receive(self):
        if self._master_handle and self._activated:
            _libec.ecrt_master_receive(self._master_handle)

    def get_domain_data(self, domain) -> Optional[ctypes.POINTER(ctypes.c_uint8)]:
        if not self._activated:
            logger.warning("Cannot access domain data - master not activated")
            return None
        return _libec.ecrt_domain_data(domain)

    def read_domain(self, domain, byte_offset: int, size: int) -> Optional[bytes]:
        """
        Read `size` bytes from the process-data domain at `byte_offset`.

        This is a convenience wrapper around IgH `ecrt_domain_data()`.
        Callers are expected to call `receive()` + `process_domain()` before reading
        in a cyclic loop.
        """
        if domain is None or size <= 0:
            return None
        if byte_offset is None or int(byte_offset) < 0:
            return None

        data = self.get_domain_data(domain)
        if not data:
            return None

        base_addr = ctypes.addressof(data.contents)
        return ctypes.string_at(base_addr + int(byte_offset), int(size))

    def write_domain(self, domain, byte_offset: int, data_bytes: bytes) -> bool:
        """
        Write bytes into the process-data domain at `byte_offset`.

        This is a convenience wrapper around IgH `ecrt_domain_data()`.
        Callers are expected to call `queue_domain()` + `send()` after writing
        in a cyclic loop.
        """
        if domain is None or not data_bytes:
            return False
        if byte_offset is None or int(byte_offset) < 0:
            return False

        data = self.get_domain_data(domain)
        if not data:
            return False

        base_addr = ctypes.addressof(data.contents)
        ctypes.memmove(base_addr + int(byte_offset), data_bytes, len(data_bytes))
        return True

    def process_domain(self, domain):
        if domain and self._activated:
            _libec.ecrt_domain_process(domain)

    def queue_domain(self, domain):
        if domain and self._activated:
            _libec.ecrt_domain_queue(domain)

    def domain_state(self, domain) -> Tuple[int, int]:
        state = ec_domain_state_t()
        if domain and self._activated:
            _libec.ecrt_domain_state(domain, ctypes.byref(state))
        return (state.working_counter, state.wc_state)

"""
NOTE: Adapter layer removed.

This module intentionally provides ONLY the direct IgH ctypes binding.
"""