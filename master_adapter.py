#!/usr/bin/env python3
"""
IgH EtherCAT Master Adapter (standalone)

This file mirrors the original pyethercat Master adapter with exact ctypes and
function prototypes, kept local to ethercat_v2 for modularity. It avoids
external package dependencies by defining exceptions inline.
"""

import ctypes
import logging
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
    _libec.ecrt_slave_config_dc.restype = None

    _libec.ecrt_master_select_reference_clock.argtypes = [
        ctypes.POINTER(ec_master_t),
        ctypes.POINTER(ec_slave_config_t)
    ]
    _libec.ecrt_master_select_reference_clock.restype = None


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

    def request(self, sdo_only: bool = True) -> bool:
        try:
            if sdo_only:
                self._master_handle = _libec.ecrt_open_master(self.master_index)
                if not self._master_handle:
                    raise MasterException(f"Failed to open master {self.master_index}")
            else:
                self._master_handle = _libec.ecrt_request_master(self.master_index)
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

    def process_domain(self, domain):
        if domain and self._activated:
            _libec.ecrt_domain_process(domain)

    def queue_domain(self, domain):
        if domain and self._activated:
            _libec.ecrt_domain_queue(domain)

import logging
from typing import Any, Dict, List, Optional, Tuple


logger = logging.getLogger(__name__)


class MasterAdapter:
    """
    Abstract interface for EtherCAT master implementations.
    Concrete adapters should wrap platform/master-specific APIs (e.g., IgH).
    """

    def request(self, sdo_only: bool = True) -> bool:
        raise NotImplementedError

    def release(self) -> None:
        raise NotImplementedError

    def create_domain(self):
        raise NotImplementedError

    def activate(self) -> bool:
        raise NotImplementedError

    def deactivate(self) -> None:
        raise NotImplementedError

    def config_slave(self, alias: int, position: int, vendor_id: int, product_code: int):
        raise NotImplementedError

    def configure_slave_pdos(self, slave_config, sync_configs: list) -> bool:
        raise NotImplementedError

    def register_pdo_entry_list(self, domain, alias: int, position: int, vendor_id: int,
                                product_code: int, entries: list) -> Dict[Tuple[int, int], int]:
        raise NotImplementedError

    def read_domain(self, domain, offset: int, size: int) -> Optional[bytes]:
        raise NotImplementedError

    def write_domain(self, domain, offset: int, data: bytes) -> bool:
        raise NotImplementedError

    def process_domain(self, domain) -> None:
        raise NotImplementedError

    def queue_domain(self, domain) -> None:
        raise NotImplementedError

    def receive(self) -> None:
        raise NotImplementedError

    def send(self) -> None:
        raise NotImplementedError

    def set_application_time(self, time_ns: int) -> None:
        raise NotImplementedError

    def sdo_upload(self, slave_position: int, index: int, subindex: int, max_size: int = 4) -> Optional[bytes]:
        raise NotImplementedError

    def sdo_download(self, slave_position: int, index: int, subindex: int, data: bytes) -> bool:
        raise NotImplementedError


class IGHMasterAdapter(MasterAdapter):
    """
    Adapter for the IgH EtherCAT Master using SttarkStandardLibrary pyethercat bindings.

    This class intentionally defers to the well-tested C-handles wrapper and exposes
    a stable interface to the v2 network/process manager.
    """

    def __init__(self, master_index: int = 0):
        try:
            # Import on-demand; rely on environment/PYTHONPATH provided by the application
            from pyethercat.master import Master  # type: ignore
        except Exception as e:
            logger.error(f"Failed to import pyethercat.master: {e}")
            raise

        self._cls_Master = Master
        self._master = self._cls_Master(master_index)

    def request(self, sdo_only: bool = True) -> bool:
        return self._master.request(sdo_only=sdo_only)

    def release(self) -> None:
        self._master.release()

    def create_domain(self):
        return self._master.create_domain()

    def activate(self) -> bool:
        return self._master.activate()

    def deactivate(self) -> None:
        self._master.deactivate()

    def config_slave(self, alias: int, position: int, vendor_id: int, product_code: int):
        return self._master.config_slave(alias, position, vendor_id, product_code)

    def configure_slave_pdos(self, slave_config, sync_configs: list) -> bool:
        return self._master.configure_slave_pdos(slave_config, sync_configs)

    def register_pdo_entry_list(self, domain, alias: int, position: int, vendor_id: int,
                                product_code: int, entries: list) -> Dict[Tuple[int, int], int]:
        return self._master.register_pdo_entry_list(domain, alias, position, vendor_id, product_code, entries)

    def read_domain(self, domain, offset: int, size: int) -> Optional[bytes]:
        return self._master.read_domain_data(domain, offset, size)

    def write_domain(self, domain, offset: int, data: bytes) -> bool:
        return self._master.write_domain_data(domain, offset, data)

    def sdo_upload(self, slave_position: int, index: int, subindex: int, max_size: int = 4) -> Optional[bytes]:
        return self._master.sdo_upload(slave_position, index, subindex, max_size)

    def sdo_download(self, slave_position: int, index: int, subindex: int, data: bytes) -> bool:
        return self._master.sdo_download(slave_position, index, subindex, data)

    def process_domain(self, domain) -> None:
        self._master.process_domain(domain)

    def queue_domain(self, domain) -> None:
        self._master.queue_domain(domain)

    def receive(self) -> None:
        self._master.receive()

    def send(self) -> None:
        self._master.send()

    def set_application_time(self, time_ns: int) -> None:
        try:
            self._master.set_application_time(time_ns)
        except Exception:
            pass


