import logging
import time
from typing import Callable, Dict, Optional

from .cia402.driver import CiA402Drive
from .commands import Command, CommandType
from .config_schema import EthercatNetworkConfig
from .process_manager import EtherCATProcessManager
from .status_model import NetworkStatus


class StatusProxy:
    def __init__(self, manager: EtherCATProcessManager):
        self._manager = manager
        self._last: Optional[NetworkStatus] = None
        self._last_at: float = 0.0

    def _refresh(self):
        now = time.time()
        refresh_period_s = float(getattr(self._manager.cfg, "status_proxy_refresh_s", 0.02))
        if refresh_period_s < 0:
            refresh_period_s = 0.0
        if now - self._last_at > refresh_period_s:
            latest = self._manager.get_status_snapshot()
            if latest is None:
                latest = self._manager.get_latest_status()
            if latest is not None:
                self._last = latest
                self._last_at = now

    def get_field(self, slave_position: int, key: str):
        self._refresh()
        if not self._last:
            return None
        drive = self._last.drives.get(slave_position) or {}
        return drive.get(key)


def attach_drive_handle(
    manager: EtherCATProcessManager,
    slave_position: int,
    status_proxy: Optional[StatusProxy] = None,
) -> CiA402Drive:
    drive = CiA402Drive(slave_position=slave_position)
    status = status_proxy or StatusProxy(manager)

    def enqueue(pos: int, cmd_type: CommandType, value, params: Dict):
        manager.send_command(Command(target_id=pos, type=cmd_type, value=value, params=params))

    def read_status(pos: int, key: str):
        return status.get_field(pos, key)

    drive._enqueue_command = enqueue  # type: ignore[attr-defined]
    drive._read_status = read_status  # type: ignore[attr-defined]

    def get_features():
        latest = manager.get_status_snapshot()
        if latest is None:
            latest = manager.get_latest_status()
        if latest and latest.drives.get(slave_position):
            return latest.drives[slave_position].get('features', {})
        return {}

    drive.get_features = get_features  # type: ignore[attr-defined]

    return drive


class EtherCATBus:
    def __init__(self, config: EthercatNetworkConfig):
        self._config = config
        self._manager: Optional[EtherCATProcessManager] = None
        self._drives: Dict[int, CiA402Drive] = {}
        self._status: Optional[StatusProxy] = None
        self._logger = logging.getLogger('EtherCATBus')

    def start(self):
        self._manager = EtherCATProcessManager(self._config)
        self._manager.start()
        self._status = StatusProxy(self._manager)
        for slave_cfg in self._config.slaves:
            if slave_cfg.cia402:
                drive = attach_drive_handle(
                    self._manager,
                    slave_cfg.position,
                    status_proxy=self._status,
                )
                self._drives[slave_cfg.position] = drive
                self._logger.info(f"Attached drive at position {slave_cfg.position}")

    def get_drive(self, position: int) -> Optional[CiA402Drive]:
        return self._drives.get(position)

    def read_field(self, slave_position: int, key: str):
        if self._status:
            return self._status.get_field(slave_position, key)
        return None

    def send_command(self, command: Command):
        if self._manager:
            self._manager.send_command(command)

    def shutdown(self):
        if self._manager:
            self._logger.info("Shutting down EtherCAT bus")
            self._manager.stop()
            self._manager = None
            self._drives.clear()
            self._status = None

    @property
    def is_running(self) -> bool:
        return self._manager is not None and self._manager.is_alive()


