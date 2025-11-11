import time
from typing import Callable, Dict, Optional

from .cia402.driver import CiA402Drive
from .commands import Command, CommandType
from .process_manager import EtherCATProcessManager
from .status_model import NetworkStatus


class StatusProxy:
    def __init__(self, manager: EtherCATProcessManager):
        self._manager = manager
        self._last: Optional[NetworkStatus] = None
        self._last_at: float = 0.0

    def _refresh(self):
        now = time.time()
        # Throttle refresh to avoid hammering the queue
        if now - self._last_at > 0.02:
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


def attach_drive_handle(manager: EtherCATProcessManager, slave_position: int) -> CiA402Drive:
    """
    Create a CiA402DriveV2 handle and attach non-blocking enqueue/status readers
    that talk to the isolated process manager.
    """
    drive = CiA402Drive(slave_position=slave_position)
    status = StatusProxy(manager)

    def enqueue(pos: int, cmd_type: CommandType, value, params: Dict):
        manager.send_command(Command(target_id=pos, type=cmd_type, value=value, params=params))

    def read_status(pos: int, key: str):
        return status.get_field(pos, key)

    # Attach closures
    drive._enqueue_command = enqueue  # type: ignore[attr-defined]
    drive._read_status = read_status  # type: ignore[attr-defined]

    # Expose feature capability snapshot
    def get_features():
        latest = manager.get_latest_status()
        if latest and latest.drives.get(slave_position):
            return latest.drives[slave_position].get('features', {})
        return {}

    drive.get_features = get_features  # type: ignore[attr-defined]

    return drive


