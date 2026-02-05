import logging
from typing import Optional


logger = logging.getLogger(__name__)


class RuckigAddon:
    """
    Optional motion planning addon integrating Ruckig trajectories.

    This addon is a small *convenience* wrapper. The actual Ruckig execution happens
    inside the isolated EtherCAT process (see `process_manager.py` + `ruckig_planner.py`)
    and streams CSP targets (0x607A) every cycle.
    """

    def __init__(self):
        try:
            import ruckig  # type: ignore
            self._ruckig_available = True
        except Exception:
            self._ruckig_available = False
            logger.warning("Ruckig not available; addon disabled")

    def available(self) -> bool:
        return self._ruckig_available

    def move_to(self, drive, position: float, *, unit: str = "native", **limits) -> bool:
        """
        Convenience wrapper for `CiA402Drive.ruckig_move_to`.
        """
        if not self.available():
            return False
        return bool(drive.ruckig_move_to(position, unit=unit, **limits))

    def set_velocity(self, drive, velocity: float, *, unit: str = "native", **limits) -> bool:
        """
        Convenience wrapper for `CiA402Drive.ruckig_set_velocity`.
        """
        if not self.available():
            return False
        return bool(drive.ruckig_set_velocity(velocity, unit=unit, **limits))

    def stop(self, drive) -> bool:
        """
        Convenience wrapper for `CiA402Drive.ruckig_stop`.
        """
        if not self.available():
            return False
        return bool(drive.ruckig_stop())


