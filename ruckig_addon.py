import logging
from typing import Optional


logger = logging.getLogger(__name__)


class RuckigAddon:
    """
    Optional motion planning addon integrating Ruckig trajectories.
    This is a placeholder shim; actual integration hooks into the process
    cyclic writer to produce CSP targets.
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


