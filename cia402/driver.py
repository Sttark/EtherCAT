import logging
from dataclasses import dataclass
from typing import Dict, Optional

from ..commands import CommandType
from ..constants import MODE_PP, MODE_PV, MODE_CSP
from ..utils.checks import nonblocking_check


logger = logging.getLogger(__name__)


@dataclass
class PdoMap:
    # Offsets will be assigned by the process during registration
    offsets: Dict[int, Dict[int, int]]  # index -> subindex -> offset


class CiA402Drive:
    """
    Non-blocking CiA 402 driver handle.
    Methods enqueue intent; verification is done via periodic checks.
    """

    def __init__(self, slave_position: int, pdo_map: Optional[PdoMap] = None):
        self.slave_position = slave_position
        self._pdo_map = pdo_map
        self._pending_checks = []
        self._last_intent: Optional[Dict] = None

        # Feature flags - to be populated from XML decoder and runtime checks
        self.supports: Dict[str, bool] = {
            'touch_probe': False,
            'mode_command_in_pdo': False,
            'mode_display_in_pdo': False,
        }

    # Mode switching
    @nonblocking_check(name="set_position_mode", timeout_s=1.0)
    def set_position_mode(self, safe_switch: bool = False) -> bool:
        self._queue(CommandType.SET_POSITION_MODE)
        return True

    @nonblocking_check(name="set_velocity_mode", timeout_s=1.0)
    def set_velocity_mode(self, safe_switch: bool = False) -> bool:
        self._queue(CommandType.SET_VELOCITY_MODE)
        return True

    @nonblocking_check(name="set_csp_mode", timeout_s=1.0)
    def set_csp_mode(self, safe_switch: bool = False) -> bool:
        self._queue(CommandType.SET_CSP_MODE)
        return True

    # Motion
    def set_velocity(self, velocity: float, unit: str = 'native') -> bool:
        self._queue(CommandType.SET_VELOCITY, value=velocity, params={'unit': unit})
        return True

    def stop_moving(self) -> bool:
        self._queue(CommandType.STOP_MOTION)
        return True

    def set_position_absolute(self, position: float, unit: str = 'native') -> bool:
        self._queue(CommandType.SET_POSITION, value=position, params={'unit': unit})
        return True

    def set_position_csp(self, position: float, unit: str = 'native') -> bool:
        self._queue(CommandType.SET_POSITION_CSP, value=position, params={'unit': unit})
        return True

    # Homing
    def start_homing(self, wait_for_completion: bool = False, timeout_s: float = 30.0) -> bool:
        self._queue(CommandType.START_HOMING, params={'wait_for_completion': wait_for_completion, 'timeout_s': timeout_s})
        return True

    # Drive state
    def enable_drive(self) -> bool:
        self._queue(CommandType.ENABLE_DRIVE)
        return True

    def disable_drive(self) -> bool:
        self._queue(CommandType.DISABLE_DRIVE)
        return True

    def shutdown_drive(self) -> bool:
        self._queue(CommandType.SHUTDOWN_DRIVE)
        return True

    # Probe
    def arm_probe(self, edge: str = 'positive', continuous: bool = False) -> bool:
        if not self.supports.get('touch_probe', False):
            logger.warning(f"Slave {self.slave_position}: touch probe not supported")
            return False
        probe_function = None
        if edge == 'positive':
            probe_function = 0x0005
        elif edge == 'negative':
            probe_function = 0x0009
        else:
            logger.error(f"Invalid probe edge: {edge}")
            return False
        self._queue(CommandType.ARM_PROBE, params={'probe_function': probe_function, 'continuous': continuous})
        return True

    def disable_probe(self) -> bool:
        if not self.supports.get('touch_probe', False):
            return False
        self._queue(CommandType.DISABLE_PROBE)
        return True

    # Query helpers (populated by process status map)
    def is_enabled(self) -> bool:
        return bool(self._read_status_field('enabled'))

    def get_status_word(self) -> Optional[int]:
        return self._read_status_field('statusword')

    def get_position(self, unit: str = 'native') -> Optional[float]:
        return self._read_status_field('position_actual')

    def get_velocity(self, unit: str = 'native') -> Optional[float]:
        return self._read_status_field('velocity_actual')

    # Internal helpers delegated to the process runtime via injection
    def _queue(self, cmd_type: CommandType, value: Optional[float] = None, params: Optional[Dict] = None) -> None:
        if not hasattr(self, '_enqueue_command'):
            raise RuntimeError("Command queue handler not attached")
        self._last_intent = {'type': cmd_type, 'value': value, 'params': params or {}}
        self._enqueue_command(self.slave_position, cmd_type, value, params or {})

    def _read_status_field(self, key: str):
        if not hasattr(self, '_read_status'):
            return None
        return self._read_status(self.slave_position, key)

    # Verification hook used by @nonblocking_check
    def _verify_last_action(self) -> bool:
        if not self._last_intent:
            return True
        intent = self._last_intent
        if intent['type'] == CommandType.SET_POSITION_MODE:
            mode = self._read_status_field('mode_display')
            return mode == MODE_PP
        if intent['type'] == CommandType.SET_VELOCITY_MODE:
            mode = self._read_status_field('mode_display')
            return mode == MODE_PV
        if intent['type'] == CommandType.SET_CSP_MODE:
            mode = self._read_status_field('mode_display')
            return mode == MODE_CSP
        if intent['type'] in (CommandType.SET_POSITION, CommandType.SET_POSITION_CSP):
            # Non-strict: position should move toward target; exact verification is domain-specific
            return True
        if intent['type'] == CommandType.SET_VELOCITY:
            return True
        return True


