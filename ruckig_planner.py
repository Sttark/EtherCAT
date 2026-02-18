import logging
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from .config_schema import RuckigConfig

logger = logging.getLogger(__name__)


@dataclass
class RuckigStep:
    position: int
    velocity: float
    acceleration: float
    done: bool
    result: Optional[Any] = None


class RuckigUnavailable(RuntimeError):
    pass


class RuckigCspPlanner:
    """
    Per-drive, 1-DoF Ruckig planner that outputs CSP position setpoints each cycle.

    This class is intentionally small and process-friendly:
    - No IO
    - No EtherCAT writes; caller consumes the setpoints
    """

    def __init__(self):
        self._ruckig = None
        self._Ruckig = None
        self._InputParameter = None
        self._OutputParameter = None
        self._Result = None
        self._load_ruckig()

        # slave_pos -> state
        self._state: Dict[int, Dict[str, Any]] = {}

    def _load_ruckig(self) -> None:
        try:
            import ruckig  # type: ignore

            self._ruckig = ruckig
            # Common python binding layout:
            self._Ruckig = getattr(ruckig, "Ruckig", None)
            self._InputParameter = getattr(ruckig, "InputParameter", None)
            self._OutputParameter = getattr(ruckig, "OutputParameter", None)
            self._Result = getattr(ruckig, "Result", None)
        except Exception as e:
            self._ruckig = None
            self._Ruckig = None
            self._InputParameter = None
            self._OutputParameter = None
            self._Result = None
            logger.warning(f"Ruckig import failed; planner unavailable: {e}")

    def available(self) -> bool:
        return bool(self._Ruckig and self._InputParameter and self._OutputParameter)

    def _require(self) -> None:
        if not self.available():
            raise RuckigUnavailable("Ruckig is not available in this environment (pip install ruckig).")

    @staticmethod
    def _dt_s(cfg: RuckigConfig, fallback_dt_s: float) -> float:
        if cfg.dt_ms is None:
            return float(fallback_dt_s)
        return float(cfg.dt_ms) / 1000.0

    @staticmethod
    def _limits(cfg: RuckigConfig, overrides: Dict[str, Optional[float]]) -> Tuple[float, float, float]:
        mv = overrides.get("max_velocity") if overrides else None
        ma = overrides.get("max_acceleration") if overrides else None
        mj = overrides.get("max_jerk") if overrides else None

        max_v = float(mv) if mv is not None else float(cfg.max_velocity or 0.0)
        max_a = float(ma) if ma is not None else float(cfg.max_acceleration or 0.0)
        max_j = float(mj) if mj is not None else float(cfg.max_jerk or 0.0)

        if max_v <= 0 or max_a <= 0 or max_j <= 0:
            raise ValueError("Ruckig limits must be > 0 (max_velocity/max_acceleration/max_jerk).")

        return max_v, max_a, max_j

    def stop(self, slave_pos: int) -> None:
        self._state.pop(slave_pos, None)

    def is_active(self, slave_pos: int) -> bool:
        return slave_pos in self._state

    def describe(self, slave_pos: int) -> Dict[str, Any]:
        s = self._state.get(slave_pos) or {}
        return {
            "active": bool(s),
            "mode": s.get("mode"),
            "target_position": s.get("target_position"),
            "target_velocity": s.get("target_velocity"),
            "error": s.get("error"),
        }

    def start_position(
        self,
        slave_pos: int,
        *,
        actual_position: int,
        actual_velocity: float,
        target_position: int,
        cfg: RuckigConfig,
        dt_s_fallback: float,
        overrides: Optional[Dict[str, Optional[float]]] = None,
    ) -> None:
        self._require()
        dt = self._dt_s(cfg, dt_s_fallback)
        max_v, max_a, max_j = self._limits(cfg, overrides or {})

        otg = self._new_otg(dt)
        inp = self._InputParameter(1)
        out = self._OutputParameter(1)

        inp.current_position = [float(actual_position)]
        inp.current_velocity = [float(actual_velocity)]
        inp.current_acceleration = [0.0]
        inp.target_position = [float(target_position)]
        inp.target_velocity = [0.0]
        inp.target_acceleration = [0.0]
        inp.max_velocity = [float(max_v)]
        inp.max_acceleration = [float(max_a)]
        inp.max_jerk = [float(max_j)]

        self._state[slave_pos] = {
            "mode": "position",
            "otg": otg,
            "inp": inp,
            "out": out,
            "target_position": int(target_position),
            "target_velocity": None,
            "cfg": cfg,
            "dt": dt,
            "lookahead_s": float(cfg.velocity_lookahead_s),
            "error": None,
        }

    def start_velocity(
        self,
        slave_pos: int,
        *,
        actual_position: int,
        actual_velocity: float,
        target_velocity: float,
        cfg: RuckigConfig,
        dt_s_fallback: float,
        overrides: Optional[Dict[str, Optional[float]]] = None,
    ) -> None:
        self._require()
        dt = self._dt_s(cfg, dt_s_fallback)
        # Velocity mode uses v/a/j limits; max_velocity isn't required to be set, but we will use it if provided.
        max_v, max_a, max_j = self._limits(cfg, overrides or {})
        lookahead = float(cfg.velocity_lookahead_s)
        if lookahead <= 0:
            raise ValueError("velocity_lookahead_s must be > 0.")

        otg = self._new_otg(dt)
        inp = self._InputParameter(1)
        out = self._OutputParameter(1)

        inp.current_position = [float(actual_position)]
        inp.current_velocity = [float(actual_velocity)]
        inp.current_acceleration = [0.0]
        inp.target_velocity = [float(target_velocity)]
        inp.target_acceleration = [0.0]
        inp.max_velocity = [float(max_v)]
        inp.max_acceleration = [float(max_a)]
        inp.max_jerk = [float(max_j)]

        # Initial moving target position to allow reaching/holding desired velocity
        inp.target_position = [float(actual_position + (target_velocity * lookahead))]

        self._state[slave_pos] = {
            "mode": "velocity",
            "otg": otg,
            "inp": inp,
            "out": out,
            "target_position": None,
            "target_velocity": float(target_velocity),
            "cfg": cfg,
            "dt": dt,
            "lookahead_s": lookahead,
            "error": None,
            "position_offset": 0,
        }

    def step(
        self,
        slave_pos: int,
        *,
        actual_position: Optional[int],
        actual_velocity: Optional[float],
    ) -> Optional[RuckigStep]:
        """
        Advance one cycle.

        - Returns None if no planner active for this slave.
        - Uses Ruckig internal state progression (preferred); measured actuals are used only to
          (re)initialize planners at start time in the calling code.
        """
        s = self._state.get(slave_pos)
        if not s:
            return None

        try:
            inp = s["inp"]
            out = s["out"]
            otg = s["otg"]

            if s.get("mode") == "velocity":
                tv = float(s.get("target_velocity") or 0.0)
                lookahead = float(s.get("lookahead_s") or 0.5)
                cur_pos = float(inp.current_position[0])

                if abs(cur_pos) > 1_000_000_000:
                    shift = int(round(cur_pos))
                    inp.current_position = [cur_pos - shift]
                    inp.target_position = [float(inp.target_position[0]) - shift]
                    s["position_offset"] = s.get("position_offset", 0) + shift
                    cur_pos = float(inp.current_position[0])

                inp.target_velocity = [tv]
                inp.target_acceleration = [0.0]
                inp.target_position = [cur_pos + (tv * lookahead)]

            res = otg.update(inp, out)
            done = self._is_finished(res)

            if hasattr(out, "pass_to_input"):
                out.pass_to_input(inp)
            else:
                inp.current_position = list(out.new_position)
                inp.current_velocity = list(out.new_velocity)
                inp.current_acceleration = list(out.new_acceleration)

            offset = s.get("position_offset", 0)
            pos = int(round(float(out.new_position[0]))) + offset
            vel = float(out.new_velocity[0])
            acc = float(out.new_acceleration[0])

            if done and s.get("mode") == "position":
                # For position moves, auto-stop when finished.
                self._state.pop(slave_pos, None)

            return RuckigStep(position=pos, velocity=vel, acceleration=acc, done=done, result=res)
        except Exception as e:
            s["error"] = str(e)
            logger.error(f"Ruckig step failed for slave {slave_pos}: {e}")
            # On failure, stop motion generation for safety.
            self._state.pop(slave_pos, None)
            return None

    def _new_otg(self, dt_s: float):
        """
        Ruckig python binding differs slightly by version; try common constructor signatures.
        """
        self._require()
        if not self._Ruckig:
            raise RuckigUnavailable("Ruckig class not found in binding.")

        # Common: Ruckig(dofs, dt)
        try:
            return self._Ruckig(1, float(dt_s))
        except TypeError:
            pass
        # Some bindings: Ruckig(dt) for 1 DoF default
        try:
            return self._Ruckig(float(dt_s))
        except TypeError:
            pass
        # Last resort: no args
        return self._Ruckig()

    def _is_finished(self, res: Any) -> bool:
        """
        Determine if Ruckig is finished, across binding variants.
        """
        Result = self._Result
        if Result is not None:
            finished = getattr(Result, "Finished", None)
            if finished is not None:
                return res == finished
        # Some bindings return strings or ints; treat unknown as "not finished"
        return False


