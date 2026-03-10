class TrackingCorrector:
    def __init__(
        self,
        kp: float = 0.0,
        ki: float = 0.0,
        max_correction: int = 5000,
        integral_limit: int = 10000,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.max_correction = int(max_correction)
        self.integral_limit = int(integral_limit)
        self._integral = 0.0

    def reset(self) -> None:
        self._integral = 0.0

    def step(self, comp_target: int, shuttle_actual: int, dt_s: float) -> int:
        if self.kp <= 0 and self.ki <= 0:
            return comp_target
        err = comp_target - shuttle_actual
        self._integral += self.ki * err * dt_s
        self._integral = max(
            -self.integral_limit,
            min(self.integral_limit, self._integral),
        )
        correction = self.kp * err + self._integral
        correction_int = int(round(correction))
        correction_int = max(
            -self.max_correction,
            min(self.max_correction, correction_int),
        )
        return comp_target + correction_int
