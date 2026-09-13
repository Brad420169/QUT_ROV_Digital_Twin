"""Reusable PID controller."""

from control_utils import wrap_to_pi


class PIDController:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_min: float = -1.0,
        output_max: float = 1.0,
        integral_limit: float = 0.3,
        wrap_angle: bool = False,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit
        self.wrap_angle = wrap_angle

        self._integral = 0.0
        self._prev_measurement = None

    def reset(self):
        self._integral = 0.0
        self._prev_measurement = None

    def compute(
        self,
        setpoint: float,
        measurement: float,
        dt: float,
        measurement_rate: float | None = None,
    ) -> float:
        if dt <= 0.0:
            return 0.0

        error = setpoint - measurement
        if self.wrap_angle:
            error = wrap_to_pi(error)

        p_term = self.kp * error

        self._integral += error * dt
        self._integral = max(
            -self.integral_limit,
            min(self.integral_limit, self._integral),
        )
        i_term = self.ki * self._integral

        if measurement_rate is not None:
            d_term = -self.kd * measurement_rate

        elif self._prev_measurement is not None:
            d_measurement = measurement - self._prev_measurement

            if self.wrap_angle:
                d_measurement = wrap_to_pi(d_measurement)

            d_term = -self.kd * (d_measurement / dt)

        else:
            d_term = 0.0

        self._prev_measurement = measurement

        output = p_term + i_term + d_term
        return max(self.output_min, min(self.output_max, output))
