#!/usr/bin/env python3

"""
PID controller for the Zumo robot.

Adjusts left/right motor speeds based on the lateral position error between
the measured centroid and the target centre position.

Usage:
    pid = Zumo328PPID(kp=0.35, ki=0.1, kd=0.1, max_speed=125.0)
    pid.control_speed(measured_position, target_position=334)
    left  = pid.get_left_speed()
    right = pid.get_right_speed()
"""

import time


class Zumo328PPID:
    def __init__(self, kp, ki, kd, max_speed=200.0, active=True,
                 integral_limit=(-50.0, 50.0)):
        """
        Args:
            kp, ki, kd: PID gains.
            max_speed: Motor speed at zero error (base speed).
            active: If False, deltaT is fixed at 1 s (no real-time timing).
            integral_limit: Anti-windup clamp for the integral term.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_speed = max_speed
        self.active = active
        self.integral_limit = integral_limit

        self._left_speed = 0.0
        self._right_speed = 0.0
        self._prev_time = time.time()
        self._last_error = 0.0
        self._integral = 0.0
        self._filtered_derivative = 0.0
        self._filter_coefficient = 0.1

    def control_speed(self, measured_position, target_position):
        """Calculate motor speeds from the lateral position error."""
        error = float(measured_position - target_position)

        if self.active:
            curr_time = time.time()
            delta_t = curr_time - self._prev_time
            if delta_t <= 0.0:
                delta_t = 1e-6
            self._prev_time = curr_time
            self._integral += error * delta_t
            self._integral = max(self.integral_limit[0],
                                 min(self._integral, self.integral_limit[1]))
        else:
            delta_t = 1.0
            self._integral = 0.0
            self._filtered_derivative = 0.0  # prevent derivative spike on re-enable

        raw_derivative = (error - self._last_error) / delta_t
        self._filtered_derivative += self._filter_coefficient * (
            raw_derivative - self._filtered_derivative)

        speed_diff = (self.kp * error
                      + self.ki * self._integral
                      + self.kd * self._filtered_derivative)
        self._last_error = error

        self._left_speed = max(0.0, min(self.max_speed + speed_diff, self.max_speed))
        self._right_speed = max(0.0, min(self.max_speed - speed_diff, self.max_speed))

    def get_left_speed(self):
        return self._left_speed

    def get_right_speed(self):
        return self._right_speed

    def set_left_speed(self, speed):
        self._left_speed = float(speed)

    def set_right_speed(self, speed):
        self._right_speed = float(speed)

    def reset(self):
        """Reset integrator and derivative state."""
        self._integral = 0.0
        self._filtered_derivative = 0.0
        self._last_error = 0.0
        self._prev_time = time.time()
