#!/usr/bin/env python3

"""Unit tests for the PID controller in pid_lib (no ROS runtime required)."""

import unittest

from pid_lib.PIDController import Zumo328PPID


class TestPID(unittest.TestCase):

    def test_zero_error_drives_straight(self):
        """On the target position both motors run at the base speed."""
        pid = Zumo328PPID(kp=0.35, ki=0.1, kd=0.1, max_speed=125.0)
        pid.control_speed(320, target_position=320)
        self.assertEqual(pid.get_left_speed(), 125.0)
        self.assertEqual(pid.get_right_speed(), 125.0)

    def test_proportional_term(self):
        """With only kp, the right motor is slowed down by kp * error."""
        pid = Zumo328PPID(kp=0.5, ki=0.0, kd=0.0, max_speed=100.0)
        pid.control_speed(360, target_position=320)  # error = +40
        self.assertEqual(pid.get_left_speed(), 100.0)   # limited to max_speed
        self.assertAlmostEqual(pid.get_right_speed(), 80.0)

    def test_steering_direction(self):
        """A line to the left of the target slows down the left motor."""
        pid = Zumo328PPID(kp=0.5, ki=0.0, kd=0.0, max_speed=100.0)
        pid.control_speed(280, target_position=320)  # error = -40
        self.assertAlmostEqual(pid.get_left_speed(), 80.0)
        self.assertEqual(pid.get_right_speed(), 100.0)

    def test_output_is_limited(self):
        """The speeds stay within [-max_speed, max_speed]."""
        pid = Zumo328PPID(kp=100.0, ki=0.0, kd=0.0, max_speed=100.0)
        pid.control_speed(640, target_position=320)
        self.assertEqual(pid.get_left_speed(), 100.0)
        self.assertEqual(pid.get_right_speed(), -100.0)

    def test_no_derivative_spike_on_first_sample(self):
        """The first sample has no previous error, so kd has no effect."""
        pid = Zumo328PPID(kp=0.0, ki=0.0, kd=10.0, max_speed=100.0)
        pid.control_speed(360, target_position=320)
        self.assertEqual(pid.get_left_speed(), 100.0)
        self.assertEqual(pid.get_right_speed(), 100.0)

    def test_derivative_acts_on_error_change(self):
        """From the second sample on, a growing error adds to the correction."""
        pid = Zumo328PPID(kp=0.0, ki=0.0, kd=1.0, max_speed=100.0,
                          active=False)  # fixed time step of 1 s
        pid.control_speed(320, target_position=320)
        pid.control_speed(360, target_position=320)  # error 0 -> 40
        # raw derivative 40, low-pass coefficient 0.1 -> 4
        self.assertAlmostEqual(pid.get_right_speed(), 96.0)

    def test_integral_is_limited(self):
        """Anti-windup: the integral stays inside integral_limit."""
        pid = Zumo328PPID(kp=0.0, ki=1.0, kd=0.0, max_speed=100.0,
                          integral_limit=(-5.0, 5.0))
        pid._prev_time -= 1000.0  # as if the last sample was long ago
        pid.control_speed(420, target_position=320)
        self.assertEqual(pid._integral, 5.0)
        self.assertAlmostEqual(pid.get_right_speed(), 95.0)

    def test_reset_clears_state(self):
        """After reset() the controller behaves like a new one."""
        pid = Zumo328PPID(kp=0.0, ki=1.0, kd=1.0, max_speed=100.0)
        pid._prev_time -= 1000.0
        pid.control_speed(420, target_position=320)
        pid.reset()
        self.assertEqual(pid._integral, 0.0)
        self.assertEqual(pid._filtered_derivative, 0.0)
        self.assertIsNone(pid._last_error)


if __name__ == '__main__':
    unittest.main()
