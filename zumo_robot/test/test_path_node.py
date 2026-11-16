#!/usr/bin/env python3

"""Unit tests for the dead-reckoning maths and the path-point bookkeeping."""

import math
import unittest

import numpy as np

from zumo_robot import odometry


class TestDeadReckoning(unittest.TestCase):
    """Tests for zumo_robot.odometry (no ROS runtime required)."""

    def test_one_revolution_is_one_circumference(self):
        """CPR ticks move the wheel by exactly one circumference."""
        self.assertAlmostEqual(
            odometry.ticks_to_metres(odometry.CPR),
            2 * math.pi * odometry.WHEEL_RADIUS)

    def test_no_ticks_no_movement(self):
        """Without ticks the pose stays as it is."""
        self.assertEqual(odometry.integrate(1.0, 2.0, 0.5, 0, 0),
                         (1.0, 2.0, 0.5))

    def test_straight_forward(self):
        """Equal ticks on both wheels: straight line along x, theta unchanged."""
        x, y, theta = odometry.integrate(0.0, 0.0, 0.0, 100, 100)
        self.assertAlmostEqual(x, odometry.ticks_to_metres(100))
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(theta, 0.0)

    def test_straight_along_heading(self):
        """With a heading of 90 degrees the robot moves along y."""
        x, y, theta = odometry.integrate(0.0, 0.0, math.pi / 2, 100, 100)
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, odometry.ticks_to_metres(100))
        self.assertAlmostEqual(theta, math.pi / 2)

    def test_backwards(self):
        """Negative ticks move the robot backwards."""
        x, _y, _theta = odometry.integrate(0.0, 0.0, 0.0, -100, -100)
        self.assertAlmostEqual(x, -odometry.ticks_to_metres(100))

    def test_spin_in_place(self):
        """Equal but opposite ticks: pure rotation, no translation."""
        x, y, theta = odometry.integrate(0.0, 0.0, 0.0, -100, 100)
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 0.0)
        expected = 2 * odometry.ticks_to_metres(100) / odometry.WHEELTRACK
        self.assertAlmostEqual(theta, expected)

    def test_left_turn_is_positive(self):
        """A faster right wheel turns the robot to the left (theta grows)."""
        _x, y, theta = odometry.integrate(0.0, 0.0, 0.0, 50, 100)
        self.assertGreater(theta, 0.0)
        self.assertGreater(y, 0.0)

    def test_theta_normalisation(self):
        """Angles are wrapped into [-pi, pi)."""
        self.assertAlmostEqual(odometry.normalise_angle(3 * math.pi), -math.pi)
        self.assertAlmostEqual(odometry.normalise_angle(-3 * math.pi), -math.pi)
        self.assertAlmostEqual(odometry.normalise_angle(0.25), 0.25)
        for theta in (-10.0, -math.pi, 0.0, math.pi, 10.0):
            wrapped = odometry.normalise_angle(theta)
            self.assertGreaterEqual(wrapped, -math.pi)
            self.assertLess(wrapped, math.pi)

    def test_full_circle_returns_to_start(self):
        """Many small steps on a circle end where they started."""
        # Ticks per step for the inner (left) and outer (right) wheel.
        left, right = 10, 20
        step_theta = (odometry.ticks_to_metres(right - left)
                      / odometry.WHEELTRACK)
        steps = round(2 * math.pi / step_theta)
        x = y = theta = 0.0
        for _ in range(steps):
            x, y, theta = odometry.integrate(x, y, theta, left, right)
        # The circle does not close exactly because steps is rounded.
        self.assertAlmostEqual(x, 0.0, delta=0.01)
        self.assertAlmostEqual(y, 0.0, delta=0.01)


class TestPathPoints(unittest.TestCase):
    """Tests for the incremental path-point list used by path_node."""

    def setUp(self):
        self.map_data = np.full((10, 10), -1, dtype=np.int8)
        self.path_points: list = []

    def _mark(self, x: int, y: int):
        # Same logic as PathMappingNode._update_map
        if self.map_data[y, x] != 100:
            self.map_data[y, x] = 100
            self.path_points.append((x, y))

    def test_no_duplicate_points(self):
        """Marking the same cell twice does not add a duplicate."""
        self._mark(5, 5)
        self._mark(5, 5)
        self.assertEqual(len(self.path_points), 1)

    def test_path_grows_monotonically(self):
        """Each new cell appended exactly once."""
        for i in range(5):
            self._mark(i, 0)
        self.assertEqual(len(self.path_points), 5)

    def test_path_list_matches_map(self):
        """Path-point list is consistent with map_data."""
        for i in range(3):
            self._mark(i, i)
        occupied = list(zip(*np.where(self.map_data == 100)))  # (y, x) pairs
        self.assertEqual(len(occupied), len(self.path_points))


if __name__ == '__main__':
    unittest.main()
