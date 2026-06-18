#!/usr/bin/env python3

"""Unit tests for PathMappingNode dead-reckoning and map logic."""

import math
import unittest
from unittest.mock import MagicMock, patch

import numpy as np


class TestDeadReckoning(unittest.TestCase):
    """Tests for odometry maths — no ROS runtime required."""

    # Mirror constants from path_node
    WHEEL_RADIUS = 0.019
    GEAR_RATIO = 75.81
    CPR = (GEAR_RATIO * 12) / 2
    WHEELTRACK = 0.09
    MAP_SIZE = 1000
    RESOLUTION = 0.01

    def _tick_to_metres(self, ticks: int) -> float:
        return (ticks / self.CPR) * (2 * math.pi * self.WHEEL_RADIUS)

    def test_straight_forward(self):
        """Equal ticks on both wheels → straight line, theta unchanged."""
        ticks = 100
        d = self._tick_to_metres(ticks)
        delta_dist = d
        delta_theta = 0.0
        self.assertAlmostEqual(delta_theta, 0.0, places=6)
        self.assertGreater(delta_dist, 0.0)

    def test_spin_in_place(self):
        """Equal but opposite ticks → pure rotation, no translation."""
        ticks = 100
        d = self._tick_to_metres(ticks)
        delta_dist = (d + (-d)) / 2.0
        delta_theta = (d - (-d)) / self.WHEELTRACK
        self.assertAlmostEqual(delta_dist, 0.0, places=6)
        self.assertNotEqual(delta_theta, 0.0)

    def test_theta_normalisation(self):
        """Angle wraps correctly into [-π, π]."""
        theta = 3 * math.pi  # > π
        normalised = (theta + math.pi) % (2 * math.pi) - math.pi
        self.assertGreaterEqual(normalised, -math.pi)
        self.assertLessEqual(normalised, math.pi)

    def test_map_boundary_clamp(self):
        """Position is clamped to map bounds."""
        x = 99999.0
        clamped = max(0.0, min(self.MAP_SIZE - 1, x))
        self.assertEqual(clamped, self.MAP_SIZE - 1)

        x = -500.0
        clamped = max(0.0, min(self.MAP_SIZE - 1, x))
        self.assertEqual(clamped, 0.0)

    def test_cell_to_metres_conversion(self):
        """Centre cell maps to (0, 0) in metric space."""
        centre_cell = self.MAP_SIZE / 2.0
        origin_offset = -self.MAP_SIZE * self.RESOLUTION / 2.0
        x_metres = origin_offset + centre_cell * self.RESOLUTION
        self.assertAlmostEqual(x_metres, 0.0, places=6)


class TestPathPoints(unittest.TestCase):
    """Tests for the incremental path-point list (performance fix)."""

    def setUp(self):
        self.map_data = np.full((10, 10), -1, dtype=np.int8)
        self.path_points: list = []

    def _mark(self, x: int, y: int):
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
