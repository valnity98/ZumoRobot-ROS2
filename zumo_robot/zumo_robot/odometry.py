#!/usr/bin/env python3

"""
Dead-reckoning maths for a differential-drive robot.

Author: Mutasem Bader

Kept free of ROS imports so that it can be unit-tested without a ROS
installation (see test/test_odometry.py). path_node uses these functions.
"""

import math

# Robot physical constants
WHEEL_RADIUS = 0.019     # metres
GEAR_RATIO = 75.81
# Counts per wheel revolution: 12 CPR encoder × gear ratio. Pololu's 12 CPR
# holds when both edges of both encoder channels are counted; the Arduino
# library counts only the edges of channel A. Calibrate the real value by
# turning a wheel exactly one revolution and reading the ticks (see README of
# the Zumo328P-Library).
CPR = GEAR_RATIO * 12
WHEELTRACK = 0.09        # metres, axle width


def ticks_to_metres(ticks):
    """Distance a wheel has rolled for the given number of encoder ticks."""
    return (ticks / CPR) * (2 * math.pi * WHEEL_RADIUS)


def normalise_angle(theta):
    """Wrap an angle into [-pi, pi)."""
    return (theta + math.pi) % (2 * math.pi) - math.pi


def integrate(x, y, theta, ticks_left, ticks_right):
    """
    Advance the pose by one encoder step.

    Args:
        x, y: Position in metres.
        theta: Heading in radians.
        ticks_left, ticks_right: Encoder ticks since the previous step.

    Returns:
        The new pose as a tuple (x, y, theta), theta wrapped into [-pi, pi).
    """
    delta_left = ticks_to_metres(ticks_left)
    delta_right = ticks_to_metres(ticks_right)

    delta_dist = (delta_left + delta_right) / 2.0
    delta_theta = (delta_right - delta_left) / WHEELTRACK

    # Midpoint heading reduces the systematic drift on curves.
    theta_mid = theta + delta_theta / 2.0
    x += delta_dist * math.cos(theta_mid)
    y += delta_dist * math.sin(theta_mid)
    return x, y, normalise_angle(theta + delta_theta)
