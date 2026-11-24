#!/usr/bin/env python3

"""
Dead-reckoning maths for a differential-drive robot.

Author: Mutasem Bader

Kept free of ROS imports so that it can be unit-tested without a ROS
installation (see test/test_odometry.py). path_node uses these functions.
"""

import math

# Robot physical constants
# Estimate: the treads of the Zumo chassis are 39 mm high (Pololu), the
# effective rolling radius is somewhat smaller. Calibrate it with a drive test.
WHEEL_RADIUS = 0.019     # metres
GEAR_RATIO = 75.81
# Counts per wheel revolution. Pololu's 12 CPR holds when both edges of both
# encoder channels are counted. The Zumo328P-Library counts only the edges of
# channel A, which is half of that: 6 counts per motor revolution. Calibrate
# the real value by turning a wheel exactly one revolution and reading the
# ticks (see the README).
COUNTS_PER_MOTOR_REV = 6
CPR = GEAR_RATIO * COUNTS_PER_MOTOR_REV
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
