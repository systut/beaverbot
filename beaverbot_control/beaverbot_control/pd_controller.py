#!/usr/bin/env python3
##
# @file pd_controller.py
#
# @brief Provide implementation of a PD trajectory-tracking controller
# for autonomous driving.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 2026/07/17.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library
import csv

# External library
import numpy as np


class PDController:
    """! PD controller

    The class provides implementation of a PD (proportional-derivative)
    trajectory-tracking controller for autonomous driving.

    Each tick, the reference index is re-derived from the robot's actual
    (x, y) position via an incremental nearest-point search over
    trajectory.x (same technique as MPC._search_nearest_index /
    PurePursuit._search_target_index), rather than trusting the
    caller-supplied `index` (a raw elapsed-tick counter). The world-frame
    position error (actual - reference, same convention as
    LinearMPC.compute_error_state) is rotated into the robot's own
    heading frame to get a longitudinal error (e_lon) and lateral error
    (e_lat), plus a wrapped heading error (e_theta). PD terms on these
    errors are added to the feedforward [v_ref, w_ref] read from
    trajectory.u.

    The default gains below are untuned starting points -- like the
    RLSCompensator/MPC defaults, they need tuning on the actual robot.
    """
    # ==================================================================================================
    # PUBLIC METHODS
    # ==================================================================================================
    def __init__(self, trajectory, kp_v=1.0, kd_v=0.1, kp_theta=2.0,
                 kd_theta=0.1, kp_lat=1.0, log_file=None):
        """! Constructor
        @param trajectory<instance>: The reference trajectory
        @param kp_v<float>: Proportional gain on the longitudinal
        (heading-aligned) position error.
        @param kd_v<float>: Derivative gain on the longitudinal position
        error.
        @param kp_theta<float>: Proportional gain on the heading error.
        @param kd_theta<float>: Derivative gain on the heading error.
        @param kp_lat<float>: Proportional gain on the lateral
        (cross-track) position error, fed into the angular velocity.
        @param log_file<str or None>: If set, the path of a CSV file to
        record the tracking state (target position, error, PD output) at
        every step, for offline gain tuning. Recording is disabled when
        None.
        """
        self.trajectory = trajectory

        self.kp_v = kp_v

        self.kd_v = kd_v

        self.kp_theta = kp_theta

        self.kd_theta = kd_theta

        self.kp_lat = kp_lat

        self.log_file = log_file

        self._nearest_index = None

        self._e_lon_previous = None

        self._e_theta_previous = None

        if self.log_file:
            with open(self.log_file, mode="w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["index", "nearest_index", "delta_t",
                                  "state_x", "state_y", "state_theta",
                                  "ref_x", "ref_y", "ref_theta",
                                  "e_lon", "e_lat", "e_theta",
                                  "v_ref", "w_ref", "v", "w"])

    def execute(self, state, input, index, delta_t):
        """! Execute the controller
        @param state<list>: The state of the vehicle [x, y, theta]
        @param input<list>: The input of the vehicle (unused)
        @param index<int>: Elapsed-tick counter from the node (unused --
        see class docstring; kept only for interface compatibility with
        the other controllers).
        @param delta_t<float>: The time step
        @return<tuple>: The status and control
        """
        last_index = len(self.trajectory.u[0, :]) - 1

        nearest_index = self._search_nearest_index(state)

        if nearest_index >= last_index:
            return False, [0, 0]

        x_ref, y_ref, theta_ref = self.trajectory.x[nearest_index]

        v_ref = self.trajectory.u[0, nearest_index]

        w_ref = self.trajectory.u[1, nearest_index]

        e_lon, e_lat = self._compute_body_frame_error(state, x_ref, y_ref)

        e_theta = self._wrap_angle(state[2] - theta_ref)

        de_lon = self._compute_derivative(e_lon, self._e_lon_previous, delta_t)

        de_theta = self._compute_derivative(e_theta, self._e_theta_previous, delta_t)

        v = v_ref - self.kp_v * e_lon - self.kd_v * de_lon

        w = w_ref - self.kp_theta * e_theta - self.kd_theta * de_theta - self.kp_lat * e_lat

        self._e_lon_previous = e_lon

        self._e_theta_previous = e_theta

        self._record_step(index, nearest_index, delta_t, state,
                          (x_ref, y_ref, theta_ref), e_lon, e_lat, e_theta,
                          v_ref, w_ref, v, w)

        return True, [v, w]

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    def _compute_body_frame_error(self, state, x_ref, y_ref):
        """! Rotate the world-frame position error into the robot's own
        heading frame.
        @param state<list>: The state of the vehicle [x, y, theta]
        @param x_ref<float>: Reference x position
        @param y_ref<float>: Reference y position
        @return<tuple>: The longitudinal and lateral error (e_lon, e_lat)
        """
        ex = state[0] - x_ref

        ey = state[1] - y_ref

        theta = state[2]

        e_lon = np.cos(theta) * ex + np.sin(theta) * ey

        e_lat = -np.sin(theta) * ex + np.cos(theta) * ey

        return e_lon, e_lat

    def _compute_derivative(self, error, error_previous, delta_t):
        """! Finite-difference derivative of an error signal against its
        previous value (0.0 on the first tick, when there is no previous
        value yet).
        @param error<float>: The current error
        @param error_previous<float or None>: The previous error
        @param delta_t<float>: The time step
        @return<float>: The derivative of the error
        """
        if error_previous is None:
            return 0.0

        return (error - error_previous) / delta_t

    def _search_nearest_index(self, state):
        """! Find the trajectory index nearest to the robot's current
        (x, y) position, searching incrementally forward from the
        previous call's result (same technique as
        MPC._search_nearest_index). Falls back to a full search only on
        the very first call.
        @param state<list>: The state of the vehicle [x, y, theta]
        @return<int>: The nearest trajectory index
        """
        if self._nearest_index is None:
            distances = self._distance(self.trajectory.x, state)

            self._nearest_index = int(np.argmin(distances))

            return self._nearest_index

        index = self._nearest_index

        last_index = len(self.trajectory.x) - 1

        this_distance = self._distance(self.trajectory.x[index], state)

        while index < last_index:
            next_distance = self._distance(self.trajectory.x[index + 1], state)

            if this_distance < next_distance:
                break

            index += 1

            this_distance = next_distance

        self._nearest_index = index

        return index

    def _record_step(self, index, nearest_index, delta_t, state, reference_state,
                      e_lon, e_lat, e_theta, v_ref, w_ref, v, w):
        """! Append one row of the current tracking state to log_file.
        @param index<int>: The elapsed-tick counter from the node
        @param nearest_index<int>: The trajectory index actually tracked
        @param delta_t<float>: The time step
        @param state<list>: The actual state of the vehicle [x, y, theta]
        @param reference_state<tuple>: The target state (x, y, theta) at
        nearest_index
        @param e_lon<float>: The longitudinal (body-frame) position error
        @param e_lat<float>: The lateral (body-frame) position error
        @param e_theta<float>: The heading error, wrapped to [-pi, pi]
        @param v_ref<float>: Reference linear velocity
        @param w_ref<float>: Reference angular velocity
        @param v<float>: The commanded linear velocity
        @param w<float>: The commanded angular velocity
        """
        if not self.log_file:
            return

        with open(self.log_file, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([index, nearest_index, delta_t,
                              state[0], state[1], state[2],
                              reference_state[0], reference_state[1], reference_state[2],
                              e_lon, e_lat, e_theta,
                              v_ref, w_ref, v, w])

    # ==================================================================================================
    # STATIC METHODS
    # ==================================================================================================
    @staticmethod
    def _wrap_angle(angle):
        """! Wrap an angle to [-pi, pi].
        @param angle<float>: The angle to wrap
        @return<float>: The wrapped angle
        """
        return np.arctan2(np.sin(angle), np.cos(angle))

    @staticmethod
    def _distance(reference_x, current_x):
        """! Euclidean (x, y) distance between trajectory point(s) and
        the current state (same convention as MPC._distance).
        @param reference_x<np.ndarray>: One trajectory row [x, y, theta]
        or the full (N, 3) trajectory.x array
        @param current_x<list>: The current state [x, y, theta]
        @return<float or np.ndarray>: Distance(s)
        """
        current_x = np.asarray(current_x, dtype=float)

        delta = current_x - reference_x

        x = delta[:, 0] if delta.ndim == 2 else delta[0]

        y = delta[:, 1] if delta.ndim == 2 else delta[1]

        return np.hypot(x, y)
