#!/usr/bin/env python3
##
# @file mpc.py
#
# @brief Provide implementation of a linear MPC controller with a fixed
# (known) slip factor for autonomous driving.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 08/12/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library
import csv

# External library
import numpy as np

# Internal library
from beaverbot_control.linear_mpc import LinearMPC


class MPC:
    """! MPC controller

    The class provides implementation of a linear MPC controller that
    corrects wheel-velocity references from the reference trajectory,
    assuming a fixed (known) wheel slip factor.

    Requires the trajectory to be built with trajectory_type="wheel", so
    that trajectory.u[0, :] / trajectory.u[1, :] hold [v, w] with the
    standard convention w = (vr - vl) / wheel_base (see
    BeaverbotControl._retrieve_u's "wheel" branch). The raw per-wheel
    references (vr_ref, vl_ref) are recovered from [v, w], and MPC's own
    wheel-velocity corrections are converted back to [v, w] in the same
    convention before being returned.

    Ignores the caller-supplied `index` (a raw elapsed-tick counter) for
    tracking purposes. Instead, each call re-derives the reference index
    from the robot's actual measured position via an incremental
    nearest-point search over trajectory.x (same technique as
    PurePursuit._search_target_index), so the error state and horizon
    are always anchored to where the robot actually is on the path, not
    to where the schedule assumes it should be. This matters most when
    slip/saturation causes the robot to fall behind (or get ahead of)
    the nominal time-parameterized schedule.
    """
    # ==================================================================================================
    # PUBLIC METHODS
    # ==================================================================================================
    def __init__(self, trajectory, wheel_base, sampling_time, N_horizon=10,
                 slip=0.0, vr_max=0.5, vl_max=0.5, du_max=0.05, log_file=None):
        """! Constructor
        @param trajectory<instance>: The trajectory
        @param wheel_base<float>: Distance between the wheels of the robot.
        @param sampling_time<float>: Time step of the control loop.
        @param N_horizon<int>: Number of time steps in the prediction horizon.
        @param slip<float>: Fixed slip factor used by the MPC's system model.
        @param vr_max<float>: Maximum velocity of the right wheel.
        @param vl_max<float>: Maximum velocity of the left wheel.
        @param du_max<float>: Maximum allowed change in each wheel's
        delta-velocity per step.
        @param log_file<str or None>: If set, the path of a CSV file to
        record the tracking state (target position, error, MPC output,
        solver status) at every step, for offline analysis. Recording is
        disabled when None.
        """
        self.trajectory = trajectory

        self._mpc = LinearMPC(dt=sampling_time, wheel_base=wheel_base,
                              N_horizon=N_horizon, vr_max=vr_max, vl_max=vl_max,
                              s=slip, du_max=du_max)

        self._nearest_index = None

        self.log_file = log_file

        if self.log_file:
            with open(self.log_file, mode="w", newline="") as file:
                writer = csv.writer(file)
                # "slip" is self._mpc.s at the time of this row -- the fixed
                # value passed in here for plain MPC, or MPCRLS's freshly
                # estimated value each tick (see MPCRLS._update_slip_estimate).
                writer.writerow(["index", "nearest_index", "delta_t",
                                  "state_x", "state_y", "state_theta",
                                  "ref_x", "ref_y", "ref_theta",
                                  "error_x", "error_y", "error_theta",
                                  "vr_ref", "vl_ref", "delta_vr", "delta_vl",
                                  "vr_cmd", "vl_cmd", "v", "w", "slip",
                                  "solver_status"])

    def execute(self, state, input, index, delta_t):
        """! Execute the controller
        @param state<list>: The state of the vehicle
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

        reference_state = np.array(self.trajectory.x[nearest_index], dtype=float)

        error_state = self._mpc.compute_error_state(
            np.array(state, dtype=float), reference_state)

        A_list, B_list, vr_ref_horizon, vl_ref_horizon = self._build_horizon(nearest_index, last_index)

        delta_vr, delta_vl = self._mpc.solve(
            error_state, A_list, B_list, vr_ref_horizon, vl_ref_horizon)

        vr_ref, vl_ref = self._wheel_velocities(nearest_index)

        vr_cmd = vr_ref + delta_vr

        vl_cmd = vl_ref + delta_vl

        v = (vr_cmd + vl_cmd) / 2

        w = (vr_cmd - vl_cmd) / self._mpc.l

        self._record_step(index, nearest_index, delta_t, state, reference_state,
                          error_state, vr_ref, vl_ref, delta_vr, delta_vl,
                          vr_cmd, vl_cmd, v, w, self._mpc.s,
                          self._mpc.problem.status)

        return True, [v, w]

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    def _record_step(self, index, nearest_index, delta_t, state, reference_state,
                      error_state, vr_ref, vl_ref, delta_vr, delta_vl,
                      vr_cmd, vl_cmd, v, w, slip, solver_status):
        """! Append one row of the current tracking state to log_file.
        @param index<int>: The elapsed-tick counter from the node
        @param nearest_index<int>: The trajectory index actually tracked
        @param delta_t<float>: The time step
        @param state<list>: The actual state of the vehicle [x, y, theta]
        @param reference_state<np.ndarray>: The target state [x, y, theta]
        at nearest_index
        @param error_state<np.ndarray>: The error state [x, y, theta]
        (state - reference_state, theta wrapped to [-pi, pi])
        @param vr_ref<float>: Reference right wheel velocity
        @param vl_ref<float>: Reference left wheel velocity
        @param delta_vr<float>: MPC correction to the right wheel velocity
        @param delta_vl<float>: MPC correction to the left wheel velocity
        @param vr_cmd<float>: Commanded right wheel velocity (vr_ref + delta_vr)
        @param vl_cmd<float>: Commanded left wheel velocity (vl_ref + delta_vl)
        @param v<float>: The commanded linear velocity
        @param w<float>: The commanded angular velocity
        @param slip<float>: The slip factor used by the MPC's system model
        this step -- fixed for plain MPC, MPCRLS's current RLS estimate
        otherwise (see MPCRLS._update_slip_estimate).
        @param solver_status<str>: The cvxpy solver status for this step
        """
        if not self.log_file:
            return
        with open(self.log_file, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([index, nearest_index, delta_t,
                              state[0], state[1], state[2],
                              reference_state[0], reference_state[1], reference_state[2],
                              error_state[0], error_state[1], error_state[2],
                              vr_ref, vl_ref, delta_vr, delta_vl,
                              vr_cmd, vl_cmd, v, w, slip, solver_status])

    def _search_nearest_index(self, state):
        """! Find the trajectory index nearest to the robot's current
        (x, y) position, searching incrementally forward from the
        previous call's result (safe to call more than once per tick
        with the same state -- it will not advance further the second
        time). Falls back to a full search only on the very first call.
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

    def _wheel_velocities(self, index):
        """! Recover the raw right/left wheel-velocity references at the
        given trajectory index from trajectory.u = [v, w] (see class
        docstring for the sign convention).
        @param index<int>: The trajectory index
        @return<tuple>: The right and left wheel-velocity references
        (vr_ref, vl_ref)
        """
        v_ref = self.trajectory.u[0, index]

        w_ref = self.trajectory.u[1, index]

        vr_ref = v_ref + w_ref * self._mpc.l / 2

        vl_ref = v_ref - w_ref * self._mpc.l / 2

        return vr_ref, vl_ref

    def _build_horizon(self, index, last_index):
        """! Build the linearized system matrices and reference
        wheel-velocity horizons starting at the given index.
        @param index<int>: The current trajectory index
        @param last_index<int>: The last valid trajectory index
        @return<tuple>: A_list, B_list, vr_ref_horizon, vl_ref_horizon
        """
        A_list, B_list = [], []

        vr_ref_horizon, vl_ref_horizon = [], []

        for i in range(self._mpc.N):
            future_index = min(index + i, last_index)

            theta_ref = self.trajectory.x[future_index, 2]

            vr_ref, vl_ref = self._wheel_velocities(future_index)

            A_i, B_i = self._mpc.define_AB_matrices(theta_ref, vr_ref, vl_ref)

            A_list.append(A_i)

            B_list.append(B_i)

            vr_ref_horizon.append(vr_ref)

            vl_ref_horizon.append(vl_ref)

        return A_list, B_list, vr_ref_horizon, vl_ref_horizon

    # ==================================================================================================
    # STATIC METHODS
    # ==================================================================================================
    @staticmethod
    def _distance(reference_x, current_x):
        """! Euclidean (x, y) distance between trajectory point(s) and
        the current state (same convention as
        PurePursuit._calculate_distance).
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
