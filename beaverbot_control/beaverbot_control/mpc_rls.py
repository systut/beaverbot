#!/usr/bin/env python3
##
# @file mpc_rls.py
#
# @brief Provide implementation of a linear MPC controller whose slip
# factor is estimated online via recursive least squares.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 08/12/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# External library
import numpy as np

# Internal library
from beaverbot_control.mpc import MPC
from beaverbot_control.rls_online import RLSOnline


class MPCRLS(MPC):
    """! MPC + RLS controller

    The class provides implementation of a linear MPC controller that
    estimates the wheel slip factor online via recursive least squares
    (RLSOnline) from the heading measurement, and feeds it into the MPC's
    linearized system model each step.

    Requires the trajectory to be built with trajectory_type="wheel", so
    that trajectory.u[0, :] / trajectory.u[1, :] hold [v, w] (see MPC).

    Like MPC, ignores the caller-supplied `index` and instead uses the
    nearest-point search (see MPC._search_nearest_index) for tracking.

    The RLS slip estimate is regressed against the angular velocity that
    was actually commanded on the *previous* tick (self._last_w_cmd, the
    w returned by the previous execute() call, including that tick's MPC
    feedback correction) rather than the raw reference angular velocity at
    the current nearest_index -- the estimator's model assumes its input
    reflects what was truly applied to the vehicle, so feeding it the
    reference instead would bias the estimate whenever the applied command
    differs from the reference (which is always, once the MPC is actively
    correcting).
    """
    # ==================================================================================================
    # PUBLIC METHODS
    # ==================================================================================================
    def __init__(self, trajectory, wheel_base, sampling_time, N_horizon=10,
                 vr_max=0.5, vl_max=0.5, du_max=0.05,
                 warmup_steps=50, slip_clip=0.3,
                 slip_estimation_max_angular_velocity=0.4,
                 lam=0.96, log_file=None):
        """! Constructor
        @param trajectory<instance>: The trajectory
        @param wheel_base<float>: Distance between the wheels of the robot.
        @param sampling_time<float>: Time step of the control loop.
        @param N_horizon<int>: Number of time steps in the prediction horizon.
        @param vr_max<float>: Maximum velocity of the right wheel.
        @param vl_max<float>: Maximum velocity of the left wheel.
        @param du_max<float>: Maximum allowed change in each wheel's
        delta-velocity per step.
        @param warmup_steps<int>: Number of initial steps the slip
        estimate is forced to 0.0 while RLS converges.
        @param slip_clip<float>: The estimated slip is clipped to
        [0, slip_clip].
        @param slip_estimation_max_angular_velocity<float or None>: Skip
        the RLS update (hold the last estimate) on any tick where the
        last commanded |angular velocity| exceeds this -- see
        RLSCompensator's parameter of the same name for the full
        rationale. In short: RLSOnline's model assumes a static gain
        (measured ~= commanded * (1-slip)), which only holds once the
        vehicle has settled into a turn; during a sharp turn the gap
        between commanded and measured rotation is dynamic response
        lag/inertia, not persistent slip, and feeding that into a static
        model creates a runaway feedback loop -- the MPC's own feedback
        correction pushes the commanded angular velocity higher still,
        which (falsely) looks like even more slip. Set to None to
        disable this gate.
        @param lam<float>: Forgetting factor used by the RLS estimator.
        @param log_file<str or None>: If set, the path of a CSV file to
        record the tracking state at every step (see MPC.__init__).
        """
        super(MPCRLS, self).__init__(
            trajectory, wheel_base, sampling_time, N_horizon=N_horizon,
            slip=0.0, vr_max=vr_max, vl_max=vl_max, du_max=du_max,
            log_file=log_file)

        s0 = np.array([[0.0]])

        P0 = np.eye(1) * 50.0

        R = np.eye(1) * 2 * 0.00436 ** 2

        self._rls = RLSOnline(s0, P0, R)

        self._warmup_steps = warmup_steps

        self._slip_clip = slip_clip

        self._slip_estimation_max_angular_velocity = slip_estimation_max_angular_velocity

        self._lam = lam

        self._yaw_previous = None

        self._step = 0

        self._last_w_cmd = None

    def execute(self, state, input, index, delta_t):
        """! Execute the controller
        @param state<list>: The state of the vehicle
        @param input<list>: The input of the vehicle (unused)
        @param index<int>: Elapsed-tick counter from the node (unused --
        see class docstring).
        @param delta_t<float>: The time step
        @return<tuple>: The status and control
        """
        last_index = len(self.trajectory.u[0, :]) - 1

        nearest_index = self._search_nearest_index(state)

        if nearest_index >= last_index:
            return False, [0, 0]

        self._update_slip_estimate(state, delta_t)

        status, u = super(MPCRLS, self).execute(state, input, index, delta_t)

        # w actually commanded this tick (includes the MPC's own feedback
        # correction, not just the raw reference) -- see _update_slip_estimate.
        self._last_w_cmd = u[1]

        return status, u

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    def _update_slip_estimate(self, state, delta_t):
        """! Update the MPC's slip factor from the online RLS estimate.
        @param state<list>: The state of the vehicle
        @param delta_t<float>: The time step
        """
        unwrapped_yaw = np.unwrap([state[2]])[0]

        # See slip_estimation_max_angular_velocity in __init__: skip the
        # update entirely on a tick where the vehicle is mid-sharp-turn --
        # the deficit between commanded and measured rotation there is
        # dynamic lag, not persistent slip, and letting it drive the
        # estimate compounds into a runaway overcorrection right when the
        # trajectory is hardest to track.
        high_dynamics = (
            self._slip_estimation_max_angular_velocity is not None
            and self._last_w_cmd is not None
            and abs(self._last_w_cmd) > self._slip_estimation_max_angular_velocity)

        if self._yaw_previous is not None and self._last_w_cmd is not None and not high_dynamics:
            # The yaw change observed now is the effect of what was actually
            # commanded last tick (self._last_w_cmd) -- not the raw reference
            # angular velocity at the current nearest_index, which is what
            # was fed here previously. Using the raw reference instead of the
            # real applied command biases the estimate: any MPC correction
            # (or slip compensation elsewhere) that made the applied command
            # differ from the reference goes uncredited, systematically
            # pulling the slip estimate away from the true value.
            self._rls.predict_sim_with_forgetting_factor(
                yaw=unwrapped_yaw,
                yaw_previous=self._yaw_previous,
                ground_angular_velocity_z=self._last_w_cmd,
                delta_t=delta_t,
                lam=self._lam,
                position=(state[0], state[1]))

        self._yaw_previous = unwrapped_yaw

        # Slip is a wheel-speed deficit, not a surplus, so a negative fit is
        # measurement noise rather than a real effect -- floor it to 0.
        slip = float(np.clip(self._rls.estimates[-1][0, 0], 0.0, self._slip_clip))

        if self._step < self._warmup_steps:
            slip = 0.0

        self._step += 1

        self._mpc.s = slip
