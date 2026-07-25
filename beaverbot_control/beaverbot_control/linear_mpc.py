#!/usr/bin/env python3
##
# @file linear_mpc.py
#
# @brief Provide implementation of a linear model predictive controller
# (error-state, cvxpy/OSQP based) for autonomous driving.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 08/12/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# External library
import numpy as np
import cvxpy as cp


class LinearMPC:
    """! Linear MPC controller

    The class provides implementation of a linear model predictive
    controller that tracks a reference trajectory by solving a quadratic
    program over the differential-drive error-state dynamics, linearized
    around the reference trajectory at each timestep.
    """
    # ==================================================================================================
    # PUBLIC METHODS
    # ==================================================================================================
    def __init__(self, dt, wheel_base, N_horizon=10,
                 Q=None, R=None, Q_N=None, S=None,
                 vr_max=0.5, vl_max=0.5, s=0.0, du_max=0.05):
        """! Constructor
        @param dt<float>: Time step for the MPC controller.
        @param wheel_base<float>: Distance between the wheels of the robot.
        @param N_horizon<int>: Number of time steps in the prediction horizon.
        @param Q<np.ndarray>: State cost matrix.
        @param R<np.ndarray>: Control cost matrix.
        @param Q_N<np.ndarray>: Terminal state cost matrix.
        @param S<np.ndarray>: Input-change cost matrix penalizing
        ΔU[k] = v_commanded[k] - v_commanded[k-1]. None disables the cost term.
        @param vr_max<float>: Maximum velocity of the right wheel.
        @param vl_max<float>: Maximum velocity of the left wheel.
        @param s<float>: Slip factor for both wheels.
        @param du_max<float>: Maximum allowed change in each wheel's
        delta-velocity per step (slew-rate / acceleration constraint).
        None disables the constraint.
        """
        self.dt = dt

        self.l = wheel_base

        self.s = s

        self.N = N_horizon

        self.Q = Q if Q is not None else np.diag([50.0, 50.0, 10.0])

        self.R = R if R is not None else np.diag([0.3, 0.3])

        self.Q_N = Q_N if Q_N is not None else np.diag([50.0, 50.0, 10.0])

        self.S = S if S is not None else np.diag([1.0, 1.0])

        self.E = cp.Variable((N_horizon + 1, 3))

        self.U = cp.Variable((N_horizon, 2))

        self.E0 = cp.Parameter(3)

        self.A = [cp.Parameter((3, 3)) for _ in range(N_horizon)]

        self.B = [cp.Parameter((3, 2)) for _ in range(N_horizon)]

        self.VR_ref = cp.Parameter(N_horizon, value=np.zeros(N_horizon))

        self.VL_ref = cp.Parameter(N_horizon, value=np.zeros(N_horizon))

        self.VR_ref_prev = cp.Parameter(value=0.0)

        self.VL_ref_prev = cp.Parameter(value=0.0)

        self._vr_ref_prev = None

        self._vl_ref_prev = None

        self.U_prev = cp.Parameter(2, value=np.zeros(2))

        self.problem = self._build_problem(vr_max, vl_max, du_max)

    def solve(self, error_state, A_matrices, B_matrices, vr_ref_horizon, vl_ref_horizon):
        """! Solve the MPC optimization problem with the given error state
        and system matrices.
        @param error_state<np.ndarray>: The current state error of the
        robot [x_error, y_error, theta_error].
        @param A_matrices<list>: List of state transition matrices for
        each time step in the horizon.
        @param B_matrices<list>: List of control input matrices for each
        time step in the horizon.
        @param vr_ref_horizon<list>: Reference right wheel velocities
        over the prediction horizon.
        @param vl_ref_horizon<list>: Reference left wheel velocities
        over the prediction horizon.
        @return<tuple>: The computed velocity corrections
        (delta_vr, delta_vl) for the right and left wheels.
        """
        self.E0.value = error_state

        self.VR_ref.value = np.array(vr_ref_horizon)

        self.VL_ref.value = np.array(vl_ref_horizon)

        if self._vr_ref_prev is None:
            self._vr_ref_prev = vr_ref_horizon[0]

            self._vl_ref_prev = vl_ref_horizon[0]

        self.VR_ref_prev.value = self._vr_ref_prev

        self.VL_ref_prev.value = self._vl_ref_prev

        for i in range(self.N):
            self.A[i].value = A_matrices[i]

            self.B[i].value = B_matrices[i]

        self.problem.solve(solver=cp.OSQP, warm_start=True, eps_abs=1e-4, eps_rel=1e-4)

        if self.problem.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            print(f"MPC optimization problem not solved to optimality. Status: {self.problem.status}")

            return 0.0, 0.0

        delta_vr = self.U.value[0, 0]

        delta_vl = self.U.value[0, 1]

        self.U_prev.value = np.array([delta_vr, delta_vl])

        self._vr_ref_prev = vr_ref_horizon[0]

        self._vl_ref_prev = vl_ref_horizon[0]

        return delta_vr, delta_vl

    def define_AB_matrices(self, theta, vel_right, vel_left):
        """! Compute the A and B matrices for the linearized system.
        @param theta<float>: Reference orientation of the robot in radians
        (linearization is done around the reference trajectory).
        @param vel_right<float>: Reference velocity of the right wheel.
        @param vel_left<float>: Reference velocity of the left wheel.
        @return<tuple>: The state transition matrix A and control input
        matrix B.
        """
        s = self.s

        l = self.l

        dt = self.dt

        v_eff = (1 - s) * (vel_right + vel_left) / 2

        A_c = np.zeros((3, 3))

        A_c[0, 2] = -v_eff * np.sin(theta)

        A_c[1, 2] = v_eff * np.cos(theta)

        B_c = np.zeros((3, 2))

        B_c[0, 0] = (1 - s) * np.cos(theta) / 2

        B_c[0, 1] = (1 - s) * np.cos(theta) / 2

        B_c[1, 0] = (1 - s) * np.sin(theta) / 2

        B_c[1, 1] = (1 - s) * np.sin(theta) / 2

        B_c[2, 0] = (1 - s) / l

        B_c[2, 1] = -(1 - s) / l

        A_k = np.eye(3) + A_c * dt

        B_k = B_c * dt

        return A_k, B_k

    def compute_error_state(self, actual_state, reference_state):
        """! Compute the error state between the actual and reference
        states.
        @param actual_state<np.ndarray>: The current state of the robot
        [x, y, theta].
        @param reference_state<np.ndarray>: The desired state of the
        robot [x_ref, y_ref, theta_ref].
        @return<np.ndarray>: The error state [x_error, y_error, theta_error].
        """
        error_state = actual_state - reference_state

        error_state[2] = np.arctan2(np.sin(error_state[2]), np.cos(error_state[2]))

        return error_state

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    def _build_problem(self, vr_max, vl_max, du_max):
        """! Build the MPC optimization problem using cvxpy.
        @param vr_max<float>: Maximum velocity of the right wheel.
        @param vl_max<float>: Maximum velocity of the left wheel.
        @param du_max<float>: Maximum allowed change in each wheel's
        delta-velocity per step. None disables the constraint.
        @return<cvxpy.Problem>: The formulated MPC optimization problem.
        """
        cost = 0

        constraints = [self.E[0] == self.E0]

        for i in range(self.N):
            cost += cp.quad_form(self.E[i], self.Q)

            cost += cp.quad_form(self.U[i], self.R)

            constraints += [self.E[i + 1] == self.A[i] @ self.E[i] + self.B[i] @ self.U[i]]

            if self.S is not None:
                if i == 0:
                    dU = cp.hstack([
                        self.U[0, 0] + self.VR_ref[0] - self.U_prev[0] - self.VR_ref_prev,
                        self.U[0, 1] + self.VL_ref[0] - self.U_prev[1] - self.VL_ref_prev
                    ])
                else:
                    dU = cp.hstack([
                        self.U[i, 0] + self.VR_ref[i] - self.U[i - 1, 0] - self.VR_ref[i - 1],
                        self.U[i, 1] + self.VL_ref[i] - self.U[i - 1, 1] - self.VL_ref[i - 1]
                    ])

                cost += cp.quad_form(dU, self.S)

        cost += cp.quad_form(self.E[self.N], self.Q_N)

        constraints += [self.U[:, 0] + self.VR_ref <= vr_max,
                        self.U[:, 0] + self.VR_ref >= -vr_max,
                        self.U[:, 1] + self.VL_ref <= vl_max,
                        self.U[:, 1] + self.VL_ref >= -vl_max]

        if du_max is not None:
            constraints += [
                self.U[0, 0] + self.VR_ref[0] - self.U_prev[0] - self.VR_ref_prev <= du_max,
                self.U[0, 0] + self.VR_ref[0] - self.U_prev[0] - self.VR_ref_prev >= -du_max,
                self.U[0, 1] + self.VL_ref[0] - self.U_prev[1] - self.VL_ref_prev <= du_max,
                self.U[0, 1] + self.VL_ref[0] - self.U_prev[1] - self.VL_ref_prev >= -du_max,
            ]

            for k in range(1, self.N):
                constraints += [
                    self.U[k, 0] + self.VR_ref[k] - self.U[k - 1, 0] - self.VR_ref[k - 1] <= du_max,
                    self.U[k, 0] + self.VR_ref[k] - self.U[k - 1, 0] - self.VR_ref[k - 1] >= -du_max,
                    self.U[k, 1] + self.VL_ref[k] - self.U[k - 1, 1] - self.VL_ref[k - 1] <= du_max,
                    self.U[k, 1] + self.VL_ref[k] - self.U[k - 1, 1] - self.VL_ref[k - 1] >= -du_max,
                ]

        return cp.Problem(cp.Minimize(cost), constraints)
