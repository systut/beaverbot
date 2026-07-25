import numpy as np
import math
import rospy


#class rls online

class RLSOnline:
    def __init__(self, s0, P0, R, excitation_threshold=1e-3, covariance_ceiling=None,
                 motion_position_threshold=0.002, motion_yaw_threshold=0.005):
        """
        Initialize the Recursive Least Squares (RLS) algorithm.

        Parameters:
        s0 : np.ndarray
            Initial state estimate.
        P0 : np.ndarray
            Initial error covariance matrix.
        R : np.ndarray
            Measurement noise covariance matrix.
        excitation_threshold : float
            predict_sim_with_forgetting_factor only inflates the
            covariance (the `* (1/lam)` forgetting term) when the
            regressor C = delta_t * ground_angular_velocity_z has at
            least this much magnitude. Below it (e.g. w_ref ~ 0, which a
            lemniscate passes through at its inflection points), there's
            no new information to justify growing the covariance, so
            skipping the inflation there prevents "covariance windup":
            P growing unbounded through a low-excitation stretch and then
            causing a wildly oversized correction the moment a real
            measurement arrives again.
        covariance_ceiling : float or None
            Hard cap applied to the covariance every step regardless of
            excitation, as a backstop. Defaults to P0's own magnitude
            (P0 is already deliberately "large" to allow fast initial
            convergence, so letting P grow past it during operation is
            the windup condition itself).
        motion_position_threshold : float
            predict_sim/predict_sim_with_forgetting_factor skip the
            update entirely when neither position nor yaw has moved by
            at least this much (meters) / motion_yaw_threshold (radians)
            since the last call. Without this, a stationary vehicle with
            a nonzero commanded angular velocity (e.g. before trajectory
            playback has actually started moving) reads as ~100% slip --
            "commanded to turn, measured no rotation" -- and the filter
            converges toward that wrong value the whole time it's still,
            becoming confident (low covariance, small gain) and thus slow
            to unlearn it once real motion begins.
        motion_yaw_threshold : float
            See motion_position_threshold.
        """
        self.s0 = s0
        self.P0 = P0
        self.R = R
        self.excitation_threshold = excitation_threshold
        self.covariance_ceiling = (
            covariance_ceiling if covariance_ceiling is not None
            else float(np.max(P0)))
        self.motion_position_threshold = motion_position_threshold
        self.motion_yaw_threshold = motion_yaw_threshold
        self._position_previous = None
        #the list where the estimates of the slip will be stored
        self.estimates = []
        self.estimates.append(s0)
        # the list where the estimation error covariance matrices will be stored Pk
        self.estimationErrorCovarianceMatrices = []
        self.estimationErrorCovarianceMatrices.append(P0)
        #the list where the Kalman gain matrices will be stored Kk
        self.gainMatrices = []
        # the list where the estimation errors will be stored ek
        self.errors = []

        self.yaw_diff = []
        self.angular_vel_z = []

        # this variable is used to track the current time step k of the estimator
        # after every time step arrives, this variables increases for one 
        # in this way, we can track the number of variblaes
        self.previousTimeStep=0

    def _detect_motion(self, position, yaw_diff):
        """! Whether the vehicle shows any real motion since the last call
        to predict_sim/predict_sim_with_forgetting_factor, used to gate
        those updates (see motion_position_threshold in __init__).
        @param position<tuple or None>: Current (x, y), or None to skip
        the position check and rely on yaw alone.
        @param yaw_diff<float>: The wrapped yaw change since the last call.
        @return<bool>: True if position or heading moved enough to trust
        this measurement as real motion.
        """
        yaw_moved = abs(yaw_diff) > self.motion_yaw_threshold

        if position is None or self._position_previous is None:
            return yaw_moved

        dx = position[0] - self._position_previous[0]
        dy = position[1] - self._position_previous[1]
        position_moved = math.hypot(dx, dy) > self.motion_position_threshold

        return yaw_moved or position_moved

    #writing method to estimate slip from simulation data
    def predict_sim(self, yaw, yaw_previous, ground_angular_velocity_z, delta_t,
                     position=None, measured_angular_velocity_z=None):
            """
            First calculating the theta difference and the angular velocity

            measured_angular_velocity_z, when given, is the IMU's directly
            measured yaw rate for this tick -- used in place of
            finite-differencing yaw/yaw_previous (see
            RLSCompensator's slip_estimation_source="yaw_rate"). This bypasses
            the fused pose entirely, so it isn't subject to that pose's own
            staleness characteristics (GPS-rate position holds, etc.).
            """
            if measured_angular_velocity_z is not None:
                yaw_diff = np.array([measured_angular_velocity_z * delta_t])
            else:
                yaw_diff= np.array([yaw - yaw_previous])
                yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))  # wrap to [-pi, pi]

            motion = self._detect_motion(position, float(yaw_diff[0]))
            self._position_previous = position
            if not motion:
                return

            C = np.array([delta_t * ground_angular_velocity_z ])
            #Calculating L matrix and its inverse
            L_matrix = self.R + np.matmul(C, np.matmul(self.estimationErrorCovarianceMatrices[self.previousTimeStep], C.T))
            L_matrix_inverse = np.linalg.inv(L_matrix)            

            #Calculating the Kalman gain matrix
            gain_matrix = np.matmul(self.estimationErrorCovarianceMatrices[self.previousTimeStep], np.matmul(C.T, L_matrix_inverse))

            #Calculating the estimation error(correction term (yk -Cxk))
            error = (C-yaw_diff) - np.matmul(C, self.estimates[self.previousTimeStep])
            rospy.loginfo(f"Error in RLS: {error}")
            #Calculating the new estimate
            estimate = self.estimates[self.previousTimeStep] + np.matmul(gain_matrix, error)

            #Calculating the new estimation error covariance matrix
            ImKc = np.eye(np.size(self.s0), np.size(self.s0)) - np.matmul(gain_matrix, C)
            estimationErrorCovarianceMatrix = np.matmul(ImKc, self.estimationErrorCovarianceMatrices[self.previousTimeStep])

            #Storing the results
            self.estimates.append(estimate)
            self.estimationErrorCovarianceMatrices.append(estimationErrorCovarianceMatrix)
            self.gainMatrices.append(gain_matrix)
            self.errors.append(error)
            self.yaw_diff.append(yaw_diff)
            self.angular_vel_z.append(ground_angular_velocity_z)

            # increase the time step
            self.previousTimeStep = self.previousTimeStep + 1
    
    def predict_sim_with_forgetting_factor(self, yaw, yaw_previous, ground_angular_velocity_z, delta_t,
                                            lam = 0.98, position=None, measured_angular_velocity_z=None):
            """
            First calculating the theta difference and the angular velocity

            measured_angular_velocity_z, when given, is the IMU's directly
            measured yaw rate for this tick -- used in place of
            finite-differencing yaw/yaw_previous (see
            RLSCompensator's slip_estimation_source="yaw_rate"). This bypasses
            the fused pose entirely, so it isn't subject to that pose's own
            staleness characteristics (GPS-rate position holds, etc.).
            """
            if measured_angular_velocity_z is not None:
                yaw_diff = np.array([measured_angular_velocity_z * delta_t])
            else:
                yaw_diff= np.array([yaw - yaw_previous])
                yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))  # wrap to [-pi, pi]

            motion = self._detect_motion(position, float(yaw_diff[0]))
            self._position_previous = position
            if not motion:
                return

            C = np.array([delta_t * ground_angular_velocity_z ])
            #Calculating L matrix and its inverse
            L_matrix = self.R + np.matmul(C, np.matmul(self.estimationErrorCovarianceMatrices[self.previousTimeStep], C.T))
            L_matrix_inverse = np.linalg.inv(L_matrix)            

            #Calculating the Kalman gain matrix
            gain_matrix = np.matmul(self.estimationErrorCovarianceMatrices[self.previousTimeStep], np.matmul(C.T, L_matrix_inverse))

            #Calculating the estimation error(correction term (yk -Cxk))
            error = (C-yaw_diff) - np.matmul(C, self.estimates[self.previousTimeStep])
            rospy.loginfo(f"Error in RLS: {error}")
            #Calculating the new estimate
            estimate = self.estimates[self.previousTimeStep] + np.matmul(gain_matrix, error)

            #Calculating the new estimation error covariance matrix
            ImKc = np.eye(np.size(self.s0), np.size(self.s0)) - np.matmul(gain_matrix, C)
            estimationErrorCovarianceMatrix = np.matmul(ImKc, self.estimationErrorCovarianceMatrices[self.previousTimeStep])

            # Only forget (inflate P by 1/lam) when C carries enough excitation
            # to justify it; otherwise this would keep inflating P unopposed
            # through a low-excitation stretch (windup), producing a hugely
            # oversized correction the moment a real measurement returns. The
            # ceiling is a hard backstop regardless of excitation.
            if np.linalg.norm(C) > self.excitation_threshold:
                estimationErrorCovarianceMatrix = estimationErrorCovarianceMatrix * (1 / lam)

            estimationErrorCovarianceMatrix = np.minimum(
                estimationErrorCovarianceMatrix, self.covariance_ceiling)

            #Storing the results
            self.estimates.append(estimate)
            self.estimationErrorCovarianceMatrices.append(estimationErrorCovarianceMatrix)
            self.gainMatrices.append(gain_matrix)
            self.errors.append(error)
            self.yaw_diff.append(yaw_diff)
            self.angular_vel_z.append(ground_angular_velocity_z)

            # increase the time step
            self.previousTimeStep = self.previousTimeStep + 1