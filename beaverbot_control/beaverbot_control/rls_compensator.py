import csv

import rospy

import numpy as np

from beaverbot_control.rls_online import RLSOnline


class RLSCompensator:
    def __init__(self, trajectory, use_forgetting_factor=True,
                 forgetting_factor=0.98, slip_clip=0.3,
                 slip_estimation_source="yaw",
                 slip_estimation_max_angular_velocity=0.4, log_file=None):
        """
        Initialize the Recursive Least Squares (RLS) algorithm.

        Parameters:
        trajectory : namedtuple: The reference trajectory
        use_forgetting_factor : bool: If True, use
            RLSOnline.predict_sim_with_forgetting_factor each step;
            if False, use RLSOnline.predict_sim instead.
        forgetting_factor : float: The forgetting factor (lam) passed to
            predict_sim_with_forgetting_factor when use_forgetting_factor
            is True.
        slip_clip : float: The estimated slip is clipped to
            [0, slip_clip] (see execute()) -- matches MPCRLS's slip_clip.
        slip_estimation_source : str: Which measured signal the RLS update
            is regressed against each tick.
            "yaw" (default): finite-differences the fused pose's heading
            (state[2], from beaverbot_pose_node's GPS+IMU pose) each tick.
            Simple, but inherits that pose's own update characteristics
            (e.g. it only changes at the ~1 Hz GPS rate when dead
            reckoning is off, or holds briefly stale during a fusion
            correction).
            "yaw_rate": uses the IMU's directly measured angular_velocity_z
            instead, passed in as `input` to execute() -- a raw gyro
            reading updated at the IMU's own (much faster) rate,
            bypassing the fused pose/GPS entirely. Requires the caller to
            supply that reading via `input` each tick (see
            BeaverbotControl._measured_angular_velocity_callback, which
            reads it off /beaverbot_pose/odom's twist.twist.angular.z
            rather than subscribing to /imu directly).
        slip_estimation_max_angular_velocity : float or None: Skip the RLS
            update (hold the last estimate) on any tick where the last
            commanded |angular velocity| exceeds this. RLSOnline's model
            assumes a static gain (measured ~= commanded * (1-slip)); that
            assumption only holds once the vehicle has settled into the
            turn. Logged data shows the gap between commanded and
            IMU-measured angular velocity is ~0 (even slightly negative)
            for |w| < 0.3 rad/s, but grows to +0.13 rad/s for |w| > 0.5 --
            i.e. during a sharp turn the deficit is dynamic response
            lag/inertia, not persistent slip, and the correlation with
            |w| is 0.50, not a fixed offset. Feeding that into a static
            model produces a runaway feedback loop: slip is
            overestimated exactly when the reference calls for its
            sharpest turn, the resulting boost pushes the command (and
            thus the lag) higher still, and left unguarded this has been
            observed to overturn hard enough to lose the path entirely
            and never recover. Set to None to disable this gate.
        log_file : str or None: If set, the path of a CSV file to record
            the estimated slip (and the inputs it was derived from) at
            every step, for offline analysis. Recording is disabled when
            None.
        """
        self.trajectory = trajectory
        s0 = np.array([[0.0]])       # initial slip estimate
        P0 = np.eye(1) * 50.0      # large initial covariance
        R = np.eye(1) * 2 * 0.00436**2         # measurement noise
        self.rls = RLSOnline(s0, P0, R)
        self.yaw_previous = None
        self.first_step = True
        self.use_forgetting_factor = use_forgetting_factor
        self.forgetting_factor = forgetting_factor
        self.slip_clip = slip_clip
        self.slip_estimation_source = slip_estimation_source
        self.slip_estimation_max_angular_velocity = slip_estimation_max_angular_velocity
        self.log_file = log_file
        # w actually commanded on the previous execute() call (post slip
        # compensation) -- this, not the raw reference, is what the RLS
        # estimator's model assumes it's being told. See execute().
        self._last_w_cmd = None

        if self.log_file:
            with open(self.log_file, mode="w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["index", "delta_t", "yaw", "yaw_previous",
                                  "measured_angular_velocity_z",
                                  "last_commanded_angular_velocity",
                                  "update_skipped_high_dynamics", "raw_slip",
                                  "clipped_slip", "v", "w"])

    # writing method to implement online RLS and compensate the velocities from reference file
    def execute(self, state, input, index, delta_t):
        """! Execute the controller
        @param state<list>: The state of the vehicle
        @param input<float or None>: The IMU's measured angular_velocity_z
        for this tick -- only read when
        slip_estimation_source == "yaw_rate"; ignored (and may be None)
        otherwise.
        @param delta_t<float>: The time step
        @return<tuple>: The status and control
        """
        status = True
        if index >= len(self.trajectory.u[0, :]) - 1:
            return False, [0, 0]
        unwrapped_yaw = np.unwrap([state[2]])[0]
        yaw_previous = self.yaw_previous
        measured_angular_velocity_z = input if self.slip_estimation_source == "yaw_rate" else None

        # See slip_estimation_max_angular_velocity in __init__: skip the
        # update entirely on a tick where the vehicle is mid-sharp-turn --
        # the deficit between commanded and measured rotation there is
        # dynamic lag, not persistent slip, and letting it drive the
        # estimate compounds into a runaway overcorrection right when the
        # trajectory is hardest to track.
        high_dynamics = (
            self.slip_estimation_max_angular_velocity is not None
            and self._last_w_cmd is not None
            and abs(self._last_w_cmd) > self.slip_estimation_max_angular_velocity)

        if not self.first_step and not high_dynamics:
            # The yaw change (or, in "yaw_rate" mode, the measured angular
            # velocity) observed now is the effect of what was actually
            # commanded last tick (self._last_w_cmd) -- using the raw
            # reference trajectory.u[1, index] instead (as before) biases the
            # estimate, since it doesn't reflect the slip compensation that
            # was actually applied.
            if self.use_forgetting_factor:
                self.rls.predict_sim_with_forgetting_factor(
                    yaw=unwrapped_yaw,
                    yaw_previous=self.yaw_previous,
                    ground_angular_velocity_z=self._last_w_cmd,
                    delta_t=delta_t,
                    lam=self.forgetting_factor,
                    position=(state[0], state[1]),
                    measured_angular_velocity_z=measured_angular_velocity_z)
            else:
                self.rls.predict_sim(
                    yaw=unwrapped_yaw,
                    yaw_previous=self.yaw_previous,
                    ground_angular_velocity_z=self._last_w_cmd,
                    delta_t=delta_t,
                    position=(state[0], state[1]),
                    measured_angular_velocity_z=measured_angular_velocity_z)
        rospy.loginfo(f"Yaw: {unwrapped_yaw}, Yaw previous: {self.yaw_previous}"
                      f"Last commanded angular velocity: {self._last_w_cmd}")
        self.yaw_previous = unwrapped_yaw
        raw_slip = self.rls.estimates[-1][0, 0]
        # Slip is a wheel-speed deficit, not a surplus, so a negative fit is
        # measurement noise rather than a real effect -- floor it to 0
        # instead of letting it reduce v, w below the reference.
        slip = max(0.0, min(raw_slip, self.slip_clip))

        # Compensate the velocities  and angular velocities
        v = self.trajectory.u[0, index]/(1-slip)
        w = self.trajectory.u[1, index]/(1-slip)
        rospy.loginfo(f"Difference in velocities due to slip compensation: v:"
                      f"{v - self.trajectory.u[0, index]}, w: {w - self.trajectory.u[1, index]}, slip: {slip}")
        self._record_slip(index, delta_t, unwrapped_yaw, yaw_previous,
                           measured_angular_velocity_z, self._last_w_cmd,
                           high_dynamics, raw_slip, slip, v, w)
        self._last_w_cmd = w
        self.first_step = False
        return status, [v, w]

    def _record_slip(self, index, delta_t, yaw, yaw_previous,
                      measured_angular_velocity_z, last_commanded_angular_velocity,
                      update_skipped_high_dynamics, raw_slip, clipped_slip, v, w):
        """! Append one row of the estimated slip and its inputs to log_file
        @param index<int>: The trajectory index of this step
        @param delta_t<float>: The time step
        @param yaw<float>: The unwrapped yaw at this step
        @param yaw_previous<float>: The unwrapped yaw at the previous step
        @param measured_angular_velocity_z<float or None>: The IMU-measured
        angular velocity this row was regressed against when
        slip_estimation_source == "yaw_rate"; None in "yaw" mode.
        @param last_commanded_angular_velocity<float or None>: The angular
        velocity actually commanded on the previous step (None on the
        first step, before any command has been sent) -- this is the value
        the RLS update this row was regressed against.
        @param update_skipped_high_dynamics<bool>: True if this tick's RLS
        update was skipped because |last_commanded_angular_velocity|
        exceeded slip_estimation_max_angular_velocity (see __init__).
        @param raw_slip<float>: The slip estimate before clipping
        @param clipped_slip<float>: The slip estimate after clipping to
        [0, self.slip_clip]
        @param v<float>: The compensated linear velocity
        @param w<float>: The compensated angular velocity
        """
        if not self.log_file:
            return
        with open(self.log_file, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([index, delta_t, yaw, yaw_previous,
                              measured_angular_velocity_z,
                              last_commanded_angular_velocity,
                              update_skipped_high_dynamics, raw_slip,
                              clipped_slip, v, w])
    # from reference trajectory getting the reference velocities
