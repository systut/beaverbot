#!usr/bin/env python3
##
# @file beaverbot_pose_node.py
#
# @brief Provide implementation of tractor-trailer system localization.
#
# @section author_doxygen_example Author(s)
# - Created by Dinh Ngoc Duc on 24/10/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard Libraries
import math
import time

# External Libraries
import tf
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix

# Internal Libraries
import geonav_transform.geonav_conversions as gc


class BeaverbotPoseNode:
    """! BeaverbotPoseNode class
    The class provides implementation of Hakuroukun pose node.
    """
    # ==========================================================================
    # PUBLIC METHODS
    # ==========================================================================

    def __init__(self):
        """! Constructor
        """
        super(BeaverbotPoseNode, self).__init__()

        rospy.init_node("robot_localization")

        self._register_parameters()

        self._get_initial_orientation()

        self._get_initial_pose()

        self._register_publishers()

        self._register_subscribers()

        self._register_log_data()

        rospy.sleep(1)

        self._register_timers()

    def run(self):
        """! Start ros node
        """
        rospy.spin()

    # ==========================================================================
    # PRIVATE METHODS
    # ==========================================================================
    def _register_parameters(self):
        """! Register ROS parameters method
        """
        self._log = rospy.get_param(
            "~log", True)

        self._publish_rate = rospy.get_param(
            "~publish_rate", 0.1)

        self._gps_to_rear_axis = rospy.get_param(
            "~gps_to_rear_axis", 0.0)

        self._imu_offset = rospy.get_param(
            "~imu_offset", 0.0)

        self._imu_epsilon = rospy.get_param(
            "~imu_epsilon", 0.0001)

        self._imu_calibration_threshold = rospy.get_param(
            "~imu_calibration_threshold", 200)

        self._initial_x = rospy.get_param(
            "~initial_x", 0.0)

        self._initial_y = rospy.get_param(
            "~initial_y", 0.0)

        self._initial_theta = rospy.get_param(
            "~initial_theta", 0.0)

        self._imu_calibration_timeout = rospy.get_param(
            "~imu_calibration_timeout", 15.0)

        self._last_gps_time = None

        self._encoder_linear_velocity = 0.0

        self._enable_position_prediction = rospy.get_param(
            "~enable_position_prediction", True)

        self._max_extrapolation_speed = rospy.get_param(
            "~max_extrapolation_speed", 0.5)

        self._max_imu_yaw_rate = rospy.get_param(
            "~max_imu_yaw_rate", 5.0)

        self._last_good_yaw = None

        self._last_imu_time = None

        self._last_predict_time = None

        rospy.loginfo(
            "beaverbot_pose_node initial pose params: "
            f"initial_x={self._initial_x}, initial_y={self._initial_y}, "
            f"initial_theta={self._initial_theta}")

    def _register_subscribers(self):
        """! Register ROS subscribers method
        """
        self._gps_sub = rospy.Subscriber(
            "/fix", NavSatFix, self._gps_callback)

        self._imu_sub = rospy.Subscriber(
            "/imu", Imu, self._imu_callback)

        self._encoder_odom_sub = rospy.Subscriber(
            "/beaverbot/odom", Odometry, self._encoder_odom_callback)

    def _register_publishers(self):
        """! Register publishers method
        """
        self._rear_odom_pub = rospy.Publisher(
            "/beaverbot_pose/odom", Odometry, queue_size=10)

        self._orientation_pub = rospy.Publisher(
            "/beaverbot_pose/orientation", Float64, queue_size=1)

        self._tf_broadcaster = tf.TransformBroadcaster()

    def _register_timers(self):
        """! Register timers method
        This method register the timer for publishing localization data
        with publish rate
        """
        rospy.Timer(rospy.Duration(self._publish_rate),
                    self._publish_rear_wheel_odometry)

        if self._log:

            rospy.Timer(rospy.Duration(self._publish_rate),
                        self._log_pose)

    def _register_log_data(self):
        """! Register log localization data method
        """
        self._log_start_time = None

        # log_folder = rospy.get_param("~log_folder", None)

        # current_time = datetime.now(pytz.timezone('Asia/Tokyo')).strftime(
        #     "position_log_%Y%m%d_%H-%M")

        # self._file_name = os.path.join(
        #     log_folder, current_time + ".csv")

        # with open(self._file_name, mode="a") as f:

        #     title = "Time (s), x_rear(m), y_rear(m), yaw(deg)\n"

        #     f.write(title)

    def _get_initial_pose(self):
        """! Get initial pose method
        This method will guarantee that data from GPS is received before
        the robot start moving
        """
        first_gps_mess = rospy.wait_for_message(
            '/fix', NavSatFix, timeout=10)

        rospy.loginfo("GPS Data Received")

        self._initial_lat = first_gps_mess.latitude

        self._initial_lon = first_gps_mess.longitude

        rospy.loginfo(
            "beaverbot_pose_node first GPS fix: "
            f"lat={first_gps_mess.latitude}, lon={first_gps_mess.longitude}, "
            f"status={first_gps_mess.status.status}")

    # def _get_initial_orientation(self):
    #     """! Get initial orientation
    #     THis method will guarantee that data from IMU is received before
    #     the robot start moving
    #     """
    #     rospy.wait_for_message('/imu/data_raw', Imu, timeout=10)

    #     rospy.loginfo("IMU Data Received")

    def _get_initial_orientation(self):
        """! Get initial orientation
        THis method will guarantee that data from IMU is received before
        the robot start moving
        """

        start_time = time.time()

        imu_data = []

        subtracted_values = []

        while not rospy.is_shutdown() and \
                (time.time() - start_time < self._imu_calibration_timeout):
            try:
                data = rospy.wait_for_message(
                    "/imu", Imu, timeout=1.0)

                euler = tf.transformations.euler_from_quaternion(
                    [data.orientation.x,
                     data.orientation.y,
                     data.orientation.z,
                     data.orientation.w])

                imu_data.append(euler[2])

                if len(imu_data) > 1:
                    difference = imu_data[-1] - imu_data[-2]

                    subtracted_values.append(difference)

                    if len(subtracted_values) > \
                            self._imu_calibration_threshold:
                        subtracted_values.pop(0)

                    if len(subtracted_values) == \
                            self._imu_calibration_threshold and \
                            all(val < self._imu_epsilon for val
                                in subtracted_values):
                        rospy.loginfo(
                            "Breaking out: last 200 differences are zero.")

                        self._imu_offset = euler[2]

                        break

                rospy.loginfo("Calibrating IMU ...")

            except rospy.ROSException:
                rospy.logwarn("No IMU message received within timeout.")

            self._yaw = 0.0

        # Shift the calibrated zero-heading by initial_theta so that the
        # published yaw reads initial_theta right after calibration,
        # instead of 0.
        self._imu_offset -= self._initial_theta

        rospy.loginfo("IMU data received.")

    def _gps_callback(self, data: NavSatFix):
        """! GPS callback method
        @param data: NavSatFix message
        @return: x_gps, y_gps, x_rear, y_rear
        @ x_gps: x position of the gps in the global frame
        @ y_gps: y position of the gps in the global frame
        @ x_rear: x position of the rear wheel in the global frame
        @ y_rear: y position of the rear wheel in the global frame
        """
        self._x_gps, self._y_gps = self._get_xy_from_latlon(
            data.latitude, data.longitude,
            self._initial_lat, self._initial_lon)

        x_rear = self._x_gps - self._gps_to_rear_axis * \
            math.cos(self._yaw) + self._initial_x

        y_rear = self._y_gps - self._gps_to_rear_axis * \
            math.sin(self._yaw) + self._initial_y

        now = rospy.Time.now()

        self._x_rear = x_rear

        self._y_rear = y_rear

        self._last_gps_time = now

        # Re-anchor the incremental dead-reckoning accumulator (see
        # _predict_pose) to this fresh, trusted fix -- otherwise it would
        # keep integrating forward from wherever it drifted to since the
        # previous fix instead of snapping back to ground truth.
        self._x_predicted = x_rear

        self._y_predicted = y_rear

        self._last_predict_time = now

    def _encoder_odom_callback(self, data: Odometry):
        """! Encoder odometry callback method
        @param data: Odometry message published by encoder_to_odom (see
        beaverbot_driver/src/beaverbot_driver/encoder_to_odom.cpp),
        derived from /encoder wheel-tick counts.

        Only twist.twist.linear.x (the wheel-derived forward speed, in
        the robot's body frame) is used, as the velocity source for
        _predict_pose's dead reckoning -- see that method's docstring for
        why this replaced the previous GPS-fix-to-fix finite difference.
        """
        self._encoder_linear_velocity = data.twist.twist.linear.x

    def _imu_callback(self, data: Imu):
        """! IMU callback method
        @param data: Imu message
        @return: yaw
        @ yaw: The yaw angle of the robot

        The IMU has been observed to intermittently freeze (repeat the
        exact same message, including angular_velocity/linear_acceleration,
        for anywhere from a fraction of a second up to several seconds)
        and then snap to a new orientation once it recovers -- a driver/
        hardware fault, not something fixable here. What this guards
        against: the snap itself implies a physically impossible yaw rate
        (tens of rad/s, versus this robot's real ~2 rad/s max), and
        applying it directly makes downstream consumers (MPC, RLS slip
        estimator) react to a heading change that never really happened.
        So the new yaw is only accepted if the implied rate since the
        last *accepted* reading is within ~max_imu_yaw_rate; otherwise the
        last good heading is kept and this message's orientation is
        dropped (angular_velocity/linear_acceleration are still applied
        either way -- only orientation was ever seen to actually corrupt
        the control loop).
        """
        self.angular_velocity_x = data.angular_velocity.x
        self.angular_velocity_y = data.angular_velocity.y
        self.angular_velocity_z = data.angular_velocity.z

        self.linear_acceleration_x = data.linear_acceleration.x
        self.linear_acceleration_y = data.linear_acceleration.y
        self.linear_acceleration_z = data.linear_acceleration.z

        euler = tf.transformations.euler_from_quaternion(
            [data.orientation.x,
             data.orientation.y,
             data.orientation.z,
             data.orientation.w])

        yaw = euler[2] - self._imu_offset

        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        now = rospy.Time.now()

        if self._last_good_yaw is not None and self._last_imu_time is not None:
            dt = (now - self._last_imu_time).to_sec()

            yaw_diff = math.atan2(math.sin(yaw - self._last_good_yaw),
                                   math.cos(yaw - self._last_good_yaw))

            if dt > 1e-3 and abs(yaw_diff) / dt > self._max_imu_yaw_rate:
                rospy.logwarn(
                    f"Rejecting IMU orientation: implied yaw rate "
                    f"{abs(yaw_diff) / dt:.2f} rad/s exceeds "
                    f"~max_imu_yaw_rate ({self._max_imu_yaw_rate} rad/s); "
                    f"keeping last accepted heading.")

                return

        self._last_good_yaw = yaw

        self._last_imu_time = now

        self._yaw = yaw

        (self.quaternion_x, self.quaternion_y,
         self.quaternion_z, self.quaternion_w) = tf.transformations. \
            quaternion_from_euler(0, 0, self._yaw)

    def _predict_pose(self):
        """! Dead-reckon (x_rear, y_rear) forward from the last GPS fix to
        now, using the wheel-encoder-derived forward speed rotated into
        the world frame by the current (IMU) yaw. /fix only publishes at
        ~1 Hz, far slower than this node's publish timer, so without this
        the published odom position would stay frozen for several
        consecutive publishes after every fix. Yaw (from IMU) is not
        similarly delayed -- _imu_callback updates it on every IMU
        message, independent of this method.

        Previously the extrapolation velocity was estimated by
        finite-differencing consecutive GPS fixes (~1 Hz, so effectively
        one velocity sample per second, noisy for a slow robot and stale
        for the whole gap). /beaverbot/odom (published by encoder_to_odom
        from /encoder wheel ticks) updates far faster and reflects the
        robot's actual current speed rather than an average over the last
        ~1 s, so _encoder_odom_callback's velocity is used instead;
        assumes the robot moves along its heading (v*cos(yaw), v*sin(yaw)),
        matching encoder_to_odom's own nonholonomic (y_dot_b_ = 0)
        assumption.

        Displacement is integrated incrementally, one small publish-timer
        tick (~publish_rate, default 0.1 s) at a time, accumulating into
        self._x_predicted/self._y_predicted -- not recomputed from
        scratch each call as (this instant's velocity) * (total time
        since the last fix). That "recompute from scratch" approach was
        tried twice and both versions shared the same underlying flaw:
        multiplying a single instantaneous velocity sample by up to a
        full ~1 s means whatever the encoder happens to read *right now*
        retroactively stands in for the whole gap. A saturating
        time-constant ceiling on top of that (the first version) bounded
        the damage but also made the prediction silently stop advancing
        partway through every gap, causing a once-per-fix catch-up jump.
        Removing the ceiling (the second version) removed the jump but
        made the flaw worse: a single momentary low/zero encoder reading
        -- normal during a turn, when forward speed legitimately dips --
        now zeroed out the *entire* displacement estimate since the last
        fix instead of just one small tick's worth, so the published
        position could stall near the last fix for the whole gap while
        yaw kept updating from the IMU every tick, i.e. the robot reads
        as rotating in place. Integrating tick-by-tick fixes both: each
        tick only ever contributes that tick's own small dt, so a
        momentary bad sample costs one small increment, not the whole
        window, while genuine sustained motion still accumulates properly
        across the gap. Disabled via ~enable_position_prediction (default
        true) -- when false, returns the raw last-fix position unchanged.
        @return<tuple>: The predicted (x_rear, y_rear)
        """
        if not self._enable_position_prediction or self._last_gps_time is None:
            return self._x_rear, self._y_rear

        now = rospy.Time.now()

        dt = (now - self._last_predict_time).to_sec()

        self._last_predict_time = now

        speed = abs(self._encoder_linear_velocity)

        if speed > self._max_extrapolation_speed and speed > 0:
            speed = self._max_extrapolation_speed * math.copysign(
                1.0, self._encoder_linear_velocity)
        else:
            speed = self._encoder_linear_velocity

        vx = speed * math.cos(self._yaw)

        vy = speed * math.sin(self._yaw)

        self._x_predicted += vx * dt

        self._y_predicted += vy * dt

        return self._x_predicted, self._y_predicted

    def _publish_rear_wheel_odometry(self, timer):
        """! Publish rear wheel pose method
        @param timer: Timer (unused)
        """
        x_rear, y_rear = self._predict_pose()

        rear_odom_msg = Odometry()
        rear_odom_msg.header.stamp = rospy.get_rostime()
        rear_odom_msg.header.frame_id = "base_link"

        rear_odom_msg.pose.pose.position.x = x_rear
        rear_odom_msg.pose.pose.position.y = y_rear
        rear_odom_msg.pose.pose.position.z = 0.0
        rear_odom_msg.pose.pose.orientation.x = self.quaternion_x
        rear_odom_msg.pose.pose.orientation.y = self.quaternion_y
        rear_odom_msg.pose.pose.orientation.z = self.quaternion_z
        rear_odom_msg.pose.pose.orientation.w = self.quaternion_w

        rear_odom_msg.twist.twist.angular.x = self.angular_velocity_x
        rear_odom_msg.twist.twist.angular.y = self.angular_velocity_y
        rear_odom_msg.twist.twist.angular.z = self.angular_velocity_z

        self._rear_odom_pub.publish(rear_odom_msg)

        self._tf_broadcaster.sendTransform(
            (x_rear, y_rear, 0),
            (self.quaternion_x, self.quaternion_y,
             self.quaternion_z, self.quaternion_w),
            rospy.Time.now(),
            "base_link",
            "map"
        )

    def _log_pose(self, timer):
        """! Log pose method
        @param timer: Timer (unused)
        """
        if self._log_start_time is None:
            self._log_start_time = time.time()

        elapsed_time = (time.time() - self._log_start_time)

        pose = f"{elapsed_time}, {self._x_rear}, {self._y_rear}, \
            {math.degrees(self._yaw)}"

        rospy.loginfo(f"Pose: {pose}")

        # with open(self._file_name, mode="a") as f:

        #     f.write(pose + "\n")

    def _get_xy_from_latlon(self, lat, long, _initial_lat, _initial_lon):
        """! Get x, y from latitude and longitude method
        @param latitude: Latitude of the robot
        @param longitude: Longitude of the robot
        @param _initial_lat: Initial latitude
        @param _initial_lon: Initial longitude

        @return: x_gps_local, y_gps_local
        @ x_gps_local: x position of the gps in the local frame
        @ y_gps_local: y position of the gps in the local frame
        """
        # initial_theta rotates the GPS (x, y) frame by the same angle the
        # yaw reference is shifted by (see _get_initial_orientation), so
        # that dx/dt = v*cos(yaw), dy/dt = v*sin(yaw) stays consistent in
        # the aligned frame.
        rotation_angle = math.radians(
            rospy.get_param("~rotation_angle", 0.0)) + self._initial_theta

        x_gps, y_gps = gc.ll2xy(lat, long, _initial_lat, _initial_lon)

        x_gps_local = x_gps * math.cos(rotation_angle) - y_gps * math.sin(
            rotation_angle) + self._gps_to_rear_axis * math.cos(self._yaw)

        y_gps_local = x_gps * math.sin(rotation_angle) + y_gps * math.cos(
            rotation_angle) + self._gps_to_rear_axis * math.sin(self._yaw)

        return x_gps_local, y_gps_local
