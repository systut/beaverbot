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

        rospy.init_node("robot_localization", anonymous=True)

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

    def _register_subscribers(self):
        """! Register ROS subscribers method
        """
        self._gps_sub = rospy.Subscriber(
            "/fix", NavSatFix, self._gps_callback)

        self._imu_sub = rospy.Subscriber(
            "/imu/data_raw", Imu, self._imu_callback)

    def _register_publishers(self):
        """! Register publishers method
        """
        self._rear_odom_pub = rospy.Publisher(
            "/hakuroukun_pose/rear_wheel_odometry", Odometry, queue_size=10)

        self._orientation_pub = rospy.Publisher(
            "/hakuroukun_pose/orientation", Float64, queue_size=1)

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

        while not rospy.is_shutdown() and (time.time() - start_time < 1):
            try:
                data = rospy.wait_for_message(
                    "/imu/data_raw", Imu, timeout=1.0)

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

        self._x_rear = self._x_gps - self._gps_to_rear_axis * \
            math.cos(self._yaw)

        self._y_rear = self._y_gps - self._gps_to_rear_axis * \
            math.sin(self._yaw)

    def _imu_callback(self, data: Imu):
        """! IMU callback method
        @param data: Imu message
        @return: yaw
        @ yaw: The yaw angle of the robot
        """
        self.quaternion_x = data.orientation.x
        self.quaternion_y = data.orientation.y
        self.quaternion_z = data.orientation.z
        self.quaternion_w = data.orientation.w

        self.angular_velocity_x = data.angular_velocity.x
        self.angular_velocity_y = data.angular_velocity.y
        self.angular_velocity_z = data.angular_velocity.z

        self.linear_acceleration_x = data.linear_acceleration.x
        self.linear_acceleration_y = data.linear_acceleration.y
        self.linear_acceleration_z = data.linear_acceleration.z

        euler = tf.transformations.euler_from_quaternion(
            [self.quaternion_x,
             self.quaternion_y,
             self.quaternion_z,
             self.quaternion_w])

        self._yaw = euler[2] - self._imu_offset

        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        (self.quaternion_x, self.quaternion_y,
         self.quaternion_z, self.quaternion_w) = tf.transformations. \
            quaternion_from_euler(0, 0, self._yaw)

    def _publish_rear_wheel_odometry(self, timer):
        """! Publish rear wheel pose method
        @param timer: Timer (unused)
        """
        rear_odom_msg = Odometry()
        rear_odom_msg.header.stamp = rospy.get_rostime()
        rear_odom_msg.header.frame_id = "base_link"

        rear_odom_msg.pose.pose.position.x = self._x_rear
        rear_odom_msg.pose.pose.position.y = self._y_rear
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
            (self._x_rear, self._y_rear, 0),
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
        rotation_angle = math.radians(rospy.get_param("~rotation_angle", 0.0))

        x_gps, y_gps = gc.ll2xy(lat, long, _initial_lat, _initial_lon)

        x_gps_local = x_gps * math.cos(rotation_angle) - y_gps * math.sin(
            rotation_angle) + self._gps_to_rear_axis * math.cos(self._yaw)

        y_gps_local = x_gps * math.sin(rotation_angle) + y_gps * math.cos(
            rotation_angle) + self._gps_to_rear_axis * math.sin(self._yaw)

        return x_gps_local, y_gps_local
