#!usr/bin/env python3
##
# @file rotation_angle_calibrator.py
#
# @brief Provide a field-test utility to derive beaverbot_pose's
# ~rotation_angle parameter.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard Libraries
import math

# External Libraries
import tf
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


class RotationAngleCalibrator:
    """! RotationAngleCalibrator class

    The class provides a field-test utility that derives beaverbot_pose's
    ~rotation_angle parameter by comparing the compass bearing of a GPS
    displacement against beaverbot_pose's reported yaw over that same
    displacement.

    This node does not command any robot motion -- drive or push the
    robot manually (as straight a line as possible, several meters) in
    between the two samples it records.
    """
    # ==========================================================================
    # PUBLIC METHODS
    # ==========================================================================

    def __init__(self):
        """! Constructor
        """
        super(RotationAngleCalibrator, self).__init__()

        rospy.init_node("rotation_angle_calibrator", anonymous=True)

        self._register_parameters()

        self._fix = None

        self._yaw = None

        self._register_subscribers()

    def run(self):
        """! Run the interactive field test
        """
        rospy.loginfo("Waiting for /fix and /beaverbot_pose/odom ...")

        self._wait_for_data()

        input("\nPosition the robot at the START of a straight run, "
              "keep it stationary, then press Enter to sample ...")

        start = self._sample("START")

        input("\nNow drive/push the robot in as straight a line as "
              "possible for several meters, stop, then press Enter to "
              "sample the END pose ...")

        end = self._sample("END")

        self._report(start, end)

    # ==========================================================================
    # PRIVATE METHODS
    # ==========================================================================
    def _register_parameters(self):
        """! Register ROS parameters method
        """
        self._initial_theta = rospy.get_param(
            "~initial_theta", 0.0)

        self._min_displacement = rospy.get_param(
            "~min_displacement", 2.0)

    def _register_subscribers(self):
        """! Register ROS subscribers method
        """
        rospy.Subscriber("/fix", NavSatFix, self._gps_callback)

        rospy.Subscriber(
            "/beaverbot_pose/odom", Odometry, self._odom_callback)

    def _gps_callback(self, data):
        """! GPS callback method
        @param data<NavSatFix>: The GPS fix message
        """
        self._fix = data

    def _odom_callback(self, data):
        """! Odometry callback method
        @param data<Odometry>: The beaverbot_pose odometry message
        """
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,
        )

        self._yaw = tf.transformations.euler_from_quaternion(quaternion)[2]

    def _wait_for_data(self):
        """! Block until both GPS and odometry data have arrived
        """
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and \
                (self._fix is None or self._yaw is None):
            rate.sleep()

    def _sample(self, label):
        """! Record one (lat, lon, yaw) sample
        @param label<str>: Label used in the printed confirmation
        @return<tuple>: (lat, lon, yaw)
        """
        lat, lon, yaw = self._fix.latitude, self._fix.longitude, self._yaw

        rospy.loginfo(
            f"{label}: lat={lat:.8f} lon={lon:.8f} "
            f"yaw={math.degrees(yaw):.2f} deg")

        return lat, lon, yaw

    def _report(self, start, end):
        """! Compute and print the recommended ~rotation_angle
        @param start<tuple>: (lat, lon, yaw) at the start of the run
        @param end<tuple>: (lat, lon, yaw) at the end of the run
        """
        lat0, lon0, yaw0 = start

        lat1, lon1, yaw1 = end

        distance = self._haversine_distance(lat0, lon0, lat1, lon1)

        if distance < self._min_displacement:
            rospy.logwarn(
                f"Displacement is only {distance:.2f} m (< "
                f"~min_displacement={self._min_displacement} m) -- GPS "
                "noise may dominate the bearing estimate. Consider "
                "driving further and re-running.")

        yaw_diff = math.atan2(
            math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))

        if abs(math.degrees(yaw_diff)) > 5.0:
            rospy.logwarn(
                f"Yaw changed by {math.degrees(yaw_diff):.2f} deg between "
                "samples -- the robot may not have driven in a straight "
                "line, which will bias the result.")

        yaw_avg = yaw0 + yaw_diff / 2.0

        bearing = self._bearing(lat0, lon0, lat1, lon1)

        rotation_angle = (bearing - 90.0 + math.degrees(yaw_avg)
                          - math.degrees(self._initial_theta))

        rotation_angle = (rotation_angle + 180.0) % 360.0 - 180.0

        rospy.loginfo(
            f"\nDisplacement: {distance:.2f} m"
            f"\nMeasured GPS bearing: {bearing:.2f} deg (0=N, 90=E, "
            "clockwise)"
            f"\nReported yaw (avg of start/end): "
            f"{math.degrees(yaw_avg):.2f} deg"
            f"\n~initial_theta currently in use: "
            f"{math.degrees(self._initial_theta):.2f} deg"
            f"\n\n==> Set ~rotation_angle to: {rotation_angle:.2f}")

    def _bearing(self, lat0, lon0, lat1, lon1):
        """! Compute the compass bearing (0=N, 90=E, clockwise) from
        (lat0, lon0) to (lat1, lon1)
        @param lat0<float>: Start latitude (deg)
        @param lon0<float>: Start longitude (deg)
        @param lat1<float>: End latitude (deg)
        @param lon1<float>: End longitude (deg)
        @return<float>: Bearing in degrees, [0, 360)
        """
        lat0_rad, lat1_rad = math.radians(lat0), math.radians(lat1)

        delta_lon = math.radians(lon1 - lon0)

        x = math.sin(delta_lon) * math.cos(lat1_rad)

        y = (math.cos(lat0_rad) * math.sin(lat1_rad)
            - math.sin(lat0_rad) * math.cos(lat1_rad) * math.cos(delta_lon))

        bearing = math.degrees(math.atan2(x, y))

        return bearing % 360.0

    def _haversine_distance(self, lat0, lon0, lat1, lon1):
        """! Compute great-circle distance between two lat/lon points
        @param lat0<float>: Start latitude (deg)
        @param lon0<float>: Start longitude (deg)
        @param lat1<float>: End latitude (deg)
        @param lon1<float>: End longitude (deg)
        @return<float>: Distance in meters
        """
        earth_radius = 6371000.0

        lat0_rad, lat1_rad = math.radians(lat0), math.radians(lat1)

        delta_lat = math.radians(lat1 - lat0)

        delta_lon = math.radians(lon1 - lon0)

        a = (math.sin(delta_lat / 2.0) ** 2 + math.cos(lat0_rad)
            * math.cos(lat1_rad) * math.sin(delta_lon / 2.0) ** 2)

        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

        return earth_radius * c
