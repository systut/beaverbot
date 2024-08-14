#!/usr/bin/env python3
##
# @file calibrate.py
#
# @brief Provide calibration for localization using GPS and IMU
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 08/13/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library
import math
from scipy.spatial.transform import Rotation

# External library
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import Datum
from geographic_msgs.msg import GeoPose, GeoPoint


class Calibrate(object):
    """! Calibration for localization using GPS and IMU

    The class provides calibration for localization using GPS and IMU.
    """
    # ==================================================================================================
    # PUBLIC METHODS
    # ==================================================================================================

    def __init__(self):
        """! Constructor"""
        super(Calibrate, self).__init__()

        rospy.init_node("calibrate")

        self._read_parameters()

        self._register_clients()

        self._poses = []

        self._gps_msg = None

    def run(self):
        """! Execute the node"""
        self._retrieve_gps_data()

        self._move_robot(0.1)

        self._retrieve_gps_data()

        msg = self._calculate_datum()

        self._datum_client(msg)

        self._move_robot(-0.1)

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    def _read_parameters(self):
        """! Read parameters
        """
        pass

    def _register_clients(self):
        """! Register controllers
        """
        self._datum_client = rospy.ServiceProxy(
            "/datum", Datum)

        try:
            rospy.loginfo("Waiting for service 'detect' to appear..")

            rospy.wait_for_service('datum', timeout=5.0)

            self._datum_client = rospy.ServiceProxy(
                "datum", Datum)

        except Exception as exception:
            rospy.logerr(exception)

    def _register_publishers(self):
        """! Register publishers
        """
        self._cmd_vel_publisher = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=10)

    def _register_subscribers(self):
        """! Register subscribers
        """
        _ = rospy.Subscriber(
            "/fix", NavSatFix, self._gps_callback)

    def _gps_callback(self, msg):
        """! GPS callback
        """
        self._gps_msg = msg

    def _retrieve_gps_data(self):
        """! Retrieve GPS data
        """
        while not rospy.is_shutdown():
            if not self._gps_msg:
                rospy.logwarn("No GPS message received")

                rospy.spin()

                continue

            self._poses.append(self._gps_msg)

            break

    def _move_robot(self, linear_velocity):
        """! Move robot
        @param linear_velocity Linear velocity
        """
        for _ in range(10):
            msg = Twist()

            msg.linear.x = linear_velocity

            self._cmd_vel_publisher.publish(msg)

            rospy.sleep(0.1)

    def _calculate_datum(self):
        """! Calculate datum
        """
        bearing = self._calculate_bearing()

        orientation = Rotation.from_euler('z', bearing).to_quaternion()

        return GeoPose(position=GeoPoint(
            latitude=self._poses[0].latitude,
            longitude=self._poses[0].longitude,
            altitude=self._poses[0].altitude
        ), orientation=orientation)

    def _calculate_bearing(self):
        """! Calculate bearing
        @return Bearing
        """
        first_latitude = math.radians(self._poses[0].latitude)
        second_latitude = math.radians(self._poses[1].latitude)

        difference_longitude = math.radians(
            self._poses[1].longitude - self._poses[0].longitude)

        x = math.sin(difference_longitude) * math.cos(second_latitude)
        y = math.cos(first_latitude) * math.sin(second_latitude)
        - (math.sin(first_latitude)
           * math.cos(second_latitude) * math.cos(difference_longitude))

        initial_bearing = math.atan2(x, y)

        east_bearing = initial_bearing - math.pi/2

        return east_bearing
