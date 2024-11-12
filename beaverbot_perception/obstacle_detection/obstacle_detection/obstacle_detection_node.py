#!/usr/bin/env python3
##
# @file obstacle_detection_node.py
#
# @brief Provide implementation of obstacle detection node.
#
# @section author_doxygen_example Author(s)
# - Created by Dinh Ngoc Duc on 24/10/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# External library
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan


class ObstacleDetectionNode(object):
    """! ObstacleDetectionNode class

    The class provides implementation of obstacle detection node.
    """

    def __init__(self):
        """! Constructor
        """
        rospy.init_node("obstacle_detection_node", anonymous=True)

        self._read_parameters()

        self._register_publishers()

        self._register_subscribers()

        self.emergency_stop_signal = Bool()

        self.emergency_stop_signal.data = False

    def run(self):
        """! Start ros node
        """
        while not rospy.is_shutdown():
            self.emergency_stop_publisher.publish(self.emergency_stop_signal)

            if self.emergency_stop_signal.data:
                rospy.logwarn('Emergency stop trigged !')

            else:
                rospy.loginfo("No obstacle in range")

            rospy.sleep(0.1)

    # ==========================================================================
    # PRIVATE METHODS
    # ==========================================================================
    def _read_parameters(self):
        """! Read parameters
        """
        self.obstacle_threshold = rospy.get_param("~obstacle_threshold", 0.1)

    def _register_publishers(self):
        """! Register publishers
        """
        self.emergency_stop_publisher = rospy.Publisher(
            "emergency_stop", Bool, queue_size=10)

    def _register_subscribers(self):
        """! Register subscribers
        """
        self.lidar_sub = rospy.Subscriber(
            "/scan", LaserScan, self._lidar_callback)

    def _lidar_callback(self, lidar_data):
        """! Lidar callback
        @param lidar_data<LaserScan>: The lidar data
        @return None
        """
        rospy.loginfo("Lidar data received")

        self.emergency_stop_signal.data = False

        for i, distance in enumerate(lidar_data.ranges):

            if distance <= self.obstacle_threshold:

                rospy.loginfo(
                    f'Obstacle detecte at angle {i}: {distance} m away')

                self.emergency_stop_signal.data = True

                return
