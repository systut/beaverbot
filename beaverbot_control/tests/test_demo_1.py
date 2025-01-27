#!/usr/bin/env python3
##
# @file test_demo_1.py
#
# @brief Provide test demo 1 for trailer tractor system.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 2025/01/28.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# External library
import rospy
from geometry_msgs.msg import Twist


class TestDemo1(object):
    """! Test demo 1
    """
    def __init__(self):
        """! Constructor
        """
        super(TestDemo1, self).__init__()

        rospy.init_node("beaverbot_control")

        self._cmd_vel_publisher = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=1)

        self._cmd_vel_message = Twist()

    def test_demo_1(self):
        """! Test demo 1
        """
        # TODO: Replace the following line with needed speed
        move_forward_speed = 0.5
        self._cmd_vel_message.linear.x = move_forward_speed
        self._cmd_vel_message.angular.z = move_forward_speed

        self._cmd_vel_publisher.publish(self._cmd_vel_message)

        # TODO: Replace the following line with rospy.sleep(5.0)
        rospy.sleep(5.0)

        self._cmd_vel_message.linear.x = 0.0
        self._cmd_vel_message.angular.z = 0.0

        self._cmd_vel_publisher.publish(self._cmd_vel_message)

        rospy.sleep(2.0)

        self._cmd_vel_message.linear.x = move_forward_speed
        self._cmd_vel_message.angular.z = move_forward_speed

        self._cmd_vel_publisher.publish(self._cmd_vel_message)

        rospy.sleep(5.0)

        self._cmd_vel_message.linear.x = 0.0
        self._cmd_vel_message.angular.z = 0.0

        self._cmd_vel_publisher.publish(self._cmd_vel_message)


if __name__ == "__main__":
    test_demo_1 = TestDemo1()

    test_demo_1.test_demo_1()
