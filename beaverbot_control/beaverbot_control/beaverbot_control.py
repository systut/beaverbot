#!/usr/bin/env python3
##
# @file beavor_control_node.py
#
# @brief Provide implementation of beaverbot control node.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 08/12/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library
from collections import namedtuple

# External library
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Internal library
from beaverbot_control.pure_pursuit import PurePursuit
from beaverbot_control.feedforward import FeedForward


class BeaverbotControl(object):
    """! Beaverbot control node

    The class provides implementation of beaverbot control node.
    """
    # ==================================================================================================
    # PUBLIC METHODS
    # ==================================================================================================

    def __init__(self):
        """! Constructor"""
        super(BeaverbotControl, self).__init__()

        rospy.init_node("beaverbot_control")

        self._read_parameters()

        self._register_controllers()
        
        self._register_publishers()

        self._register_subscribers()

        self._register_timers()

    def run(self):
        """! Execute the node"""
        rospy.spin()

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    def _read_parameters(self):
        """! Read parameters
        """
        self._sampling_time = rospy.get_param(
            "~sampling_time", 0.1)

        self._controller_type = rospy.get_param(
            "~controller_type", "pure_pursuit")

        if not rospy.has_param("~trajectory_file"):
            raise Exception("No trajectory file is provided")

        self._trajectory_file = rospy.get_param(
            "~trajectory_file")

        self._is_derivative = rospy.get_param(
            "~is_derivative", True)

        self._state = None

        self._nu = 2

        self._nx = 3

        self._index = 0

    def _register_controllers(self):
        """! Register controllers
        """
        trajectory = self._generate_trajectory(
            self._trajectory_file, self._nx, self._nu, self._is_derivative)

        if self._controller_type == "pure_pursuit":
            self._controller = PurePursuit(trajectory)

        elif self._controller_type == "feedforward":
            self._controller = FeedForward(trajectory)

        else:
            raise NotImplementedError

    def _register_subscribers(self):
        """! Register subscriber
        """
        rospy.Subscriber("odom", Odometry, self._odom_callback)

    def _register_publishers(self):
        """! Register publisher
        """
        self._velocity_publisher = rospy.Publisher(
            "/beaverbot_diff_drive_controller/cmd_vel",
            Twist, queue_size=10)

    def _register_timers(self):
        """! Register timers
        """
        self._timer = rospy.Timer(rospy.Duration(self._sampling_time),
                                  self._timer_callback)

    def _odom_callback(self, msg):
        """! Odometry callback
        @param msg<Odometry>: The odometry message
        """
        self._state = [msg.pose.pose.position.x,
                       msg.pose.pose.position.y,
                       msg.pose.pose.orientation.z]

    def _timer_callback(self, event):
        """! Timer callback
        @param event<Event>: The event
        """
        if not self._state and self._controller in ["pure_pursuit"]:
            rospy.logwarn("No current status of the vehicle")

            return

        if not self._controller:
            rospy.logwarn("No controller is registered")

            return

        status, u = self._controller.execute(self._state, None, self._index)

        if not status:
            rospy.logwarn("Failed to execute controller")

            return

        msg = self._convert_control_input_to_msg(u)

        self._velocity_publisher.publish(msg)

        self._index += 1

    def _convert_control_input_to_msg(self, u):
        """! Convert control input to message
        @param u<list>: The control input
        @return<Twist>: The message
        """
        msg = Twist()

        if len(u) != 2:
            rospy.logwarn("Invalid control input")

            return msg

        msg.linear.x = u[0]

        msg.angular.z = u[1]

        return msg

    def _generate_trajectory(self, file_path, nx, nu, is_derivative=False):
        """! Generate a simple trajectory.
        @param file_path<str>: The file path of the
        generated trajectory
        @param nx<int>: The number of states
        @param nu<int>: The number of inputs
        @param is_derivative<bool>: The flag to indicate if the
        generated trajectory is a derivative
        @return None
        """
        trajectory = {}

        data = np.genfromtxt(file_path, delimiter=",")

        initial_index = 0

        if np.isnan(np.nan):
            initial_index = 1

        trajectory["x"] = np.array(data[initial_index:, 1: 1 + nx])

        if len(data) > 1 + nx:
            trajectory["u"] = self._retrieve_u(
                initial_index, data, nx, nu, is_derivative)

        trajectory["t"] = np.array(data[initial_index:, 0])

        trajectory["sampling_time"] = trajectory["t"][1] - trajectory["t"][0]

        trajectory_instance = namedtuple("Trajectory", trajectory.keys())(
            *trajectory.values())

        return trajectory_instance

    def _retrieve_u(self, initial_index, data, nx, nu, is_derivative):
        """! Retrieve the input at time t.
        @param t<float>: The time
        @return u<list>: The input
        """
        if not is_derivative:
            u = np.array(data[initial_index:, 1 + nx: 1 + nx + nu])

        else:
            u = np.zeros((self._nu, len(data) - initial_index))

            u[0, :] = np.hypot(
                np.array(data[initial_index:, 1 + nx: 1 + nx + 1]),
                np.array(data[initial_index:, 1 + nx + 1: 1 + nx + 2]),
            ).reshape(-1)

            u[1, :] = np.array(
                data[initial_index:, 1 + nx + 2: 1 + nx + 3]).reshape(-1)

        return u
