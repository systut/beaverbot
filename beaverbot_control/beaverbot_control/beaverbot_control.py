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
from nav_msgs.msg import Odometry, Path
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist, PoseStamped

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

        self._register_publishers()

        self._register_controllers()

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

        self._trajectory_type = rospy.get_param(
            "~trajectory_type", "derivative")

        self._state = None

        self._nu = 2

        self._nx = 3

        self._index = 0

        self._length_base = 0.53

    def _register_controllers(self):
        """! Register controllers
        """
        trajectory = self._generate_trajectory(
            self._trajectory_file, self._nx, self._nu, self._trajectory_type)

        if self._controller_type == "pure_pursuit":
            self._controller = PurePursuit(trajectory)

        elif self._controller_type == "feedforward":
            self._controller = FeedForward(trajectory)

        else:
            raise NotImplementedError

    def _register_subscribers(self):
        """! Register subscriber
        """
        rospy.Subscriber("odometry/filtered/global", Odometry,
                         self._odom_callback)

    def _register_publishers(self):
        """! Register publisher
        """
        self._velocity_publisher = rospy.Publisher(
            "cmd_vel",
            Twist, queue_size=10)

        self._trajectory_publisher = rospy.Publisher(
            "reference_trajectory", Path, queue_size=10)

    def _register_timers(self):
        """! Register timers
        """
        self._timer = rospy.Timer(rospy.Duration(self._sampling_time),
                                  self._timer_callback)

    def _odom_callback(self, msg):
        """! Odometry callback
        @param msg<Odometry>: The odometry message
        """
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

        heading = Rotation.from_quat(quaternion).as_euler(
            "zyx", degrees=False)[0]

        self._state = [msg.pose.pose.position.x,
                       msg.pose.pose.position.y,
                       heading]

    def _timer_callback(self, event):
        """! Timer callback
        @param event<Event>: The event
        """
        if not self._state and self._controller_type in ["pure_pursuit"]:
            rospy.logwarn("No current status of the vehicle")

            return

        if not self._controller:
            rospy.logwarn("No controller is registered")

            return

        status, u = self._controller.execute(self._state, None, self._index)

        if not status:
            rospy.logwarn("Failed to execute controller")

            return

        if self._is_goal(self._state, self._controller.trajectory):
            rospy.loginfo("The vehicle has reached the goal")

            u = [0, 0]

        msg = self._convert_control_input_to_msg(u)

        rospy.loginfo(f"Send control input {msg}")

        self._velocity_publisher.publish(msg)

        self._index += 1

    def _is_goal(self, state, trajectory):
        """! Check if the vehicle has reached the goal
        @param state<list>: The state of the vehicle
        @param trajectory<instance>: The trajectory
        @return<bool>: The flag to indicate if the vehicle has reached the goal
        """
        delta_x = trajectory.x[-1, 0] - state[0]

        delta_y = trajectory.x[-1, 1] - state[1]

        distance = np.hypot(delta_x, delta_y)

        return distance < 0.1

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

    def _generate_trajectory(self, file_path, nx, nu, trajectory_type=False):
        """! Generate a simple trajectory.
        @param file_path<str>: The file path of the
        generated trajectory
        @param nx<int>: The number of states
        @param nu<int>: The number of inputs
        @param trajectory_type<bool>: The flag to indicate if the
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
                initial_index, data, nx, nu, trajectory_type)

        trajectory["t"] = np.array(data[initial_index:, 0])

        trajectory["sampling_time"] = trajectory["t"][1] - trajectory["t"][0]

        trajectory_instance = namedtuple("Trajectory", trajectory.keys())(
            *trajectory.values())

        self._visualize_trajectory(trajectory_instance)

        return trajectory_instance

    def _visualize_trajectory(self, trajectory):
        """! Visualize the trajectory
        @param trajectory<list>: The trajectory
        """
        path = Path()

        path.header.frame_id = "map"

        for index in range(len(trajectory.x)):
            pose = PoseStamped()

            pose.header.frame_id = "map"

            pose.pose.position.x = trajectory.x[index, 0]

            pose.pose.position.y = trajectory.x[index, 1]

            path.poses.append(pose)

        for _ in range(10):
            self._trajectory_publisher.publish(path)

    def _retrieve_u(self, initial_index, data, nx, nu, trajectory_type):
        """! Retrieve the input at time t.
        @param t<float>: The time
        @return u<list>: The input
        """
        if trajectory_type == "normal":
            u = np.transpose(np.array(
                data[initial_index:, 1 + nx: 1 + nx + nu]))

        elif trajectory_type == "derivative":
            u = np.zeros((self._nu, len(data) - initial_index))

            u[0, :] = np.hypot(
                np.array(data[initial_index:, 1 + nx: 1 + nx + 1]),
                np.array(data[initial_index:, 1 + nx + 1: 1 + nx + 2]),
            ).reshape(-1)

            u[1, :] = np.array(
                data[initial_index:, 1 + nx + 2: 1 + nx + 3]).reshape(-1)

        elif trajectory_type == "wheel":
            u = np.zeros((self._nu, len(data) - initial_index))

            u[0, :] = (
                np.array(data[initial_index:, 1 + nx: 1 + nx + 1]) +
                np.array(data[initial_index:, 1 + nx + 1: 1 + nx + 2])
            ).reshape(-1) / 2

            u[1, :] = (
                np.array(data[initial_index:, 1 + nx: 1 + nx + 1]) -
                np.array(data[initial_index:, 1 + nx + 1: 1 + nx + 2])
            ).reshape(-1) / self._length_base

        return u
