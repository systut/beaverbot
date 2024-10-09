#!/usr/bin/env python3
##
# @file pure_pursuit.py
#
# @brief Provide implementation of pure pursuit controller for
# autonomous driving.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 08/12/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library
import math
import numpy as np


class PurePursuit:
    """! Pure Pursuit controller

    The class provides implementation of pure pursuit controller for
    autonomous driving.
    """

    lookahead_distance = 0.5

    lookahead_gain = 0.1

    k = 1

    # ==================================================================================================
    # PUBLIC METHODS
    # ==================================================================================================
    def __init__(self, trajectory):
        """! Constructor
        @param trajectory<instance>: The trajectory
        """
        self.trajectory = trajectory

        self.old_nearest_point_index = None

    def execute(self, state, input, previous_index):
        """! Execute the controller
        @param state<list>: The state of the vehicle
        @param input<list>: The input of the vehicle
        @param previous_index<int>: The previous index
        @return<tuple>: The status and control
        """
        status = True

        index, lookahead_distance = self._search_target_index(state, input)

        if previous_index >= index:
            index = previous_index

        if index < len(self.trajectory.x):
            trajectory_x = self.trajectory.x[index, 0]

            trajectory_y = self.trajectory.x[index, 1]

        else:
            trajectory_x = self.trajectory.x[-1, 0]

            trajectory_y = self.trajectory.x[-1, 1]

            index = len(self.trajectory.x) - 1

        alpha = (
            math.atan2(
                trajectory_y - state[1],
                trajectory_x - state[0],
            )
            - state[2]
        )

        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        v = 0.3

        w = v * 2.0 * alpha / lookahead_distance

        return status, [v, w]

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    def _apply_proportional_control(k_p, target, current):
        """! Apply proportional control
        @param k_p<float>: The proportional gain
        @param target<list>: The target
        @param current<list>: The current
        @return<float>: The control
        """
        target_v = (target[0] + target[1]) / 2

        v = (current[0] + current[1]) / 2

        a = k_p * (target_v - v)

        return a

    def _search_target_index(self, state, input):
        if self.old_nearest_point_index is None:
            all_distance = self._calculate_distance(self.trajectory.x, state)

            index = np.argmin(all_distance)

        else:
            index = self.old_nearest_point_index

            this_distance = self._calculate_distance(
                self.trajectory.x[index], state)

            while True:
                next_distance = self._calculate_distance(
                    self.trajectory.x[index + 1], state
                )

                if this_distance < next_distance:
                    break

                if (index + 1) < len(self.trajectory.x):
                    index += 1

                this_distance = next_distance

            self.old_nearest_point_index = index

        lookahead_distance = PurePursuit.lookahead_distance

        distance = self._calculate_distance(self.trajectory.x[index], state)

        while lookahead_distance > distance:
            if index + 1 >= len(self.trajectory.x):
                break

            index += 1

            distance = self._calculate_distance(
                self.trajectory.x[index], state)

        return index, lookahead_distance

    # ==================================================================================================
    # STATIC METHODS
    # ==================================================================================================
    @staticmethod
    def _calculate_distance(reference_x, current_x):
        distance = current_x - reference_x

        x = distance[:, 0] if distance.ndim == 2 else distance[0]

        y = distance[:, 1] if distance.ndim == 2 else distance[1]

        return np.hypot(x, y)
