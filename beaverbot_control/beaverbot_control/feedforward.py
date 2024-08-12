#!/usr/bin/env python3
##
# @file feedforward.py
#
# @brief Provide implementation of pure pursuit controller for
# autonomous driving.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 08/12/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.


class FeedForward:
    """! FeedForward controller

    The class provides implementation of feedforward controller for
    autonomous driving.
    """
    # ==================================================================================================
    # PUBLIC METHODS
    # ==================================================================================================
    def __init__(self, trajectory):
        """! Constructor
        @param trajectory<instance>: The trajectory
        """
        self.trajectory = trajectory

    def execute(self, state, input, index):
        """! Execute the controller
        @param state<list>: The state of the vehicle
        @param input<list>: The input of the vehicle
        @param index<int>: The index
        @return<tuple>: The status and control
        """
        status = True

        if index >= len(self.trajectory.u[0, :]) - 1:
            return False, [0, 0]

        u = self.trajectory.u[:, index]

        return status, u
