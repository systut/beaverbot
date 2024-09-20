#!/usr/bin/env python3
##
# @file tsnd5_driver_node.py
#
# @brief Provide implementation of tsnd5 driver node for
# autonomous driving.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 2024/09/19
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library
import numpy as np
from queue import Queue

# External library
import rospy
from sensor_msgs.msg import Imu

# Internal library
from tsnd5_driver.tsnd5_driver import TSND151


class Tsnd5DriverNode:
    """! Tsnd5DriverNode class.

    This class provides implementation of tsnd5 driver node.
    """
    # ========================================================================
    # PUBLIC METHODS
    # ========================================================================

    def __init__(self):
        """! Constructor.
        """
        super(Tsnd5DriverNode, self).__init__()

        rospy.init_node("tsnd5_driver_node")

        self._read_parameters()

        self._open_sensor()

        self._register_publishers()

        self._register_timers()

    def run(self):
        """! Execute the node.
        """
        rospy.spin()

    # ========================================================================
    # PRIVATE METHODS
    # ========================================================================
    def _setup_sensor(self):
        """! Setup sensor.
        @note The unit of interval is millisecond.
        acceleration range: +-2, +-4, +-8, +-16 g
        gyroscopic range: +-250, +-500, +-1000, +-2000 deg/s
        """
        hz = 100

        self._driver.set_time()

        self._driver.set_acc_range(16)

        self._driver.set_gyro_range(2000)

        self._driver.set_acc_and_gyro_interval(
            interval_in_ms=2,
            avg_num_for_send=int(1000 / hz / 2),
            avg_num_for_save=0)

        self._driver.set_quaternion_interval(
            interval_in_5ms_unit=1,
            avg_num_for_send=int(1000 / hz / 5),
            avg_num_for_save=0)

        self._driver.set_magnetism_interval(0, 0, 0)

        self._driver.set_atmosphere_interval(0, 0, 0)

        self._driver.set_battery_voltage_measurement(False, False)

        self._driver.set_overwrite_protection(False)

        self._driver.set_auto_power_off(0)

    def _open_sensor(self):
        """! Open sensor.
        """
        self._driver = TSND151.open(
            self._serial_port,
            wait_sec_on_open_for_stability=0.2,
            wait_sec_on_auto_close_for_stability=0.2)

        self._driver.clear_all_queue()

        self._setup_sensor()

        self._q = Queue()

        self._driver.set_response_queue(
            'quaternion_acc_gyro_data', self.q)

    def _close_sensor(self):
        """! Close sensor.
        """
        self._driver.close()

    def _read_parameters(self):
        """! Read parameters.
        """
        self._serial_port = rospy.get_param("~serial_port", "/dev/ttyACM0")

        self._sampling_time = rospy.get_param("~sampling_time", 0.1)

    def _register_publishers(self):
        """! Register publishers.
        """
        self._imu_publisher = rospy.Publisher(
            "imu/data_raw", Imu, queue_size=10)

    def _register_timers(self):
        """! Register timers.
        """
        self._timer = rospy.Timer(
            rospy.Duration(self._sampling_time),
            self._timer_callback)

    def _timer_callback(self, event):
        """! Timer callback.
        @param event Timer event.
        """
        if self._q.empty():
            return

        data = np.array([self._parse_queue_data(self._q.get()) for i in range(
            self._q.qsize())])

        first_in_queue = data[0]

        imu_msg = self._convert_data_to_imu_msg(first_in_queue)

        self._imu_publisher.publish(imu_msg)

    def _convert_data_to_imu_msg(self, data):
        """! Convert data to imu message.
        @param data: Data.
        @return Imu message.
        @note (ms, quaternion, acc, gyro):
        acc=(ax, ay, az) in 0.1mg,
        gyro=(gx, gy, gz) in 0.01dps
        """
        imu_msg = Imu()

        imu_msg.header.stamp = rospy.Time.now()

        imu_msg.header.frame_id = "imu_link"

        imu_msg.orientation.w = data["quaternion"][0] / 10000

        imu_msg.orientation.x = data["quaternion"][1] / 10000

        imu_msg.orientation.y = data["quaternion"][2] / 10000

        imu_msg.orientation.z = data["quaternion"][3] / 10000

        imu_msg.angular_velocity.x = float(data["gyro"][0]) * 0.01

        imu_msg.angular_velocity.y = float(data["gyro"][1]) * 0.01

        imu_msg.angular_velocity.z = float(data["gyro"][2]) * 0.01

        imu_msg.linear_acceleration.x = float(
            data["acceleration"][0]) / (10**3)

        imu_msg.linear_acceleration.y = float(
            data["acceleration"][1]) / (10**3)

        imu_msg.linear_acceleration.z = float(
            data["acceleration"][2]) / (10**3)

        return imu_msg

    def _parse_queue_data(self, queue):
        """! Parse queue data.
        @param queue Queue data.
        @note (ms, quaternion, acc, gyro):
        acc=(ax, ay, az) in 0.1mg,
        gyro=(gx, gy, gz) in 0.01dps
        """
        raw_data = self._driver.parse_quaternion_acc_gyro(queue)

        _, quaternion, acceleration, gyro = raw_data

        data = {
            'quaternion': quaternion,
            'acceleration': acceleration,
            'gyro': gyro
        }

        return data
