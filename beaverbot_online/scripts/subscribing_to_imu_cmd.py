#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math

#initinalizing global variable to store most recent commanded angular velocity
cmd_angular_velocity_z = 0.0


def quat_to_yaw(q):
    # Converts quaternion to yaw (theta)
    w, x, y, z = q.w, q.x, q.y, q.z
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def imu_callback(msg):
    # Callback function for IMU data

    global cmd_angular_velocity_z

    orientation = msg.orientation
    yaw = quat_to_yaw(orientation)
    rospy.loginfo(f"Yaw (rad): {yaw:.4f}, Cmd ω (rad/s): {cmd_angular_velocity_z:.4f}")


def cmd_callback(msg):
    # Callback function for cmd_vel data

    global cmd_angular_velocity_z
    
    cmd_angular_velocity_z = msg.angular.z

def imu_and_cmd_listener():

    rospy.init_node('imu_and_cmd_listener', anonymous=True)
    rospy.Subscriber('/imu/data_raw', Imu, imu_callback)
    rospy.Subscriber('/cmd_vel', Twist, cmd_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        imu_and_cmd_listener()
    except rospy.ROSInterruptException:
        pass

