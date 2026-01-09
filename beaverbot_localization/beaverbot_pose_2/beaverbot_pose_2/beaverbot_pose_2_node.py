#!usr/bin/env python3
##
# @file beaverbot_pose_2_node.py
#
# @brief Provide implementation of tractor-trailer system localization.
#
# @section author_doxygen_example Author(s)
# - Created by Loc Dang on 03/12/2025.
#
# Copyright (c) 2025 System Engineering Laboratory.  All rights reserved.

# Standard Libraries
import math
import time
import threading

# External Libraries
import tf
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

# Internal Libraries
import geonav_transform.geonav_conversions as gc


class BeaverbotPose2Node:
    """! BeaverbotPose2Node class
    The class provides implementation of Beaverbot localization by combining GPS and IMU data.
    """
    # ==========================================================================
    # PUBLIC METHODS
    # ==========================================================================

    def __init__(self):
        """! Constructor
        """
        super(BeaverbotPose2Node, self).__init__()

        rospy.init_node("beaverbot_pose_2", anonymous=True)

        self._yaw = None
        self._yaw_rate = None
        self._x_rear = None
        self._y_rear = None
        self._initial_yaw_enu = None
        self._initial_lat = None
        self._initial_lon = None
        self._trajectory_msg = Path()
        
        
        self._register_parameters()

        self._register_subscribers()
        
        self._register_publishers()
        
        self._imu_data_mutex = threading.Lock()
        self._gps_data_mutex = threading.Lock()

    def run(self):
        """! Start ros node
        """
        rospy.spin()

    # ==========================================================================
    # PRIVATE METHODS
    # ==========================================================================
    def _register_parameters(self):
        """! Register ROS parameters method
        """
        self._log = rospy.get_param(
            "~debug", True)
        
        # rotation angle of the robot frame with respect to ENU frame
        self._use_imu_rotation_angle = rospy.get_param("~use_imu_rotation_angle", False)
        
        self._rotation_angle = math.radians(rospy.get_param("~rotation_angle_deg", 0.0))
        
        self._publish_rate = rospy.get_param(
            "~publish_rate", 20.0)

        self._gps_to_rear_axis = rospy.get_param(
            "~gps_to_rear_axis", 0.0)

        self._imu_offset = math.radians(rospy.get_param(
            "~imu_offset", 0.0))

    def _register_subscribers(self):
        """! Register ROS subscribers
        """
        self._gps_sub = rospy.Subscriber(
            "/gps/fix", NavSatFix, self._gps_callback, queue_size=10)

        self._imu_sub = rospy.Subscriber(
            "/imu/data_raw", Imu, self._imu_callback, queue_size=10)

    def _register_publishers(self):
        """! Register publishers 
        """
        self._odom_pub = rospy.Publisher(
            "/odom", Odometry, queue_size=10)
        
        self._trajectory_pub = rospy.Publisher(
            "/trajectory", Path, queue_size=10)

        rospy.Timer(rospy.Duration(1.0 / self._publish_rate),
                    self._publish_odometry)
            
        self._tf_broadcaster = tf.TransformBroadcaster()

    def _gps_callback(self, data: NavSatFix):
        """! get x_rear, y_rear from gps data
        @param data: NavSatFix message
        """
        if self._yaw is None:
            rospy.loginfo("Waiting for IMU data to get yaw")
            return
        
        if self._initial_lat is None or self._initial_lon is None:
            rospy.loginfo("Initial GPS Data Received")
            self._initial_lat = data.latitude
            self._initial_lon = data.longitude
            return 
        
        with self._gps_data_mutex:
            self._x_rear, self._y_rear = self._get_xy_from_latlon(
                data.latitude, data.longitude, self._initial_lat, self._initial_lon)
        

    def _imu_callback(self, data: Imu):
        """! IMU callback method
        @param data: Imu message
        @return: yaw
        @ yaw: The yaw angle of the robot
        """
            
        euler = tf.transformations.euler_from_quaternion(
            [data.orientation.x,
             data.orientation.y,
             data.orientation.z,
             data.orientation.w])

        with self._imu_data_mutex:
            yaw_enu = euler[2] + self._imu_offset

            yaw_enu = math.atan2(math.sin(yaw_enu), math.cos(yaw_enu))
            
            if self._initial_yaw_enu is None:
                self._initial_yaw_enu = yaw_enu
                rospy.loginfo(f"Intial yaw in ENU: {math.degrees(yaw_enu)} degrees")
                return
            
            self._yaw = yaw_enu - self._initial_yaw_enu

            self._yaw_rate = data.angular_velocity.z    

    def _publish_odometry(self, timer):
        """! Publish rear wheel pose method
        @param timer: Timer (unused)
        """
        if self._x_rear is None or self._y_rear is None or self._yaw is None or self._yaw_rate is None:
            rospy.logwarn("Waiting for data to publish odometry")
            return
        
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self._yaw)
        
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.get_rostime()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self._x_rear
        odom_msg.pose.pose.position.y = self._y_rear
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        odom_msg.twist.twist.angular.z = self._yaw_rate

        self._odom_pub.publish(odom_msg)

        self._tf_broadcaster.sendTransform(
            (self._x_rear, self._y_rear, 0),
            (quaternion[0], quaternion[1],
             quaternion[2], quaternion[3]),
            rospy.Time.now(),
            "base_link",
            "odom"
        )
        
        # publish trajectory
        self._trajectory_msg.header.stamp = rospy.get_rostime()
        self._trajectory_msg.header.frame_id = "odom"
        self._trajectory_msg.poses.append(PoseStamped(
            header=Header(stamp=rospy.get_rostime(), frame_id="odom"),
            pose=Pose(position=Point(x=self._x_rear, y=self._y_rear, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1))
        ))
        self._trajectory_pub.publish(self._trajectory_msg)
        
        rospy.loginfo(f"x_rear, y_rear, yaw: {self._x_rear}, {self._y_rear}, {self._yaw}")

    def _get_xy_from_latlon(self, lat, lon, initial_lat, initial_lon):
        """! Get x, y from latitude and longitude method
        @param latitude: Current latitude of the robot
        @param longitude: Current longitude of the robot
        @param initial_lat: Initial latitude of the robot
        @param initial_lon: Initial longitude of the robot

        @return: x_rear_in_rear_frame: x position of the rear in the rear frame
        @return: y_rear_in_rear_frame: y position of the rear in the rear frame
        """
        with self._imu_data_mutex:
            yaw = self._yaw

        # get x, y in UTM (enu) frame from latitude and longitude
        x_gps_enu, y_gps_enu = gc.ll2xy(lat, lon, initial_lat, initial_lon)

        if self._use_imu_rotation_angle:
            rotation_angle = self._initial_yaw_enu
        else:
            rotation_angle = self._rotation_angle
        
        # convert to gps frame from ENU frame
        x_gps_in_gps_frame =   x_gps_enu * math.cos(rotation_angle) + y_gps_enu * math.sin(rotation_angle)
        y_gps_in_gps_frame = - x_gps_enu * math.sin(rotation_angle) + y_gps_enu * math.cos(rotation_angle)
        
        # convert to rear frame from gps frame (gps is along x axis of the gps frame)
        x_gps_in_rear_frame = x_gps_in_gps_frame - self._gps_to_rear_axis
        y_gps_in_rear_frame = y_gps_in_gps_frame
        
        # get rear position in rear frame
        x_rear_in_rear_frame = x_gps_in_rear_frame + self._gps_to_rear_axis * math.cos(yaw)
        y_rear_in_rear_frame = y_gps_in_rear_frame - self._gps_to_rear_axis * math.sin(yaw)

        return x_rear_in_rear_frame, y_rear_in_rear_frame
