#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time
import numpy as np
from rls_online import RLSOnline  
from trajectory_recorder import TrajectoryRecorder

class RLSNode:
    def __init__(self):
        rospy.init_node('online_rls_node', anonymous=True)

        # Initialize RLS
        s0 = np.array([[1.1]])       # initial slip estimate
        P0 = np.eye(1) * 100.0      # large initial covariance
        R = np.eye(1) * 0.01         # measurement noise
        self.rls = RLSOnline(s0, P0, R)

        # Variables to track state
        self.angular_z = 0.0
        self.linear_x = 0.0
        self.v_right_ground = 0.0
        self.v_left_ground= 0.0
        self.prev_yaw = None
        self.prev_time = None
        self.v_left_compensated = 0.0
        self.v_right_compensated = 0.0
        self.L = 0.5  # distance between wheels
        self.compensated_linear_x = 0.0
        self.compensated_angular_z = 0.0



        # Subscribers
        rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback)
        rospy.Subscriber('/ground_truth/odom', Odometry, self.ground_odom_callback)

        rospy.loginfo("Online RLS node started — listening to /imu/data and /ground_truth/odom.")

    def quat_to_yaw(self, q):
        """Convert quaternion to yaw (in radians)."""
        w, x, y, z = q.w, q.x, q.y, q.z
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def ground_odom_callback(self, msg):
        """Store latest commanded angular and linear velocity."""
        self.linear_x = msg.twist.twist.linear.x
        self.angular_z = msg.twist.twist.angular.z

        #computing ground velocity for each wheel
        
        self.v_right_ground = self.linear_x + (self.L / 2.0) * self.angular_z
        self.v_left_ground = self.linear_x - (self.L / 2.0) * self.angular_z


    def imu_callback(self, msg):
        """Process IMU message and run one RLS update step."""
        yaw = self.quat_to_yaw(msg.orientation)
        current_time = time.time()

        if self.prev_yaw is not None and self.prev_time is not None:
            delta_t = current_time - self.prev_time
            if delta_t <= 0:
                return

            # Call RLS update
            self.rls.predict_sim(
                yaw=yaw,
                yaw_previous=self.prev_yaw,
                ground_angular_velocity_z=self.angular_z,
                delta_t=delta_t
            )

            # Log the latest slip estimate
            latest_est = self.rls.estimates[-1][0,0]
            rospy.loginfo(f"Slip estimate: {latest_est:.6f}")

        
            #rospy.loginfo(f"v_right_ground: {self.v_right_ground:.6f}, v_left_ground: {self.v_left_ground:.6f}")
            # Compensate and publish cmd_vel
            self.compensate_and_publish(latest_est)
            self.get_trajectory = TrajectoryRecorder(filename = "trajectory_with_slip.csv")

        # Update stored values for next step
        self.prev_yaw = yaw
        self.prev_time = current_time

    #method for compensating and publishing cmd_vel
    def compensate_and_publish(self, slip_est):
        #calculating compensated wheel velocities
            slip = np.clip(slip_est, -0.99, 0.99)  # avoid division by zero or negative slip
            self.v_right_compensated = self.v_right_ground / (1 - slip)
            self.v_left_compensated = self.v_left_ground / (1 - slip)
            #calculating compensated linear and angular velocities
            self.compensated_linear_x = (self.v_right_compensated + self.v_left_compensated) / 2.0
            self.compensated_angular_z = (self.v_right_compensated - self.v_left_compensated) / self.L
            #publishing compensated velocities
            cmd_msg = Twist()
            cmd_msg.linear.x = self.compensated_linear_x
            cmd_msg.angular.z = self.compensated_angular_z
            # Use a persistent publisher instead of creating it every callback
            if not hasattr(self, 'cmd_pub'):
               self.cmd_pub = rospy.Publisher('beaverbot/cmd_vel', Twist, queue_size=10)
            self.cmd_pub.publish(cmd_msg)
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    node = RLSNode()
    node.spin()
