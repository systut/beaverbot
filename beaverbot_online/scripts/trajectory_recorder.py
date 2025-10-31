import rospy
from nav_msgs.msg import Odometry
import csv
import os

class TrajectoryRecorder:
    def __init__(self, filename = "trajectory_no_slip.csv"):
        
        #rospy.init_node('trajectory_recorder', anonymous=True)
        self.filename = filename


        #Preparing the csv file 
        with open(self.filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time', 'x', 'y'])

        rospy.Subscriber('/ground_truth/odom', Odometry, self.odom_callback)
        rospy.loginfo(f"Trajectory recorder started — writing to {self.filename}")

    def odom_callback(self, msg):
        #extracting time,x,y from IMU message
        time = msg.header.stamp.to_sec()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        #theta = self.quat_to_yaw(msg.pose.pose.orientation)

        # Writing to CSV
        with open(self.filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([time, x, y])
    
    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        recorder = TrajectoryRecorder(filename = "trajectory_no_slip.csv")
        recorder.spin()
    except rospy.ROSInterruptException:
        pass
