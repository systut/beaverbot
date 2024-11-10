#include "ros/ros.h"
#include "beaverbot_driver/robot_data.h"
#include <math.h>
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_broadcaster.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include "boost/assign.hpp"

using namespace industrial_robot;

class CommandOdom
{
private:
    double left_wheel_, right_wheel_;
	double * wheel_vel_;
	double * robot_vel_;

    ros::NodeHandle nh_;
    ros::Publisher odometry_;
    tf::TransformBroadcaster odom_broadcaster_;

    // variables for the odometry
    double x_, y_;
    double x_dot_, y_dot_;
    double phi_;
    double phi_dot_;

    // time which need to be stored
    ros::Time last_time_;
    double timestamp_;

    // function to check if velocity is reasonable or nonsense (if nonsense keep old one)
    bool feasableWheelAcc(double new_left_wheel, double new_right_wheel)
    {
	double left_acc = (new_left_wheel - left_wheel_) / timestamp_;
	double right_acc = (new_right_wheel - right_wheel_) / timestamp_;

	//ROS_INFO("%f %f", left_acc, right_acc);

	if (fabs(left_acc) < RobotParams::max_translational_acc && fabs(right_acc) < RobotParams::max_translational_acc)
	{
	    return true;
	}
	return false;
    }

    void update_pose()
    {
	double new_x, new_y, new_phi, new_x_dot, new_y_dot;

	new_x_dot = RobotModel::calc_x_dot(left_wheel_, right_wheel_, phi_);
	new_y_dot = RobotModel::calc_y_dot(left_wheel_, right_wheel_, phi_);
	std::cout << new_y_dot << std::endl;
	new_x = RobotModel::calc_x(x_, x_dot_, new_x_dot, timestamp_);
	new_y = RobotModel::calc_x(y_, y_dot_, new_y_dot, timestamp_);
	new_phi = RobotModel::calc_x(phi_, phi_dot_, robot_vel_[1], timestamp_);

	x_ = new_x;
	y_ = new_y;
	phi_ = new_phi;
	// twist in the robot frame
	x_dot_ = robot_vel_[0]; //linear velocity
	y_dot_ = 0.0; // because robot is nonholonomic
	phi_dot_ = robot_vel_[1]; // angular velocity
    }
public:
    CommandOdom(ros::NodeHandle nh)
    {
	nh_ = nh;
	odometry_ = nh_.advertise<nav_msgs::Odometry>("/command/odom", 50);

	// initialize pose and twist
	x_ = 0.0;
	y_ = 0.0;
	phi_ = 0.0;
	x_dot_ = 0.0;
	y_dot_ = 0.0;
	phi_dot_ = 0.0;

	left_wheel_ = 0;
	right_wheel_ = 0;

	static double robot_vel [2];
	robot_vel[0] = 0;

	robot_vel[1] = 0;

	robot_vel_ = robot_vel;

	last_time_ = ros::Time::now();
    }

    void publishOdom()
    {
	nav_msgs::Odometry odom;

	update_pose();

	// rpy to quaternion
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(phi_);

	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = last_time_;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x_;
	odom_trans.transform.translation.y = y_;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	// send the transform
	// odom_broadcaster_.sendTransform(odom_trans);

	// save data in messenger
	odom.header.stamp = last_time_;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";

	// pose
	odom.pose.pose.position.x = x_;
	odom.pose.pose.position.y = y_;

	odom.pose.pose.orientation = odom_quat;

	odom.pose.covariance =  boost::assign::list_of(1e-3) (0)   (0)  (0)  (0)  (0)
	                                                (0) (1e-3)  (0)  (0)  (0)  (0)
	                                                (0)   (0)  (1e6) (0)  (0)  (0)
	                                                (0)   (0)   (0) (1e6) (0)  (0)
	                                                (0)   (0)   (0)  (0) (1e6) (0)
	                                                (0)   (0)   (0)  (0)  (0)  (1e3) ;

	// twist
	odom.twist.twist.linear.x = x_dot_;
	odom.twist.twist.linear.y = y_dot_;

	odom.twist.twist.angular.z = phi_dot_;

	odom.twist.covariance =  boost::assign::list_of(1e-3) (0)   (0)  (0)  (0)  (0)
	                                                (0) (1e-3)  (0)  (0)  (0)  (0)
	                                                (0)   (0)  (1e6) (0)  (0)  (0)
	                                                (0)   (0)   (0) (1e6) (0)  (0)
	                                                (0)   (0)   (0)  (0) (1e6) (0)
	                                                (0)   (0)   (0)  (0)  (0)  (1e3) ;

	odometry_.publish(odom);
    }

    void getCommandVelocity(const geometry_msgs::Twist::ConstPtr &robot_vel_msg)
    {
	// get timestamp and update last time
	ros::Time current_time = ros::Time::now();
	timestamp_ = (current_time - last_time_).toSec();
	last_time_ = current_time;

	static double robot_vel [2];
	robot_vel[0] = robot_vel_msg->linear.x;

	robot_vel[1] = robot_vel_msg->angular.z;

	robot_vel_ = robot_vel;

	wheel_vel_ = RobotModel::robot_velocites_to_wheel_velocities(robot_vel_msg->linear.x, robot_vel_msg->angular.z);

	double new_left_wheel, new_right_wheel;

	new_left_wheel = wheel_vel_[0];
	new_right_wheel = wheel_vel_[1];

	// update values if feasible
	if (feasableWheelAcc(new_left_wheel, new_right_wheel))
	{
	    left_wheel_ = new_left_wheel;
	    right_wheel_ = new_right_wheel;
	}
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "command_to_odom");
  ros::NodeHandle nh;
  CommandOdom commandOdom(nh);
  ros::Subscriber subCommand = nh.subscribe("/cmd_vel", 1000, &CommandOdom::getCommandVelocity, &commandOdom);
  ros::Rate loop_rate(20);

  while(ros::ok())
  {
      ros::spinOnce();

      commandOdom.publishOdom();

      loop_rate.sleep();
  }
  return 0;
}

