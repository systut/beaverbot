#include "ros/ros.h"
#include <beaverbot_driver/EncoderWheelVel.h>
#include "beaverbot_driver/robot_data.h"
#include <math.h>
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_broadcaster.h"
#include <nav_msgs/Odometry.h>
#include "boost/assign.hpp"

using namespace industrial_robot;

class EncoderOdom
{
private:
    int enc_left_wheel_, enc_right_wheel_;

    ros::NodeHandle nh_;
    ros::Publisher odometry_;
    tf::TransformBroadcaster odom_broadcaster_;

    // variables for the odometry
    double x_, y_; //position in world frame
    double x_dot_, y_dot_; //linear velocity in world frame
	double x_dot_b_, y_dot_b_; //linear velocity in body frame
    double phi_;
    double phi_dot_;

    // time which need to be stored
    ros::Time last_time_;
    double timestamp_;

	// publish_tf flag
	bool publish_tf_;

    // function to check if velocity is reasonable or nonsense (if nonsense keep old one)
    bool feasableWheelAcc(int new_enc_left, int new_enc_right)
    {
	double left_acc = ((new_enc_left * RobotParams::input_value_to_vel) - (enc_left_wheel_ * RobotParams::input_value_to_vel)) / timestamp_;
	double right_acc = ((new_enc_right * RobotParams::input_value_to_vel) - (enc_right_wheel_ * RobotParams::input_value_to_vel)) / timestamp_;

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
	double * robot_vel;

	new_x_dot = RobotModel::calc_x_dot(enc_left_wheel_ * RobotParams::input_value_to_rad, enc_right_wheel_ * RobotParams::input_value_to_rad, phi_);
	new_y_dot = RobotModel::calc_y_dot(enc_left_wheel_ * RobotParams::input_value_to_rad, enc_right_wheel_ * RobotParams::input_value_to_rad, phi_);

	robot_vel = RobotModel::wheel_velocities_to_robot_velocities(enc_left_wheel_ * RobotParams::input_value_to_rad, enc_right_wheel_ * RobotParams::input_value_to_rad);

	new_x = RobotModel::calc_x(x_, x_dot_, new_x_dot, timestamp_);
	new_y = RobotModel::calc_x(y_, y_dot_, new_y_dot, timestamp_);
	new_phi = RobotModel::calc_x(phi_, phi_dot_, robot_vel[1], timestamp_);

	x_ = new_x;
	y_ = new_y;
	phi_ = new_phi;
	x_dot_ = new_x_dot;
	y_dot_ = new_y_dot;

	// twist in the robot frame
	x_dot_b_ = robot_vel[0]; //linear velocity
	y_dot_b_ = 0.0; // because robot is nonholonomic
	phi_dot_ = robot_vel[1]; // angular velocity
    }
public:
    EncoderOdom(ros::NodeHandle nh)
    {
	nh_ = nh;
	odometry_ = nh_.advertise<nav_msgs::Odometry>("/beaverbot/odom", 50);

	ros::param::param<bool>("~publish_tf", publish_tf_, true);
	// initialize pose and twist
	x_ = 0.0;
	y_ = 0.0;
	phi_ = 0.0;
	x_dot_ = 0.0;
	y_dot_ = 0.0;
	phi_dot_ = 0.0;

	x_dot_b_ = 0.0;
	y_dot_b_ = 0.0;

	enc_left_wheel_ = 0;
	enc_right_wheel_ = 0;

	last_time_ = ros::Time::now();
    }

    void publishOdom()
    {
	nav_msgs::Odometry odom;

	update_pose();

	// rpy to quaternion
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(phi_);

	if (publish_tf_==true)
	{
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = last_time_;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x_;
	odom_trans.transform.translation.y = y_;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	// send the transform
	odom_broadcaster_.sendTransform(odom_trans);
	}

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
	odom.twist.twist.linear.x = x_dot_b_;
	odom.twist.twist.linear.y = y_dot_b_;

	odom.twist.twist.angular.z = phi_dot_;

	odom.twist.covariance =  boost::assign::list_of(1e-3) (0)   (0)  (0)  (0)  (0)
	                                                (0) (1e-3)  (0)  (0)  (0)  (0)
	                                                (0)   (0)  (1e6) (0)  (0)  (0)
	                                                (0)   (0)   (0) (1e6) (0)  (0)
	                                                (0)   (0)   (0)  (0) (1e6) (0)
	                                                (0)   (0)   (0)  (0)  (0)  (1e3) ;

	odometry_.publish(odom);
    }

    void getEncoderData(const beaverbot_driver::EncoderWheelVel::ConstPtr &enc)
    {
	// get timestamp and update last time
	ros::Time current_time = ros::Time::now();
	timestamp_ = (current_time - last_time_).toSec();
	last_time_ = current_time;

	int new_enc_left, new_enc_right;

	new_enc_left = enc->enc_left_wheel;
	new_enc_right = enc->enc_right_wheel;

	// update values if feasible
	if (feasableWheelAcc(new_enc_left, new_enc_right))
	{
	    enc_left_wheel_ = new_enc_left;
	    enc_right_wheel_ = new_enc_right;
	}
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "encoder_to_odom");
  ros::NodeHandle nh;
  EncoderOdom encOdom(nh);
  ros::Subscriber subEncoder = nh.subscribe("/encoder", 1000, &EncoderOdom::getEncoderData, &encOdom);

  ros::Rate loop_rate(20);

  ROS_INFO_STREAM("Transform encoder data.");

  while(ros::ok())
  {
      ros::spinOnce();

      encOdom.publishOdom();

      loop_rate.sleep();
  }
  return 0;
}

