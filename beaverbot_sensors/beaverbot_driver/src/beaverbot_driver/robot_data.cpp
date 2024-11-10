#include "beaverbot_driver/robot_data.h"
#include <iostream>
#include "ros/ros.h"
#include <boost/math/constants/constants.hpp>
#include <boost/math/special_functions/round.hpp>

using namespace industrial_robot;

RobotParams::RobotParams(){}

const double RobotParams::wheel_radius = 0.328 / 2;
const double RobotParams::wheel_to_center = 0.53 / 2;

// m/s
const double RobotParams::input_value_to_vel = 0.01 / 3.6;
// rad/s
const double RobotParams::input_value_to_rad = RobotParams::input_value_to_vel / RobotParams::wheel_radius;

// in rad/s
const double RobotParams::max_wheel_vel_unit = RobotParams::max_wheel_vel_robot * RobotParams::input_value_to_rad;
// in m/s^2 for the wheels not the robot
const double RobotParams::max_translational_acc = 10.5;
// for the wheels
const double RobotParams::max_defined_lin_acc = 4.0;	// m/s^2
const double RobotParams::max_defined_ang_acc = RobotParams::max_defined_lin_acc / RobotParams::wheel_radius;	//rad/s^2

const double RobotParams::g_to_acc = 9.81;

double RobotParams::rad_to_deg = 180 / boost::math::constants::pi<double>();
double RobotParams::deg_to_rad = boost::math::constants::pi<double>() / 180;

// step between two discrete inputs (here command 2 and command 1 in robot values)
const double RobotParams::quantizer_rad = (2  * RobotParams::input_value_to_rad) - (1  * RobotParams::input_value_to_rad);

double RobotParams::TestInput()
{
  return 5.0;
}

RobotModel::RobotModel(){}

// wheel velocities are in rad/s
double * RobotModel::wheel_velocities_to_robot_velocities(double left_wheel_vel, double right_wheel_vel)
{
  static double robot_velocities [2];

  double linear_robot_vel = (RobotParams::wheel_radius / 2) * (right_wheel_vel + left_wheel_vel);
  robot_velocities[0] = linear_robot_vel;

  double angular_robot_vel = (RobotParams::wheel_radius / (2 * RobotParams::wheel_to_center)) * (right_wheel_vel - left_wheel_vel);
  robot_velocities[1] = angular_robot_vel;

  return robot_velocities;
}

// wheel velocities are in rad/s
double * RobotModel::robot_velocites_to_wheel_velocities(double linear_robot_vel, double angular_robot_vel)
{
  static double wheel_velocities [2];

  double left_wheel_vel = (linear_robot_vel / RobotParams::wheel_radius) - (RobotParams::wheel_to_center * angular_robot_vel / RobotParams::wheel_radius);
  wheel_velocities[0] = left_wheel_vel;

  double right_wheel_vel = (linear_robot_vel / RobotParams::wheel_radius) + (RobotParams::wheel_to_center * angular_robot_vel / RobotParams::wheel_radius);
  wheel_velocities[1] = right_wheel_vel;

  return wheel_velocities;
}

double RobotModel::calc_x_dot(double left_wheel_vel, double right_wheel_vel, double prev_phi)
{
    return 0.5 * cos(prev_phi) * RobotParams::wheel_radius * (left_wheel_vel + right_wheel_vel);
}

double RobotModel::calc_y_dot(double left_wheel_vel, double right_wheel_vel, double prev_phi)
{
    return 0.5 * sin(prev_phi) * RobotParams::wheel_radius * (left_wheel_vel + right_wheel_vel);
}

double RobotModel::calc_phi_dot(double left_wheel_vel, double right_wheel_vel)
{
    return (RobotParams::wheel_radius / (2 * RobotParams::wheel_to_center)) * (right_wheel_vel - left_wheel_vel);
}

double RobotModel::calc_x(double prev_x, double prev_x_dot, double x_dot, double sampling_time)
{
    return RobotSupportFunctions::integrate(prev_x, prev_x_dot, x_dot, sampling_time);
}

double RobotModel::calc_y(double prev_y, double prev_y_dot, double y_dot, double sampling_time)
{
    return RobotSupportFunctions::integrate(prev_y, prev_y_dot, y_dot, sampling_time);
}

double RobotModel::calc_phi(double prev_phi, double prev_phi_dot, double phi_dot, double sampling_time)
{
    double phi = RobotSupportFunctions::integrate(prev_phi, prev_phi_dot, phi_dot, sampling_time);
    return RobotSupportFunctions::twoPiMod(phi);
}

RobotSupportFunctions::RobotSupportFunctions(){}

int RobotSupportFunctions::max_robot_velocity(int desired_robot_vel)
{
  // return the robot value which satisfies the robot constraints
  if (desired_robot_vel > RobotParams::max_wheel_vel_robot)
  {
    return RobotParams::max_wheel_vel_robot;
  }
  else if (desired_robot_vel < -RobotParams::max_wheel_vel_robot)
  {
    return -RobotParams::max_wheel_vel_robot;
  }
  else
  {
    return desired_robot_vel;
  }
}

int RobotSupportFunctions::rad_to_robot_value(double wheel_vel_in_rad)
{
  double quantized_value;
  quantized_value = RobotParams::quantizer_rad * boost::math::iround(wheel_vel_in_rad / RobotParams::quantizer_rad);
  quantized_value = quantized_value * RobotParams::wheel_radius;
  quantized_value = quantized_value * 3.6;
  // round the value to the nearest integer
  int ret = boost::math::iround(quantized_value * 100);
  // return the value which satisfies the robot constraints
  return RobotSupportFunctions::max_robot_velocity(ret);
}

double RobotSupportFunctions::trapzoidal_rule(double prev_value, double current_value, double sampling_time)
{
    return sampling_time * ((prev_value + current_value) / 2);
}

double RobotSupportFunctions::integrate(double prev_value, double prev_value_dot, double current_value_dot, double sampling_time)
{
    return (prev_value + RobotSupportFunctions::trapzoidal_rule(prev_value_dot, current_value_dot, sampling_time));
}

double RobotSupportFunctions::twoPiMod(double angle)
{
    angle = fmod(angle, 2.0 * boost::math::constants::pi<double>());
    if (angle < 0.0)
    {
	angle += 2.0 * boost::math::constants::pi<double>();
    }
    return angle;
}

