#ifndef ROBOTDATA_H
#define ROBOTDATA_H

#include <boost/math/constants/constants.hpp>

namespace industrial_robot
{
  class RobotParams
  {
  public:
    RobotParams();
    // robot constants
    static const double wheel_radius;
    static const double wheel_to_center;

    // converter for the robot input to rad of the wheels
    static const double input_value_to_rad;
    // converts the robot input to translational velocity
    static const double input_value_to_vel;

    // robot constraints
    static const int max_wheel_vel_robot = 600; // in robot values
    static const double max_wheel_vel_unit; // in m/s
    // empirically measured
    static const double max_translational_acc; // value without mass (lift robot up) is ~10.5 m/s^2
    static const double max_defined_lin_acc;    // max acceleration which is taken for the robot
    static const double max_defined_ang_acc;

    // port names
    // not yet done

    // imu scaling factors -> maybe not needed in ROS

    // converter constants
    static const double g_to_acc;

    static double rad_to_deg;
    static double deg_to_rad;

    // convert to robot input values 1 to 600 or in km/h 0.01 km/h to 6 km/h
    static const double quantizer_rad;

    // calibration counter -> maybe not needed with ROS
    // static const int calibration_count_imu = 500;

    static double TestInput();
  };

  class RobotModel
  {
  public:
    RobotModel();
    // convert the wheel velocities to the robot velocities
    // wheel velocites must be in rad/s!!!
    static double * wheel_velocities_to_robot_velocities(double left_wheel_vel, double right_wheel_vel);

    // convert the robot velocites to wheel velocites
    // linear velocity must be in m/s and angular velocity must be in rad/s!!!
    static double * robot_velocites_to_wheel_velocities(double linear_robot_vel, double angular_robot_vel);

    // the following are functions to compute the twist in the world frame (wheel vel must be in rad/s)
    static double calc_x_dot(double left_wheel_vel, double right_wheel_vel, double prev_phi);
    static double calc_y_dot(double left_wheel_vel, double right_wheel_vel, double prev_phi);
    static double calc_phi_dot(double left_wheel_vel, double right_wheel_vel);

    static double calc_x(double prev_x, double prev_x_dot, double x_dot, double sampling_time);
    static double calc_y(double prev_y, double prev_y_dot, double y_dot, double sampling_time);
    static double calc_phi(double prev_phi, double prev_phi_dot, double phi_dot, double sampling_time);
  };

  class RobotSupportFunctions
  {
  public:
    RobotSupportFunctions();
    // safety constraint of the robot
    static int max_robot_velocity(int desired_robot_vel);

    // returns the velocity in units which are accpeted by the robot
    static int rad_to_robot_value(double wheel_vel_in_rad);

    // trapezoidal rule for numerical integration
    static double trapzoidal_rule(double prev_value, double current_value, double sampling_time);

    static double integrate(double prev_value, double prev_value_dot, double current_value_dot, double sampling_time);

    static double twoPiMod(double angle);
  };

}
#endif // ROBOTDATA_H
