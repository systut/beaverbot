#ifndef GLOBAL_TRAJECTORY_H
#define GLOBAL_TRAJECTORY_H

#include <cmath>
#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <fstream>

#include "sdv_msgs/Trajectory.h"
#include "sdv_msgs/TrajectoryPoint.h"
#include "trajectory_generation/utilities.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

class Global_traj_class
{
public:
    Global_traj_class(ros::NodeHandle nh, ros::NodeHandle private_nh, Eigen::MatrixXd coverage_path);
    ~Global_traj_class();
    void euler_spiral_coefficient_generator();
    Eigen::RowVector2d unit_euler_spiral_coordinate_calculator(double &angle, Eigen::MatrixXd &table);
    double calculate_angle_from_vector(Eigen::RowVector2d &vector);
    void get_robot_current_state(Eigen::RowVector2d& robot_current_position, double& robot_current_path_velocity);
    void way_point_generator();
    void way_point_generator_01();
    void euler_spiral_corner_smoothing_solver();
    Eigen::RowVector4d jlap_acceleration_phase_displacement_calculator(double& path_vel_start, double &path_acc_start, double &path_vel_end, double &path_acc_end);
    void point_to_point_motion_jlap_solver();

    Eigen::VectorXd point_to_point_motion_single_line_jlap_solver(Eigen::RowVector2d& pos_start, double& path_vel_start, double& path_acc_start, Eigen::RowVector2d& pos_end, double& path_vel_end, double& path_acc_end);
    void point_to_point_single_line_trajectory_interpolator();
    void convertToFrontWheel(double x_dot, double y_dot, double phi_dot, double &v_front, double &delta);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    void trajectory_interpolator();
    void export_global_trajectory();
    void publishPathAndTrajectory();
    void generateTrajectory();

    void rpm_convert();

private:

    bool isSame(double lhs, double rhs);

    const double EPS = 1e-9;

    // messages
    sdv_msgs::Trajectory trajectory_msg_;
    nav_msgs::Path adapted_path_msg_;

    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
    ros::Publisher traj_pub_;
    ros::Publisher path_pub_;
    ros::Publisher emergency_stop_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber map_sub_;

    nav_msgs::OccupancyGrid map_;
    nav_msgs::Odometry odom_;

    std::vector<Eigen::Vector3d> astar_path_;

    // Mobile robot parameters/////////////////////////////////////AWARE : MOST OF THESE ARE VALUES OF BIG MUSASHI ROBOT/////////////
    const double PI                 = 3.14159265359;
    const double SAMPLING_TIME      = 1;              // [s]
    const double ROBOT_MASS         = 120.4;             // [kg]
    const double ROBOT_LENGTH       = RobotConstants::ROBOT_LENGTH;               // [m]
    const double ROBOT_WIDTH        = RobotConstants::ROBOT_WIDTH;              // [m]
    const double WHEEL_RADIUS       = RobotConstants::WHEEL_RADIUS;              // [m]
    const double WHEEL_AXIS_DIST    = 0.5*ROBOT_WIDTH; // [m] Distance between a driving wheeland center of the wheel - to - wheel axis
    const double GEAR_RATIO         = 100.0;                // [ ] Driving motor to wheel gear ratio
    const double RATED_MOTOR_TORQUE = 1.3;               // [Nm]
    const double RATED_MOTOR_SPEED  = 3500;              // [RPM]
    const double MOTOR_INERTIA      = 0.66e-4;           // [kgm^2]

    const double RATED_TORQUE = GEAR_RATIO * RATED_MOTOR_TORQUE;         // [Nm]    Torque reflected to the robot's wheels
    const double INERTIA      = GEAR_RATIO * GEAR_RATIO * MOTOR_INERTIA; // [kgm^2] INERTIA reflected to the robot's wheels
                     // [m/s]   Path velocity limit (< 0.943)
    const double PATH_ACC_LIM  = RATED_TORQUE * WHEEL_RADIUS/ (0.25 * ROBOT_MASS * WHEEL_RADIUS * WHEEL_RADIUS + INERTIA);  // [m/s^2] 50% Path acceleration limit
    const double PATH_JERK_LIM = PATH_ACC_LIM / (40 * SAMPLING_TIME); // [m/s^3] Path jerk limit

    const double ANG_ACC_LIM  = PATH_ACC_LIM / WHEEL_RADIUS;                   // [rad/s^2] Wheel angular acceleration limit
    const double ANG_VEL_LIM  = (2 * PI / 60) * RATED_MOTOR_SPEED / GEAR_RATIO; // [rad/s]  Wheel angular velocity limit
    const double ANG_JERK_LIM = PATH_JERK_LIM / WHEEL_RADIUS;              // [rad/s^3]  Wheel angular jerk limit

    double PATH_VEL_LIM;                         // [m/s]   Path velocity limit

    // Approximate Integration of Euler spiral
    const unsigned short int N_EULER = 50;                   // Number of Euler coefficients
    Eigen::MatrixXd euler_table;              // Table consisting of Euler coefficients
    Eigen::RowVector2d unit_euler_coordinate; // Coordinate of a point along the unit Euler spiral
    double phi_abs = 0;                        // Orientation angle at a point on the unit Euler spiral, where phi_abs > 0

    // Robot's current state
    Eigen::RowVector2d pos_current; // [m]   Robot's current [X, Y] coordinates in the global map
    double path_vel_current;        // [m/s] Robot's current path velocity.
    // NB: It is assumed that the robot's heading is toward the next way point

    // Way point generation
    Eigen::MatrixXd way_points_original; // [m] Original Way points through which the mobile robot should traverse
    Eigen::MatrixXd way_points; // [m] Way_points based on the robots current position
    Eigen::MatrixXd map_bounds; // [m] Coordinates of the edges of the map boundaries
    unsigned int n_point, n_line, n_corner; // Number of corners

    // Euler spiral corner smoothing parameters
    double epsilon_max = 0.5;         // [m]   Maximum allowable cornering tolerance
    Eigen::VectorXd epsilon;          // [m]   Cornering tolerance at each corner
    Eigen::MatrixXd pos_start_corner; // [m]   Start point of each corner
    Eigen::MatrixXd pos_mid_corner;   // [m]   Mid point of each corner
    Eigen::MatrixXd pos_end_corner;   // [m]   End point of each corner
    Eigen::VectorXd path_vel_corner;  // [m/s] Path velocity of each corner
    Eigen::VectorXd	T_corner;         // [s]   Time required to traverse the each corner
    Eigen::VectorXd	a_euler;          // [m]   Euler scaling factor
    Eigen::VectorXd Lc;               // [m]   Eucledian length
    Eigen::VectorXd	turn_direction;   // [ ]   1: Counter clockwise and -1 : clockwise

    // Point to point motion JLAP solver data
    Eigen::MatrixXd T_jlap;         // JLAP time intervals for all linear segments
    Eigen::MatrixXd pos_start;      // [m]   Initial position of each linear segment
    Eigen::MatrixXd pos_end;        // [m]   Ending position of each linear segment
    Eigen::VectorXd path_vel_start; // [m/s] Initial path velocities for each linear segment
    Eigen::VectorXd path_vel_end;   // [m/s] Ending path velocities for each linear segment

    double PATH_VEL_STEP = 0.01;     // [m/s] Step intervals in recursively determining the path velocity

    // Trajectory interpolation
    unsigned int Ns;             // [] Number of samples
    Eigen::MatrixXd unit_vector; // []
    Eigen::VectorXd t;           // [s] time instant
    Eigen::MatrixXd ang_acc;     // [rad/s^2] Angular accelerations
    Eigen::MatrixXd ang_vel;     // [rad/s] Angular velocities
    Eigen::MatrixXd acc;         // [m/s^2] Axial accelerations
    Eigen::MatrixXd vel;         // [m/s] Axial velocities
    Eigen::MatrixXd pos;         // [m] robot position

    Eigen::VectorXd path_acc;    // [m/s^2] path acceleration
    Eigen::VectorXd path_vel;    // [m/s] path velocity
    Eigen::VectorXd path_length; // [m]
    Eigen::VectorXd phi;         // [rad] Robot orientation
    Eigen::VectorXd dphi;        // [rad/s] First derivative of robot orientation
    Eigen::VectorXd ddphi;       // [rad/s^2] Second derivative of robot orientation

    Eigen::RowVectorXd max_ang_vel; // [rad/s] maximum angular velocity while turning
    Eigen::RowVectorXd max_ang_acc; // [rad/s^2] maximum angular acceleration while turning

    // Getting the Trajectory
    Eigen::MatrixXd ref_traj; // Overall trajectory data set.

    Eigen::MatrixXd v_rpm;
};


#endif // GLOBAL_TRAJECTORY_H
