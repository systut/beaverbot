// Musashi_project_cpp.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "trajectory_generation/global_trajectory.h"
#include "trajectory_generation/coverage_planner.h"
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <cmath>
#include <math.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <chrono>

#include <ros/ros.h>

#include "sdv_msgs/Trajectory.h"
#include "sdv_msgs/TrajectoryPoint.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offline_trajectory_node");

    auto start_time = std::chrono::system_clock::now();

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    CoveragePlanner coverage_planner;
    
    int iteration;
    ros::param::param<int>("~iteration", iteration, 2);

    double length;
    ros::param::param<double>("~length", length, 13.0);

    double width;
    ros::param::param<double>("~width", width, 13.0);

    Eigen::MatrixXd coverage_path;

    coverage_planner.generate(length, width, iteration, coverage_path);

    std::cout << "Coverage path:\n" << coverage_path << std::endl;

    Global_traj_class GlobalTraj(nh, private_nh, coverage_path);

    Eigen::RowVector2d robot_current_position; // [m] Robot's current [X, Y] coordinates in the global map
    robot_current_position << 0.0, 0.0;
    double robot_current_path_velocity = 0;        // [m/s] Robot's current path velocity

    GlobalTraj.get_robot_current_state(robot_current_position, robot_current_path_velocity);

    // std::cout << "Generating the global trajectory...\n";
    // GlobalTraj.export_global_trajectory();
    // GlobalTraj.rpm_convert();

    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_time = end_time - start_time;

    std::cout << "The global trajectory was successfully generated.\n";
    std::cout << "Elapsed time: " << 1000*elapsed_time.count() << " [msec].\n" << std::endl;

    ros::Rate loop_rate(0.2);

    while (ros::ok())
    {            
        GlobalTraj.generateTrajectory();
        GlobalTraj.publishPathAndTrajectory();
        std::cout << "Publishing the global trajectory...\n";
        ros::spinOnce();    
        loop_rate.sleep();
        std::cout << "The global trajectory was successfully published.\n";
    }

    return 0;
}
