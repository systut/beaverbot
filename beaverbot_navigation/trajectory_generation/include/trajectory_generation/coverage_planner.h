/**
 * @file coverage_planner.h
 * @brief This file contains the class definition for the coverage planner.
 * @author Tran Viet Thanh
 * @date 2024-10-24
 */
#ifndef COVERAGE_PLANNER_H
#define COVERAGE_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Eigen>

class CoveragePlanner
{
public:
    CoveragePlanner(){};
    ~CoveragePlanner(){};

    bool generate(double length, double width, nav_msgs::Path &coverage_path);

    bool generate(double length, double width, int iteration, Eigen::MatrixXd &coverage_path);

private:
    double length_;
    double width_;

    double n_width = 1.5; 
    double w_turning_min = 4; 
    std::vector<double> init_position = {0, 0}; 

    void _generatePose(std::vector<std::vector<double>> p_lower, std::vector<std::vector<double>> p_middle, std::vector<std::vector<double>> p_upper, int k, nav_msgs::Path &coverage_path);
};

#endif // COVERAGE_PLANNER_H