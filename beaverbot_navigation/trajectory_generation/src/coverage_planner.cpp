#include "trajectory_generation/coverage_planner.h"

bool CoveragePlanner::generate(double length, double width, int iteration, Eigen::MatrixXd &coverage_path)
{
    int n = static_cast<int>(width / n_width);
    if (width - n * n_width != 0) {
        n += 1;
    }

    std::vector<std::vector<double>> p_lower, p_upper;

    for (int i = 0; i <= n + 1; ++i) {
        p_lower.push_back({init_position[0] + i * n_width, init_position[1]});
        p_upper.push_back({init_position[0] + i * n_width, init_position[1] + length});
    }

    int n_upper_turn = static_cast<int>(std::ceil(w_turning_min / n_width));
    int n_turn = n - n_upper_turn;

    coverage_path.resize((n_turn+1) * 4, 2);
    
    for (int k = 0; k <= n_turn; ++k) {
        coverage_path.row(k * 4) = Eigen::RowVector2d(p_lower[k][1], p_lower[k][0]);
        coverage_path.row(k * 4 + 1) = Eigen::RowVector2d(p_upper[k][1], p_upper[k][0]);
        coverage_path.row(k * 4 + 2) = Eigen::RowVector2d(p_upper[k][1], p_upper[k+n_upper_turn][0]);
        coverage_path.row(k * 4 + 3) = Eigen::RowVector2d(p_lower[k][1], p_lower[k+n_upper_turn][0]);
    }

    int original_rows = coverage_path.rows();

    coverage_path.conservativeResize(original_rows * iteration, Eigen::NoChange);

    for (int i = 1; i < iteration; ++i) {
        coverage_path.middleRows(i * original_rows, original_rows) = coverage_path.topRows(original_rows);
    }

    return true;
}

bool CoveragePlanner::generate(double length, double width, nav_msgs::Path &coverage_path)
{
    int n = static_cast<int>(width / n_width);
    if (width - n * n_width != 0) {
        n += 1;
    }

    std::vector<std::vector<double>> p_lower, p_middle, p_upper;

    for (int i = 0; i <= n + 1; ++i) {
        p_lower.push_back({init_position[0] - i * n_width, init_position[1]});
        p_middle.push_back({init_position[0] - i * n_width, init_position[1] + length / 2});
        p_upper.push_back({init_position[0] - i * n_width, init_position[1] + length});
    }

    int n_upper_turn = static_cast<int>(std::ceil(w_turning_min / n_width));
    int n_turn = n - n_upper_turn;

    coverage_path.poses.clear();
    coverage_path.header.frame_id = "map"; 

    for (int k = 0; k <= n_turn; ++k) {
        _generatePose(p_lower, p_middle, p_upper, k, coverage_path);
    }

    return true;
}

void CoveragePlanner::_generatePose(
    std::vector<std::vector<double>> p_lower, 
    std::vector<std::vector<double>> p_middle, 
    std::vector<std::vector<double>> p_upper, 
    int k, nav_msgs::Path &coverage_path)
{
    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = p_lower[k][0];
    pose.pose.position.y = p_lower[k][1];
    pose.pose.position.z = 0.0;
    coverage_path.poses.push_back(pose);

    pose.pose.position.x = p_middle[k][0];
    pose.pose.position.y = p_middle[k][1];
    coverage_path.poses.push_back(pose);

    pose.pose.position.x = p_upper[k][0];
    pose.pose.position.y = p_upper[k][1];
    coverage_path.poses.push_back(pose);
}