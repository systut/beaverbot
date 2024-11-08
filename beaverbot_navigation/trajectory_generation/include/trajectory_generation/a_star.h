/*
 * ===============================================================================
 * a_star.h
 * Author: Schaefle Tobias
 * Date: 18.02.2021
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * A simple A* algorithm to generate a piecewise linear path for the
 * trajectory generation.
 * ===============================================================================
 */

#ifndef A_STAR_H
#define A_STAR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <boost/heap/binomial_heap.hpp>

#include <math.h>

struct Node
{
    double heuristic_cost = 0.0;
    double cost_so_far = 0.0;
    double cost = 0.0;
    Eigen::Vector2i position;
    int heading_idx;
    int direction;
    bool open = true;
    Node *parent = nullptr;
};

typedef boost::shared_ptr<Node> NodePtr;

struct CompareNodes
{
    bool operator()(Node lhs, Node rhs) const
    {
        return lhs.cost > rhs.cost;
    }
};

typedef boost::heap::binomial_heap<Node, boost::heap::compare<CompareNodes> > priorityQueue;

class Astar
{
public:
    /// default constructor
    Astar(ros::NodeHandle nh, ros::NodeHandle private_nh, double cell_size);

    /// run the astar algorithm
    bool runAstar();

private:
    /// check if node is safe
    bool isSafe(const Eigen::Vector2i &pos);

    void initializeMessages();

    /// get the pose vector
    inline void getPose(const geometry_msgs::Pose &pose, Eigen::Vector3i &node);

    inline unsigned int getOrientationIdx(const double angle);

    /// update cost so far
    inline void updateCostSoFar(NodePtr &node);

    /// update heuristic cost
    inline void updateHeuristicCost(NodePtr &node);

    /// transform pose to map origin
    inline void transformPose(const geometry_msgs::Pose &pose, geometry_msgs::Pose &trans_pose);

    void setNodeList(std::vector< std::vector< std::vector<NodePtr> > > &nodes);

    void updatePath(NodePtr &final);

    /// publish the path
    void publishPath();

    /// trace path
    void tracePath(Node *node);

    /// start and goal node from rviz
    Eigen::Vector3i start_node_, goal_node_;
    Eigen::Vector3d start_, goal_;

    geometry_msgs::PoseStamped start_msg_, goal_msg_;
    geometry_msgs::PoseArray intermediate_msg_;
    nav_msgs::Path path_msg_;

    std::vector<Eigen::Vector3i> path_nodes_;
    std::vector<Eigen::Vector3i> temp_nodes_;

    boost::shared_ptr<nav_msgs::OccupancyGrid const> static_map_;

    /// intermediate nodes from rviz
    std::vector<Eigen::Vector3i> goal_nodes_;

    /// motions (8x2)
    std::array< std::array<int, 2>, 8 > motion_;
    std::vector<int> direction_;

    // get number of cells in width and height from static map
    double cell_size_;
    int num_width_;
    int num_height_;

    double heading_res_;

    bool completed_;

    ros::Publisher start_pub_;
    ros::Publisher goal_pub_;
    ros::Publisher path_pub_;
    ros::Publisher intermediate_pub_;

    priorityQueue open_list_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    tf::TransformListener *listener_;
    tf::StampedTransform transform_map_odom_;
    tf::StampedTransform transform_map_rearaxis_;
};

#endif // A_STAR_H
