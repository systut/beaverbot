/** @file a_star.h
 *  @brief Function prototypes for the A* algorithm.
 *
 *  This contains the prototypes for the A* algorithm.
 *  
 *  @author Schaefle Tobias
 *  @author Tran Viet Thanh
 *  @bug No known bugs.
 */

#ifndef EULER_GENERATOR_A_STAR_H
#define EULER_GENERATOR_A_STAR_H

#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include <eigen3/Eigen/Dense>
#include <boost/heap/binomial_heap.hpp>

/**
 * @brief Node structure for the A* algorithm
 * @param start The start position
 * @param goal The goal position
 * @param heading The heading of the start position
 * @param map The map
 * @param heading_map The heading map
 * @param step_size The step size
 * @param heuristic_weight The weight of the heuristic
 * @return The path
 */
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

/**
 * @brief Type definition for a shared pointer to a Node
 */
typedef boost::shared_ptr<Node> NodePtr;

/**
 * @brief Struct to compare two nodes
 * @param lhs The left hand side node
 * @param rhs The right hand side node
 * @return True if the left hand side node has a higher cost than the right hand side node
 */
struct CompareNodes
{
    bool operator()(Node lhs, Node rhs) const
    {
        return lhs.cost > rhs.cost;
    }
};

/**
 * @brief Type definition for the priority queue
 * @param Node The node
 * @param CompareNodes The comparison function 
 * @return The priority queue
 */
typedef boost::heap::binomial_heap<Node, boost::heap::compare<CompareNodes> > priorityQueue;

/**
 * @brief Class for the A* algorithm
 * @param cell_size The cell size
 * @return The A* algorithm
 */
class Astar
{
public:
    /**
     * @brief Constructor
     * @param cell_size The cell size
     * @return The A* algorithm
     */
    Astar(double cell_size);

    /**
     * @brief Execute the A* algorithm
     */
    bool runAstar();

private:
    /** 
     * @brief Check if the position is safe
     * @param position The position
     */
    bool isSafe(const Eigen::Vector2i &position);

    /**
     * @brief Check if the position is valid
     * @param position The position
     */
    inline void getPose(const geometry_msgs::Pose &pose, Eigen::Vector3i &node);

    inline unsigned int getOrientationIdx(const double angle);

    inline void updateCostSoFar(NodePtr &node);

    inline void updateHeuristicCost(NodePtr &node);

    void setNodeList(std::vector< std::vector< std::vector<NodePtr> > > &nodes);

    void updatePath(NodePtr &final);

    void publishPath();

    void tracePath(Node *node);

    Eigen::Vector3i start_node_, goal_node_;

    Eigen::Vector3d start_, goal_;

    std::vector<Eigen::Vector3i> path_nodes_;

    std::vector<Eigen::Vector3i> temp_nodes_;


    std::vector<Eigen::Vector3i> goal_nodes_;

    std::array< std::array<int, 2>, 8 > motion_;

    std::vector<int> direction_;

    double cell_size_;

    int num_width_;

    int num_height_;

    double heading_resolution_;

    bool completed_;

    priorityQueue open_list_;
};

#endif // EULER_GENERATOR_A_STAR_H
