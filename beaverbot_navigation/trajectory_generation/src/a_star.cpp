/*
 * ===============================================================================
 * a_star.cpp
 * Author: Schaefle Tobias
 * Date: 18.02.2021
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * A simple A* algorithm to generate a piecewise linear path for the
 * trajectory generation.
 * ===============================================================================
 */

#include "../include/trajectory_generation/a_star.h"

Astar::Astar(ros::NodeHandle nh, ros::NodeHandle private_nh, double cell_size)
{
    nh_ = nh;
    private_nh_ = private_nh;
    cell_size_ = cell_size;
    heading_res_ = 2 * M_PI / 8;

    completed_ = false;

    motion_[0][0] = 1; motion_[0][1] = 0;		    // up
    motion_[1][0] = 1; motion_[1][1] = 1;          // up left
    motion_[2][0] = 0; motion_[2][1] = 1;          // left
    motion_[3][0] = -1; motion_[3][1] = 1;         // down left
    motion_[4][0] = -1; motion_[4][1] = 0;          // down
    motion_[5][0] = -1; motion_[5][1] = -1;          // down right
    motion_[6][0] = 0; motion_[6][1] = -1;		    // right
    motion_[7][0] = 1; motion_[7][1] = -1;		    // up right

    // first try straight motion then 45deg, 90deg, 135deg, 180deg
    direction_ = {0, 1, -1, 2, -2, 3, -3, 4};

    start_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("a_star_start", 1);
    goal_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("a_star_goal", 1);
    path_pub_ = private_nh_.advertise<nav_msgs::Path>("a_star_path", 1);
    intermediate_pub_ = private_nh_.advertise<geometry_msgs::PoseArray>("a_star_intermediate", 1);

    path_nodes_.clear();

    listener_ = new tf::TransformListener();

    while(true)
    {
        try
        {
            listener_->lookupTransform("/map", "/odom", ros::Time(0), transform_map_odom_);
            listener_->lookupTransform("/map", "/rear_axis_center", ros::Time(0), transform_map_rearaxis_);
            std::cout << "Transform received" << std::endl;
            std::cout << "Transform:\n" << transform_map_rearaxis_.getOrigin().getX() << ", \t" << transform_map_rearaxis_.getOrigin().getY() << std::endl;
            break;
        }
        catch (tf::TransformException &ex)
        {
            //ROS_WARN("Test");
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }


    initializeMessages();
}

void Astar::initializeMessages()
{
    // get map
    ROS_INFO("Set Map.");
    static_map_ = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("inflate_static_map_node/inflated_map", nh_, ros::Duration(15.0));
    if (static_map_ == nullptr)
    {
        ROS_ERROR("No static map received.");
    }
    else
    {
        num_width_ = static_cast<int>(static_map_->info.width * static_map_->info.resolution / cell_size_);
        num_height_ = static_cast<int>(static_map_->info.height * static_map_->info.resolution / cell_size_);
    }

    // get start
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> shared_start;
    ROS_INFO("Set Start.");
    shared_start = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("initialpose", nh_, ros::Duration(5.0));
    if (shared_start == nullptr)
    {
        /*
        geometry_msgs::PoseStamped start;
        tf::Stamped<tf::Pose> start_tf;
        start_tf.setBasis(transform_map_rearaxis_.getBasis());
        start_tf.setOrigin(transform_map_rearaxis_.getOrigin());
        tf::poseStampedTFToMsg(start_tf, start);
        getPose(start.pose, start_node_);
        if (!isSafe(start_node_.head<2>()))
        {
            ROS_WARN("ASTAR_CLASS_WARN: Start node outside map or inside obstacle.");
            return;
        }
        //getPose(shared_start->pose.pose, start_node_);
        ROS_INFO_STREAM("Start node:\n" << start_node_);
        start_ << start_node_.x() * cell_size_, start_node_.y() * cell_size_, start_node_.z() * heading_res_;
        start_msg_.header.frame_id = "map";
        start_msg_.pose.position.x = start_.x();
        start_msg_.pose.position.y = start_.y();
        start_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(start_.z());

        start_node_ << 4, 35, 0;
        start_ << start_node_.x() * cell_size_, start_node_.y() * cell_size_, start_node_.z() * heading_res_;
        start_msg_.header.frame_id = "map";
        start_msg_.pose.position.x = start_.x();
        start_msg_.pose.position.y = start_.y();
        start_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(start_.z());
        */

        //start_node_ << 258, 154, 0;
        start_node_ << 220, 105, 0;
        start_ << start_node_.x() * cell_size_, start_node_.y() * cell_size_, start_node_.z() * heading_res_;
        start_msg_.header.frame_id = "map";
        start_msg_.pose.position.x = start_.x();
        start_msg_.pose.position.y = start_.y();
        start_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(start_.z());


//        start_node_ << 5, 20, 0;
//        start_ << start_node_.x() * cell_size_, start_node_.y() * cell_size_, start_node_.z() * heading_res_;
//        start_msg_.header.frame_id = "map";
//        start_msg_.pose.position.x = start_.x();
//        start_msg_.pose.position.y = start_.y();
//        start_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(start_.z());
    }
    else
    {
        geometry_msgs::Pose trans;
        transformPose(shared_start->pose.pose, trans);
        getPose(trans, start_node_);
        if (!isSafe(start_node_.head<2>()))
        {
            ROS_WARN("ASTAR_CLASS_WARN: Start node outside map or inside obstacle.");
            return;
        }
        //getPose(shared_start->pose.pose, start_node_);
        ROS_INFO_STREAM("Start node:\n" << start_node_);
        start_ << start_node_.x() * cell_size_, start_node_.y() * cell_size_, start_node_.z() * heading_res_;
        start_msg_.header.frame_id = "map";
        start_msg_.pose.position.x = start_.x();
        start_msg_.pose.position.y = start_.y();
        start_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(start_.z());
    }
    start_pub_.publish(start_msg_);

    // get goal
    boost::shared_ptr<geometry_msgs::PoseStamped const> shared_goal;
    ROS_INFO("Set Goal.");
    bool set_goals = true;
    intermediate_msg_.header.frame_id = "map";
    geometry_msgs::Pose pose;
    while (set_goals)
    {
        shared_goal = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/move_base_simple/goal", nh_, ros::Duration(10.0));
        if (shared_goal == nullptr)
        {
            if (goal_nodes_.empty())
            {
                /*
                goal_node_ << 18, 21, 6;
                pose.position.x = goal_node_.x() * cell_size_;
                pose.position.y = goal_node_.y() * cell_size_;
                pose.orientation = tf::createQuaternionMsgFromYaw(goal_node_.z() * heading_res_);
                intermediate_msg_.poses.push_back(pose);
                goal_nodes_.push_back(goal_node_);

                goal_node_ << 34, 7, 0;
                //goal_node_ << 34, 35, 0;
                goal_msg_.header.frame_id = "map";
                goal_msg_.pose.position.x = goal_.x();
                goal_msg_.pose.position.y = goal_.y();
                goal_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(goal_.z());
                goal_nodes_.push_back(goal_node_);
                */

//                goal_node_ << 7, 20, 0;
//                pose.position.x = goal_node_.x() * cell_size_;
//                pose.position.y = goal_node_.y() * cell_size_;
//                pose.orientation = tf::createQuaternionMsgFromYaw(goal_node_.z() * heading_res_);
//                intermediate_msg_.poses.push_back(pose);
//                goal_nodes_.push_back(goal_node_);

                /*
                goal_node_ << 11, 22, 2;
                pose.position.x = goal_node_.x() * cell_size_;
                pose.position.y = goal_node_.y() * cell_size_;
                pose.orientation = tf::createQuaternionMsgFromYaw(goal_node_.z() * heading_res_);
                intermediate_msg_.poses.push_back(pose);
                goal_nodes_.push_back(goal_node_);

                goal_node_ << 11, 24, 2;
                pose.position.x = goal_node_.x() * cell_size_;
                pose.position.y = goal_node_.y() * cell_size_;
                pose.orientation = tf::createQuaternionMsgFromYaw(goal_node_.z() * heading_res_);
                intermediate_msg_.poses.push_back(pose);
                goal_nodes_.push_back(goal_node_);
                */

//                goal_node_ << 11, 19, 7;
//                //goal_node_ << 34, 35, 0;
//                goal_msg_.header.frame_id = "map";
//                goal_msg_.pose.position.x = goal_.x();
//                goal_msg_.pose.position.y = goal_.y();
//                goal_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(goal_.z());
//                goal_nodes_.push_back(goal_node_);

//                goal_node_ << 11, 24, 2;
//                //goal_node_ << 34, 35, 0;
//                goal_msg_.header.frame_id = "map";
//                goal_msg_.pose.position.x = goal_.x();
//                goal_msg_.pose.position.y = goal_.y();
//                goal_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(goal_.z());
//                goal_nodes_.push_back(goal_node_);

//                goal_node_ << 15, 18, 0;
//                //goal_node_ << 34, 35, 0;
//                goal_msg_.header.frame_id = "map";
//                goal_msg_.pose.position.x = goal_.x();
//                goal_msg_.pose.position.y = goal_.y();
//                goal_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(goal_.z());
//                goal_nodes_.push_back(goal_node_);

                //goal_node_ << 315, 154, 0;
                goal_node_ << 258, 105, 0;
                //goal_node_ << 34, 35, 0;
                goal_msg_.header.frame_id = "map";
                goal_msg_.pose.position.x = goal_.x();
                goal_msg_.pose.position.y = goal_.y();
                goal_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(goal_.z());
                goal_nodes_.push_back(goal_node_);

                //goal_node_ << 315, 134, 6;
                goal_node_ << 258, 65, 6;
                //goal_node_ << 34, 35, 0;
                goal_msg_.header.frame_id = "map";
                goal_msg_.pose.position.x = goal_.x();
                goal_msg_.pose.position.y = goal_.y();
                goal_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(goal_.z());
                goal_nodes_.push_back(goal_node_);


                /*
                goal_node_ << 15, 20, 0;
                //goal_node_ << 34, 35, 0;
                goal_msg_.header.frame_id = "map";
                goal_msg_.pose.position.x = goal_.x();
                goal_msg_.pose.position.y = goal_.y();
                goal_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(goal_.z());
                goal_nodes_.push_back(goal_node_);
                */
            }
            else
            {
                goal_node_ = goal_nodes_.back();
                intermediate_msg_.poses.pop_back();
            }
            goal_ << goal_node_.x() * cell_size_, goal_node_.y() * cell_size_, goal_node_.z() * heading_res_;
            goal_msg_.header.frame_id = "map";
            goal_msg_.pose.position.x = goal_.x();
            goal_msg_.pose.position.y = goal_.y();
            goal_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(goal_.z());
            set_goals = false;
        }
        else
        {
            geometry_msgs::Pose trans;
            transformPose(shared_goal->pose, trans);
            getPose(trans, goal_node_);
            if (!isSafe(goal_node_.head<2>()))
            {
                ROS_WARN("ASTAR_CLASS_WARN: Goal node outside map or inside obstacle.");
                return;
            }
            pose.position.x = goal_node_.x() * cell_size_;
            pose.position.y = goal_node_.y() * cell_size_;
            pose.orientation = tf::createQuaternionMsgFromYaw(goal_node_.z() * heading_res_);
            intermediate_msg_.poses.push_back(pose);
            //getPose(shared_goal->pose, goal_node_);
            ROS_INFO_STREAM("Goal:\n" << goal_node_);
            goal_nodes_.push_back(goal_node_);
        }
    }

    goal_pub_.publish(goal_msg_);
    intermediate_pub_.publish(intermediate_msg_);

    path_msg_.header.frame_id = "odom";
    path_msg_.poses.clear();
}

bool Astar::runAstar()
{
    if (completed_)
    {
        path_pub_.publish(path_msg_);
        return true;
    }
    if (static_map_ == nullptr)
    {
        ROS_ERROR("ASTAR_CLASS_ERROR: No static map received.");
        return false;
    }

    open_list_.clear();
    path_msg_.poses.clear();

    unsigned int goal_counter = 0;

    // initialize nodes
    std::vector< std::vector< std::vector<NodePtr> > > nodes;
    setNodeList(nodes);

    // set start
    Node temp;
    NodePtr start = boost::make_shared<Node>(temp);
    start->position << start_node_.x(), start_node_.y();
    start->heading_idx = start_node_.z();
    updateHeuristicCost(start);
    start->cost_so_far = 0.0;
    start->cost = start->heuristic_cost + start->cost_so_far;
    start->direction = 0;

    nodes[start_node_.x()][start_node_.y()][start_node_.z()] = start;

    open_list_.push(*start.get());

    Eigen::Vector2i action;

    goal_ << goal_nodes_.at(goal_counter).x() * cell_size_, goal_nodes_.at(goal_counter).y() * cell_size_, goal_nodes_.at(goal_counter).z() * heading_res_;

    while (!open_list_.empty())
    {
        Node temp = open_list_.top();
        open_list_.pop();
        nodes[temp.position.x()][temp.position.y()][temp.heading_idx]->open = false;
        NodePtr node = nodes[temp.position.x()][temp.position.y()][temp.heading_idx];

        // check if at goal
        if (node->position.x() == goal_nodes_.at(goal_counter).x()
                && node->position.y() == goal_nodes_.at(goal_counter).y()
                && node->heading_idx == goal_nodes_.at(goal_counter).z())
        {
            std::cout << "Goal found." << std::endl;

            updatePath(node);

            Eigen::Vector3i current_goal_node(goal_nodes_.at(goal_counter).x(), goal_nodes_.at(goal_counter).y(), goal_nodes_.at(goal_counter).z());

            goal_counter++;
            if (goal_counter == goal_nodes_.size())
            {
                // publish data
                completed_ = true;
                publishPath();
                return true;
            }
            else
            {
                // reset nodes and openlist
                setNodeList(nodes);
                open_list_.clear();
                Node temp;
                NodePtr start = boost::make_shared<Node>(temp);
                start->position << current_goal_node.x(), current_goal_node.y();
                start->heading_idx = current_goal_node.z();
                updateHeuristicCost(start);
                start->cost_so_far = 0.0;
                start->cost = start->heuristic_cost + start->cost_so_far;
                start->direction = 0;

                goal_ << goal_nodes_.at(goal_counter).x() * cell_size_, goal_nodes_.at(goal_counter).y() * cell_size_, goal_nodes_.at(goal_counter).z() * heading_res_;

                nodes[current_goal_node.x()][current_goal_node.y()][current_goal_node.z()] = start;

                open_list_.push(*start.get());

                continue;
            }
        }

        for (unsigned int ii = 0; ii < direction_.size(); ++ii)
        {
            Node foo;
            NodePtr new_node = boost::make_shared<Node>(foo);
            int entry = node->heading_idx + direction_[ii];
            if (entry < 0)
            {
                entry += direction_.size();
            }
            else if (entry >= direction_.size())
            {
                entry -= direction_.size();
            }

            action << motion_[entry][0], motion_[entry][1];
            new_node->position = node->position + action;
            new_node->heading_idx = entry;

            // check if new node is open and inside map
            if (new_node->position.x() > -1 && new_node->position.x() < num_width_ &&
                    new_node->position.y() > -1 && new_node->position.y() < num_height_)
            {
                // check if inside obstacle
                if (isSafe(new_node->position))
                {
                    if (nodes[new_node->position.x()][new_node->position.y()][new_node->heading_idx]->open)
                    {
                        // update data
                        new_node->direction = direction_[ii];
                        new_node->parent = node.get();
                        new_node->heading_idx = entry;

                        // update costs
                        updateHeuristicCost(new_node);
                        updateCostSoFar(new_node);
                        new_node->cost = new_node->heuristic_cost + new_node->cost_so_far;
                        open_list_.push(*new_node.get());
                        nodes[new_node->position.x()][new_node->position.y()][new_node->heading_idx] = new_node;
                    }
                    else
                    {
                        // update data
                        new_node->direction = direction_[ii];
                        new_node->parent = node.get();
                        new_node->heading_idx = entry;

                        // update costs
                        updateHeuristicCost(new_node);
                        updateCostSoFar(new_node);
                        new_node->cost = new_node->heuristic_cost + new_node->cost_so_far;

                        if (new_node->cost_so_far < nodes[new_node->position.x()][new_node->position.y()][new_node->heading_idx]->cost_so_far)
                        {
                            open_list_.push(*new_node.get());
                            nodes[new_node->position.x()][new_node->position.y()][new_node->heading_idx] = new_node;
                        }
                    }
                }
            }
        }
    }

    std::cout << "No solution found" << std::endl;
    return false;
}

inline void Astar::updateHeuristicCost(NodePtr &node)
{
    // euclidean distance
    Eigen::Vector2d pos(node->position.x() * cell_size_, node->position.y() * cell_size_);
    node->heuristic_cost = (pos - goal_.head<2>()).norm();
}

inline void Astar::updateCostSoFar(NodePtr &node)
{
    // cost based on direction change
    double cost;
    double add;
    if (node->direction == 0)
    {
        add = 1;
    }
    else if (std::abs(node->direction) == 1)
    {
        add = 1.3;
    }
    else if (std::abs(node->direction) == 2)
    {
        add = 1.5;
    }
    else if (std::abs(node->direction) == 3)
    {
        add = 1.7;
    }
    else if (std::abs(node->direction) == 4)
    {
        add = 1.9;
    }

    //add = 1;
    if ((std::abs(node->direction) + 2) % 2 == 0)
    {
        cost = cell_size_* add;
    }
    else
    {
        cost = cell_size_ * add * std::sqrt(2);// * add;
    }
    node->cost_so_far = node->parent->cost_so_far + cost;
}

/// node containts cell number of width and height and orientation index
inline void Astar::getPose(const geometry_msgs::Pose &pose, Eigen::Vector3i &node)
{
    double yaw = tf::getYaw(pose.orientation);
    if (yaw < 0)
    {yaw += 2.0 * M_PI;}

    //unsigned int idx = static_cast<unsigned int>(yaw / heading_res_);
    //idx %= 8;

    unsigned int idx = getOrientationIdx(yaw);

    int x = std::ceil(pose.position.x / cell_size_);
    int y = std::ceil(pose.position.y / cell_size_);

    node << x, y, idx;
}

inline void Astar::transformPose(const geometry_msgs::Pose &pose, geometry_msgs::Pose &trans_pose)
{
    tf::Transform orig_tf, point_tf;
    tf::poseMsgToTF(static_map_->info.origin, orig_tf);
    tf::poseMsgToTF(pose, point_tf);

    point_tf = transform_map_odom_ * point_tf;

    tf::poseTFToMsg(point_tf, trans_pose);
}

void Astar::setNodeList(std::vector<std::vector<std::vector<NodePtr> > > &nodes)
{
    Node bar;
    NodePtr foo = boost::make_shared<Node>(bar);
    foo->open = true;
    nodes.resize(num_width_);
    for (unsigned int ii = 0; ii < num_width_; ++ii)
    {
        nodes.at(ii).resize(num_height_);
        for (unsigned int jj = 0; jj < num_height_; ++jj)
        {
            nodes.at(ii).at(jj).resize(direction_.size());
            for (unsigned int kk = 0; kk < direction_.size(); ++kk)
            {
                nodes[ii][jj][kk] = foo;
            }
        }
    }
}

bool Astar::isSafe(const Eigen::Vector2i &pos)
{
    // get real coordinates
    float x = pos.x() * cell_size_;
    float y = pos.y() * cell_size_;

    // get map cell entries
    int cell_x = static_cast<int>(x / static_map_->info.resolution);
    int cell_y = static_cast<int>(y / static_map_->info.resolution);

    // check if in map
    if ((cell_x < 0 || cell_x >= static_cast<signed int>(static_map_->info.width))
            || (cell_y < 0 || cell_y >= static_cast<signed int>(static_map_->info.height)))
    {
        return false;
    }

    unsigned int index = static_cast<unsigned int>(cell_x) + static_map_->info.width * static_cast<unsigned int>(cell_y);

    // check if obstacle
    if (static_map_->data[index] > 0)
    {
        return false;
    }
    return true;
}

inline unsigned int Astar::getOrientationIdx(const double angle)
{
    if (angle < M_PI / 8)
    {
        return 0;
    }
    else if (angle < 3 * M_PI / 8)
    {
        return 1;
    }
    else if (angle < 5 * M_PI / 8)
    {
        return 2;
    }
    else if (angle < 7 * M_PI / 8)
    {
        return 3;
    }
    else if (angle < 9 * M_PI / 8)
    {
        return 4;
    }
    else if (angle < 11 * M_PI / 8)
    {
        return 5;
    }
    else if (angle < 13 * M_PI / 8)
    {
        return 6;
    }
    else if (angle < 15 * M_PI / 8)
    {
        return 7;
    }
    else if (angle < 17 * M_PI / 8)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

void Astar::updatePath(NodePtr &final)
{
    tracePath(final.get());

    std::reverse(temp_nodes_.begin(), temp_nodes_.end());

    path_nodes_.insert(path_nodes_.end(), temp_nodes_.begin(), temp_nodes_.end());

    temp_nodes_.clear();
}

void Astar::publishPath()
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "odom";

    tf::Stamped<tf::Pose> start_tf;
    geometry_msgs::PoseStamped start;
    start.header.frame_id = "odom";
    tf::Transform trans_odom_rear = transform_map_odom_.inverse() * transform_map_rearaxis_;
    start_tf.setBasis(trans_odom_rear.getBasis());
    start_tf.setOrigin(trans_odom_rear.getOrigin());
    tf::poseStampedTFToMsg(start_tf, start);
    path_msg_.poses.push_back(start);

    std::cout << start << std::endl;
    ROS_WARN("Rear axis included in the front of the path message.");

    int direction = path_nodes_[0].z();

    pose.pose.position.x = path_nodes_[0].x() * cell_size_ - transform_map_odom_.getOrigin().getX();
    pose.pose.position.y = path_nodes_[0].y() * cell_size_ - transform_map_odom_.getOrigin().getY();
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(path_nodes_[0].z() * heading_res_);
    //path_msg_.poses.push_back(pose);

    std::cout << "First A* point (x, y, t):\t(" << pose.pose.position.x << ", " << pose.pose.position.y << ", " << path_nodes_[0].z() * heading_res_ << ")" << std::endl;

    for (unsigned int ii = 1; ii < (path_nodes_.size() - 1); ++ii)
    {
        if (direction == path_nodes_[ii+1].z())
        {
            continue;
        }
        direction = path_nodes_[ii+1].z();
        pose.pose.position.x = path_nodes_[ii].x() * cell_size_ - transform_map_odom_.getOrigin().getX();
        pose.pose.position.y = path_nodes_[ii].y() * cell_size_ - transform_map_odom_.getOrigin().getY();
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(path_nodes_[ii].z() * heading_res_);
        path_msg_.poses.push_back(pose);

        std::cout << "First A* point (x, y, t):\t(" << pose.pose.position.x << ", " << pose.pose.position.y << ", " << path_nodes_[ii].z() * heading_res_ << ")" << std::endl;
    }
    pose.pose.position.x = path_nodes_.back().x() * cell_size_ - transform_map_odom_.getOrigin().getX();
    pose.pose.position.y = path_nodes_.back().y() * cell_size_ - transform_map_odom_.getOrigin().getY();
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(path_nodes_.back().z() * heading_res_);
    path_msg_.poses.push_back(pose);

    std::cout << "First A* point (x, y, t):\t(" << pose.pose.position.x << ", " << pose.pose.position.y << ", " << path_nodes_.back().z() * heading_res_ << ")" << std::endl;

    //ROS_WARN("ASTAR_CLASS: Currently hardcoded coordinate transform.");
    path_pub_.publish(path_msg_);
}

void Astar::tracePath(Node *node)
{
    if (node == nullptr)
    {
        return;
    }
    Eigen::Vector3i val(node->position.x(), node->position.y(), node->heading_idx);
    temp_nodes_.push_back(val);
    tracePath(node->parent);
    return;
}







