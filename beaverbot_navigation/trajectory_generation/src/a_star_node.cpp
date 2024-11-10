/*
 * ===============================================================================
 * a_star_node.cpp
 * Author: Schaefle Tobias
 * Date: 18.02.2021
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * Main call for the A* algorithm.
 * ===============================================================================
 */

#include <ros/ros.h>

#include "../include/trajectory_generation/a_star.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double cell_size = 0.5;

    Astar a_star(nh, private_nh, cell_size);

    // for now as publisher but later potentially a service
    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        ros::spinOnce();

        a_star.runAstar();
        //break;

        loop_rate.sleep();
    }

    return 0;
}
