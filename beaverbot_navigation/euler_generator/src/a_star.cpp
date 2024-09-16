
#include "euler_generator/a_star.h"


Astar::Astar(double cell_size)
{
    cell_size_ = cell_size;

    heading_resolution_ = 2 * M_PI / 8;

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

    path_nodes_.clear();
}