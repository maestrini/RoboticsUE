/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Johannes Maurer,
 *                      Institute for Software Technology,
 *                      Graz University of Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Graz University of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <tug_2d_occupancy_grid_map/tug_2d_occupancy_grid_map.hpp>

namespace tug_2d_occupancy_grid_map
{

Tug2dOccupancyGridMap::Tug2dOccupancyGridMap(ros::NodeHandle nh) :
    nh_(nh)
{
}

Tug2dOccupancyGridMap::~Tug2dOccupancyGridMap()
{
}

void Tug2dOccupancyGridMap::init(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y)
{
    size_x_ = size_x;
    size_y_ = size_y;
    num_cells_ = size_x * size_y;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;

    occupancy_grid_.header.frame_id = "odom";
    occupancy_grid_.header.seq = 0;

    occupancy_grid_.info.height = size_y;
    occupancy_grid_.info.width = size_x;
    occupancy_grid_.info.origin.position.x = origin_x;
    occupancy_grid_.info.origin.position.y = origin_y;
    occupancy_grid_.info.origin.position.z = 0.0;
    occupancy_grid_.info.origin.orientation.w = 1.0;
    occupancy_grid_.info.resolution = resolution;

    map_.reset(new double[num_cells_]);
    for(int i = 0; i <  num_cells_; i++)
    {
        // TODO: init the map with the right value

        map_[i] = 0.0;

        occupancy_grid_.data.push_back(255);
    }


    occupancy_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);

    timer = nh_.createTimer(ros::Duration(5.0), &Tug2dOccupancyGridMap::timedCallback, this);

}

bool Tug2dOccupancyGridMap::worldToMap(double world_x, double world_y, int& map_x, int& map_y) const
{
    map_x = static_cast<int>((world_x - origin_x_) / resolution_);
    map_y = static_cast<int>((world_y - origin_y_) / resolution_);

    if (world_x < origin_x_ || world_y < origin_y_ || map_x > size_x_ || map_y > size_y_)
    {
        return false;
    }

    return true;
}

double Tug2dOccupancyGridMap::updateCell(unsigned int cell_index, double value)
{
    // TODO: update the cell

    map_[cell_index] = value;

    if(map_[cell_index] > 0.954)
    {
        occupancy_grid_.data[cell_index] = 254;
    }
    else if(map_[cell_index] < -0.954)
    {
        occupancy_grid_.data[cell_index] = 0;
    }

    return map_[cell_index];
}

double Tug2dOccupancyGridMap::updateCell(unsigned int cell_x, unsigned int cell_y, double value)
{
    return updateCell(getIndex(cell_x, cell_y), value);
}

double Tug2dOccupancyGridMap::getCell(unsigned int cell_index)
{
    return map_[cell_index];
}

double Tug2dOccupancyGridMap::getCell(unsigned int cell_x, unsigned int cell_y)
{
    return getCell(getIndex(cell_x, cell_y));
}

void Tug2dOccupancyGridMap::timedCallback(const ros::TimerEvent&)
{
    occupancy_grid_pub_.publish(occupancy_grid_);
}

} // end namespace
