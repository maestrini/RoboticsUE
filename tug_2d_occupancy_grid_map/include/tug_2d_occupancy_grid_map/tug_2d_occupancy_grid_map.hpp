/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Johannes Maurer,
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

#ifndef rae_2d_attribute_map_hpp___
#define rae_2d_attribute_map_hpp___

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <boost/utility.hpp>
#include <boost/shared_array.hpp>


namespace tug_2d_occupancy_grid_map
{

class Tug2dOccupancyGridMap : boost::noncopyable
{
private:
    ros::NodeHandle nh_;

    unsigned int size_x_;
    unsigned int size_y_;
    unsigned int num_cells_;
    double resolution_;
    double origin_x_;
    double origin_y_;

    boost::shared_array<double> map_;

    ros::Publisher occupancy_grid_pub_;

    nav_msgs::OccupancyGrid occupancy_grid_;

    ros::Timer timer;

public:
    Tug2dOccupancyGridMap(ros::NodeHandle nh);

    virtual ~Tug2dOccupancyGridMap();

    void init(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y);

    bool worldToMap(double world_x, double world_y, int& map_x, int& map_y) const;

    double updateCell(unsigned int cell_index, double value);

    double updateCell(unsigned int cell_x, unsigned int cell_y, double value);

    unsigned int getSizeX()
    {
        return size_x_;
    }

    unsigned int getSizeY()
    {
        return size_y_;
    }

    double getResolution()
    {
        return resolution_;
    }

    /**
     * @brief  Returns the value and the covariance of the given cell index
     * @param  index The index of the cell
     * @return  value The value of the cell
     */
    double getCell(unsigned int cell_index);

    /**
     * @brief  Returns the value and the covariance of the given map coordinates
     * @param  cell_x The x coordinate of the cell
     * @param  cell_y The y coordinate of the cell
     * @return  value The value of the cell
     */
    double getCell(unsigned int cell_x, unsigned int cell_y);

private:

    /**
     * @brief  Computes the associated index from map coordinates
     * @param  cell_x The x coordinate of the cell
     * @param  cell_y The y coordinate of the cell
     * @return The associated index of the cell
     */
    inline unsigned int getIndex(unsigned int cell_x, unsigned int cell_y) const
    {
        return cell_y * size_x_ + cell_x;
    }

    /**
     * @brief  Computes the associated map coordinates from index
     * @param  index The index of the cell
     * @param  cell_x The x coordinate of the cell
     * @param  cell_y The y coordinate of the cell
     */
    inline void indexToCells(unsigned int index, unsigned int& cell_x, unsigned int& cell_y) const
    {
        cell_x = index / size_x_;
        cell_y = index - (cell_y * size_x_);
    }

    void timedCallback(const ros::TimerEvent&);

};

} // end namespace

#endif
