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

#ifndef mobile_robots_node_h___
#define mobile_robots_node_h___

#include <vector>

#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <tug_mobile_robots/CallMovement.h>
#include <tug_stage_ros/SetRobotPose.h>
#include <tug_stage_ros/WheelCmdVel.h>
#include <tug_stage_ros/AckermannCmd.h>

using namespace Eigen;

namespace tug_mobile_robots
{

class MobileRobotsNode
{
private:
    ros::NodeHandle nh_;

    ros::ServiceServer set_ellipse_srv_;

    ros::ServiceClient set_robot_pose_client_;

    ros::Publisher cmd_vel_pub_;
    ros::Publisher wheel_cmd_vel_pub_;
    ros::Publisher ackermann_cmd_pub_;

    int mode_;

    unsigned int step_count_;

    std::vector<Vector3d> robot_poses_;
    std::vector<Vector2d> robot_vels_;
    std::vector<Vector2d> robot_wheel_vels_;
    std::vector<Vector2d> robot_ackermann_cmds_;

public:
    MobileRobotsNode();

    virtual ~MobileRobotsNode();

    void init();

    void step();

private:
    bool callMovementCB(tug_mobile_robots::CallMovementRequest &request, tug_mobile_robots::CallMovementResponse &response);
};


} // end namespace tug_mobile_robots

#endif // mobile_robots_node_h___
