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

#include <tug_mobile_robots/mobile_robots_node.hpp>

namespace tug_mobile_robots
{

MobileRobotsNode::MobileRobotsNode() :
    nh_(),
    step_count_(0),
    mode_(0)
{

}

MobileRobotsNode::~MobileRobotsNode()
{

}

void MobileRobotsNode::init()
{
    set_ellipse_srv_ = nh_.advertiseService("call_movement", &MobileRobotsNode::callMovementCB, this);

    set_robot_pose_client_ =  nh_.serviceClient<tug_stage_ros::SetRobotPose>("set_robot_pose");
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    wheel_cmd_vel_pub_ = nh_.advertise<tug_stage_ros::WheelCmdVel>("wheel_cmd_vel", 10);
    ackermann_cmd_pub_ = nh_.advertise<tug_stage_ros::AckermannCmd>("ackermann_cmd", 10);
}

void MobileRobotsNode::step()
{
    switch(mode_)
    {
    case CallMovementRequest::POSITION_MODE:
        if(step_count_ < robot_poses_.size())
        {
            Vector3d pose = robot_poses_[step_count_];

            tug_stage_ros::SetRobotPose srv;
            srv.request.x = pose[0];
            srv.request.y = pose[1];
            srv.request.yaw = pose[2];

            ROS_INFO_STREAM("set_robot_pose: x=" << srv.request.x << ", y=" << srv.request.y << ", yaw=" << srv.request.yaw);

            if(!set_robot_pose_client_.call(srv))
            {
              ROS_ERROR("Failed to call service set_robot_pose");
            }

            step_count_++;
        }
        break;
    case CallMovementRequest::VELOCITY_MODE:
        if(step_count_ < robot_vels_.size())
        {
            Vector2d robot_vel = robot_vels_[step_count_];

            geometry_msgs::Twist msg;
            msg.linear.x = robot_vel[0];
            msg.angular.z = robot_vel[1];

            ROS_INFO_STREAM("cmd_vel: trans=" << msg.linear.x << ", rot=" << msg.angular.z);

            cmd_vel_pub_.publish(msg);

            step_count_++;
        }
        break;
    case CallMovementRequest::WHEEL_MODE:
        if(step_count_ < robot_wheel_vels_.size())
        {
            Vector2d wheel_vel = robot_wheel_vels_[step_count_];

            tug_stage_ros::WheelCmdVel msg;
            msg.left = wheel_vel[0];
            msg.right = wheel_vel[1];

            wheel_cmd_vel_pub_.publish(msg);

            ROS_INFO_STREAM("wheel_cmd_vel: left=" << msg.left << ", right=" << msg.right);

            step_count_++;
        }
    case CallMovementRequest::ACKERMANN_MODE:
        if(step_count_ < robot_ackermann_cmds_.size())
        {
            Vector2d ackermann_cmd = robot_ackermann_cmds_[step_count_];

            tug_stage_ros::AckermannCmd msg;
            msg.velocity = ackermann_cmd[0];
            msg.steering = ackermann_cmd[1];

            ackermann_cmd_pub_.publish(msg);

            ROS_INFO_STREAM("ackermann_cmd: velocity=" << msg.velocity << ", steering=" << msg.steering);

            step_count_++;
        }
        break;
    default:
        ROS_ERROR("Invalid operation mode.");
    }
}


bool MobileRobotsNode::callMovementCB(CallMovementRequest &request, CallMovementResponse &response)
{
    ROS_INFO_STREAM("mode: " << request.mode);

    mode_ = request.mode;
    step_count_ = 0;


    robot_poses_.clear();

    // TODO: compute robot position and heading

    Vector3d pose;
    pose[0] = 0.0;  // x
    pose[1] = 0.0;  // y
    pose[2] = 0.0;  // yaw

    robot_poses_.push_back(pose);

    robot_vels_.clear();

    // TODO: compute robot linear velocity and angular velocity

    Vector2d vel;
    vel[0] = 0.0;  // trans
    vel[1] = 3.14; // rot

    robot_vels_.push_back(vel);

    robot_wheel_vels_.clear();

    // TODO: compute left and right wheel velocity

    Vector2d wheel_vel;
    wheel_vel[0] = 0.0;  // left wheel
    wheel_vel[1] = 3.14; // right wheel

    robot_wheel_vels_.push_back(wheel_vel);

    robot_ackermann_cmds_.clear();

    // TODO: compute translational velocity and steering angle

    Vector2d ackermann_cmd;
    ackermann_cmd[0] = 0.1;   // translational velocity
    ackermann_cmd[1] = 0.707; // steering angle

    robot_ackermann_cmds_.push_back(ackermann_cmd);

    return true;
}

} // end namespace tug_mobile_robots

int main(int argc, char** argv)
{

    try
    {
        ros::init(argc,argv,"mobile_robots_node");

        tug_mobile_robots::MobileRobotsNode mobile_robots_node;

        mobile_robots_node.init();

        ros::Rate loop_rate(20); // 20Hz
        while(ros::ok())
        {
            ros::spinOnce();
            mobile_robots_node.step();
            loop_rate.sleep();
        }
    }
    catch(...)
    {
        ROS_ERROR("Unhandled exception!");
        return -1;
    }

    return 0;
}
