/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Hermann Steffan
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
 *   * All advertising materials mentioningfeatures or use of this
 *     software must display the following acknowledgement: “This product
 *     includes software developed by Graz University of Technology and
 *     its contributors.”
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

#include <tug_simple_turtle/simple_turtle.h>
#include <stdio.h>

namespace SimpleTurtle
{

SimpleTurtle::SimpleTurtle() :
    nh_(),
    linear_(0.0),
    angular_(0.0),
    button_0_pressed_(false),
    occupancy_grid_map_(nh_)
{

	poses = 0;
	
	posefile.open("pose.mat", std::ios::out|std::ios::trunc);
	odomfile.open("odom.mat", std::ios::out|std::ios::trunc);
		
}

SimpleTurtle::~SimpleTurtle()
{
	posefile.close();
	odomfile.close();
}

void SimpleTurtle::init()
{
    occupancy_grid_map_.init(2048, 2048, 0.05, -51.2, -51.2);

    bumper_sub_ = nh_.subscribe<kobuki_msgs::BumperEvent>("/mobile_base/events/bumper", 10, &SimpleTurtle::bumberCallback, this);
    button_sub_ = nh_.subscribe<kobuki_msgs::ButtonEvent>("/mobile_base/events/button", 10, &SimpleTurtle::buttonCallback, this);
    infrared_sub_ = nh_.subscribe<kobuki_msgs::DockInfraRed>("/mobile_base/events/", 10, &SimpleTurtle::dockingCallback, this);

    laser_scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 10, &SimpleTurtle::laserScanCallback, this);

    pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("/pose2D", 10, &SimpleTurtle::poseCallback, this);
    
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 10, &SimpleTurtle::odomCallback, this);

    sensor_state_sub_ = nh_.subscribe("/mobile_base/sensors/core", 10, &SimpleTurtle::sensorStateCallback, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
}

// publish cmd_vel
void SimpleTurtle::pubCmdVel(double linear, double angular)
{
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;

    vel_pub_.publish(vel);
}

// Bumper
void SimpleTurtle::bumberCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    ROS_INFO("Bumper");

    switch(msg->state)
    {
    case kobuki_msgs::BumperEvent::CENTER:
        ROS_INFO("Center");
        break;
    case kobuki_msgs::BumperEvent::LEFT:
        ROS_INFO("Left");
        break;
    case kobuki_msgs::BumperEvent::RIGHT:
        ROS_INFO("Right");
        break;

    }
}


// Buttons
void SimpleTurtle::buttonCallback(const kobuki_msgs::ButtonEvent::ConstPtr& msg)
{
    ROS_INFO("Buttons");
    switch(msg->button)
    {
    case kobuki_msgs::ButtonEvent::Button0:
        ROS_INFO("Button 0");
        button_0_pressed_ = true;
        break;
    case kobuki_msgs::ButtonEvent::Button1:
        ROS_INFO("Button 1");
        break;
    case kobuki_msgs::ButtonEvent::Button2:
        ROS_INFO("Button 2");
        break;
    }
}


// Docking sensor
void SimpleTurtle::dockingCallback(const kobuki_msgs::DockInfraRed::ConstPtr& msg)
{
    ROS_INFO("Docking");
}

// Sensor State
void SimpleTurtle::sensorStateCallback(const kobuki_msgs::SensorState::ConstPtr& msg)
{
    ROS_INFO_STREAM("left encoder= " << msg->left_encoder);
    ROS_INFO_STREAM("right encoder= " << msg->right_encoder);
}


// PointCloud
void SimpleTurtle::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("laser scan");
}

// PointCloud
void SimpleTurtle::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_INFO("odom");
}

// Scan Matcher Pose
void SimpleTurtle::poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  ROS_INFO("pose");
  // LOG THE LASER POSE
}


void SimpleTurtle::dump_pose()
{
	posefile << "happy logging\n";
}

void SimpleTurtle::dump_odom()
{
	odomfile << "happy logging\n";
}

void SimpleTurtle::publishTramsform(tf::Vector3 translation, tf::Quaternion rotation)
{
	tf::Transform t;
	t.setOrigin(translation);
	t.setRotation(rotation);
	br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "odom", "my_base"));
}

// step function
// called with 10Hz
void SimpleTurtle::step()
{
    if(button_0_pressed_)
    {
        // MAKE YOUR CHANGES HERE
        
        linear_ = 0.0;
        angular_ = 0.0;

        pubCmdVel(linear_,angular_);
        pubCmdVel(linear_,angular_);
        
    }
}

} // end namespace SimpleTurtle

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_turtle");

    SimpleTurtle::SimpleTurtle simple_turtle;

    simple_turtle.init();

    // main loop at 10Hz
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        simple_turtle.step();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
