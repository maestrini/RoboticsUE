/*
 *  stageros
 *  Copyright (c) 2008, Willow Garage, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**

@mainpage

@htmlinclude manifest.html
**/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>


// libstage
#include <stage.hh>

// roscpp
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <rosgraph_msgs/Clock.h>

#include "tf/transform_broadcaster.h"

#include <tug_stage_ros/Fiducials.h>
#include <tug_stage_ros/SetRobotPose.h>
#include <tug_stage_ros/WheelCmdVel.h>
#include <tug_stage_ros/AckermannCmd.h>

#define USAGE "stageros <worldfile>"
#define ODOM "odom"
#define BASE_SCAN "base_scan"
#define FIDUCIALS "fiducials"
#define BASE_POSE_GROUND_TRUTH "base_pose_ground_truth"
#define CMD_VEL "cmd_vel"
#define WHEEL_CMD_VEL "wheel_cmd_vel"
#define ACKERMANN_CMD "ackermann_cmd"
#define SET_POSE "set_robot_pose"

#define WHEEL_RADIUS 0.075
#define WHEEL_BASE 0.3
#define ACKERMANN_LENGTH 0.8

// Our node
class StageNode
{
private:
    // Messages that we'll send or receive
    tug_stage_ros::Fiducials * fiducialsMsgs;
    sensor_msgs::LaserScan *laserMsgs;
    nav_msgs::Odometry *odomMsgs;
    nav_msgs::Odometry *groundTruthMsgs;
    rosgraph_msgs::Clock clockMsg;

    // roscpp-related bookkeeping
    ros::NodeHandle n_;

    // A mutex to lock access to fields that are used in message callbacks
    boost::mutex msg_lock;

    // The models that we're interested in
    std::vector<Stg::ModelRanger *> lasermodels;
    std::vector<Stg::ModelPosition *> positionmodels;
    std::vector<Stg::ModelFiducial *> fiducialmodels;
    std::vector<ros::Publisher> fiducials_pubs_;
    std::vector<ros::Publisher> laser_pubs_;
    std::vector<ros::Publisher> odom_pubs_;
    std::vector<ros::Publisher> ground_truth_pubs_;
    std::vector<ros::Subscriber> cmdvel_subs_;
    std::vector<ros::Subscriber> wheelcmdvel_subs_;
    std::vector<ros::Subscriber> ackermanncmd_subs_;
    ros::Publisher clock_pub_;

    std::vector<ros::ServiceServer> set_robot_pose_srvs_;

    // A helper function that is executed for each stage model.  We use it
    // to search for models of interest.
    static void ghfunc(Stg::Model* mod, StageNode* node);

    static bool s_update(Stg::World* world, StageNode* node)
    {
        node->WorldCallback();
        // We return false to indicate that we want to be called again (an
        // odd convention, but that's the way that Stage works).
        return false;
    }

    // Appends the given robot ID to the given message name.  If omitRobotID
    // is true, an unaltered copy of the name is returned.
    const char *mapName(const char *name, size_t robotID);

    tf::TransformBroadcaster tf;

    // Last time that we received a velocity command
    ros::Time base_last_cmd;
    ros::Duration base_watchdog_timeout;

    // Current simulation time
    ros::Time sim_time;

    // Last time we saved global position (for velocity calculation).
    ros::Time base_last_globalpos_time;
    // Last published global pose of each robot
    std::vector<Stg::Pose> base_last_globalpos;

public:
    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    StageNode(int argc, char** argv, bool gui, const char* fname);
    ~StageNode();

    // Subscribe to models of interest.  Currently, we find and subscribe
    // to the first 'laser' model and the first 'position' model.  Returns
    // 0 on success (both models subscribed), -1 otherwise.
    int SubscribeModels();

    // Our callback
    void WorldCallback();
    
    // Do one update of the world.  May pause if the next update time
    // has not yet arrived.
    bool UpdateWorld();

    // Message callback for a MsgBaseVel message, which set velocities.
    void cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg);

    // Message callback for a Wheel Vel message, which set velocities of the left and right wheel.
    void wheelcmdvelCB(int idx, const boost::shared_ptr<tug_stage_ros::WheelCmdVel const>& msg);

    // Message callback for an ackermann command message, which set velocity and steering angle.
    void ackermanncmdCB(int idx, const boost::shared_ptr<tug_stage_ros::AckermannCmd const>& msg);

    // Service callback for SetRobotPose, which set the robot to a pose.
    bool setRobotPoseCB(int idx, tug_stage_ros::SetRobotPose::Request &request, tug_stage_ros::SetRobotPose::Response &response);

    // The main simulator object
    Stg::World* world;
};

// since stageros is single-threaded, this is OK. revisit if that changes!
const char *
StageNode::mapName(const char *name, size_t robotID)
{
    if (positionmodels.size() > 1)
    {
        static char buf[100];
        snprintf(buf, sizeof(buf), "/robot_%u/%s", (unsigned int)robotID, name);
        return buf;
    }
    else
        return name;
}

void
StageNode::ghfunc(Stg::Model* mod, StageNode* node)
{
    if (dynamic_cast<Stg::ModelRanger *>(mod))
        node->lasermodels.push_back(dynamic_cast<Stg::ModelRanger *>(mod));
    if (dynamic_cast<Stg::ModelPosition *>(mod))
        node->positionmodels.push_back(dynamic_cast<Stg::ModelPosition *>(mod));
    if (dynamic_cast<Stg::ModelFiducial *>(mod))
        node->fiducialmodels.push_back(dynamic_cast<Stg::ModelFiducial *>(mod));
}

void
StageNode::cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg)
{
    boost::mutex::scoped_lock lock(msg_lock);
    this->positionmodels[idx]->SetSpeed(msg->linear.x,
                                        msg->linear.y,
                                        msg->angular.z);
    this->base_last_cmd = this->sim_time;
}

void
StageNode::wheelcmdvelCB(int idx, const boost::shared_ptr<tug_stage_ros::WheelCmdVel const>& msg)
{
    boost::mutex::scoped_lock lock(msg_lock);

    double vel_left = msg->left * WHEEL_RADIUS;
    double vel_right = msg->right * WHEEL_RADIUS;

    double lin_x = (vel_left + vel_right) / 2.0;
    double lin_y = 0.0;
    double ang_z = (vel_right - vel_left) / WHEEL_BASE;
    this->positionmodels[idx]->SetSpeed(lin_x,
                                        lin_y,
                                        ang_z);
    this->base_last_cmd = this->sim_time;
}

void
StageNode::ackermanncmdCB(int idx, const boost::shared_ptr<tug_stage_ros::AckermannCmd const>& msg)
{
    boost::mutex::scoped_lock lock(msg_lock);



    double lin_x = msg->velocity;
    double lin_y = 0.0;
    double ang_z = msg->velocity*tan(msg->steering)/ACKERMANN_LENGTH;
    this->positionmodels[idx]->SetSpeed(lin_x,
                                        lin_y,
                                        ang_z);
    this->base_last_cmd = this->sim_time;
}

bool
StageNode::setRobotPoseCB(int idx, tug_stage_ros::SetRobotPose::Request &request, tug_stage_ros::SetRobotPose::Response &response)
{

    Stg::Pose pose(request.x, request.y, 0.0, request.yaw);
    this->positionmodels[idx]->SetPose(pose);

    return true;
}

StageNode::StageNode(int argc, char** argv, bool gui, const char* fname)
{
    this->sim_time.fromSec(0.0);
    this->base_last_cmd.fromSec(0.0);
    double t;
    ros::NodeHandle localn("~");
    if(!localn.getParam("base_watchdog_timeout", t))
        t = 0.2;
    this->base_watchdog_timeout.fromSec(t);

    // We'll check the existence of the world file, because libstage doesn't
    // expose its failure to open it.  Could go further with checks (e.g., is
    // it readable by this user).
    struct stat s;
    if(stat(fname, &s) != 0)
    {
        ROS_FATAL("The world file %s does not exist.", fname);
        ROS_BREAK();
    }

    // initialize libstage
    Stg::Init( &argc, &argv );

    if(gui)
        this->world = new Stg::WorldGui(800, 700, "Stage (ROS)");
    else
        this->world = new Stg::World();

    // Apparently an Update is needed before the Load to avoid crashes on
    // startup on some systems.
    // As of Stage 4.1.1, this update call causes a hang on start.
    //this->UpdateWorld();
    this->world->Load(fname);

    // We add our callback here, after the Update, so avoid our callback
    // being invoked before we're ready.
    this->world->AddUpdateCallback((Stg::world_callback_t)s_update, this);

    this->world->ForEachDescendant((Stg::model_callback_t)ghfunc, this);
    if (lasermodels.size() != positionmodels.size())
    {
        ROS_FATAL("number of position models and laser models must be equal in "
                  "the world file.");
        ROS_BREAK();
    }
    size_t numRobots = positionmodels.size();
    ROS_INFO("found %u position/laser pair%s in the file",
             (unsigned int)numRobots, (numRobots==1) ? "" : "s");

    this->fiducialsMsgs = new tug_stage_ros::Fiducials[numRobots];
    this->laserMsgs = new sensor_msgs::LaserScan[numRobots];
    this->odomMsgs = new nav_msgs::Odometry[numRobots];
    this->groundTruthMsgs = new nav_msgs::Odometry[numRobots];
}


// Subscribe to models of interest.  Currently, we find and subscribe
// to the first 'laser' model and the first 'position' model.  Returns
// 0 on success (both models subscribed), -1 otherwise.
//
// Eventually, we should provide a general way to map stage models onto ROS
// topics, similar to Player .cfg files.
int
StageNode::SubscribeModels()
{
    n_.setParam("/use_sim_time", true);

    for (size_t r = 0; r < this->positionmodels.size(); r++)
    {
        if(this->lasermodels[r])
        {
            this->lasermodels[r]->Subscribe();
        }
        else
        {
            ROS_ERROR("no laser");
            return(-1);
        }
        if(this->positionmodels[r])
        {
            this->positionmodels[r]->Subscribe();
        }
        else
        {
            ROS_ERROR("no position");
            return(-1);
        }
        fiducials_pubs_.push_back(n_.advertise<tug_stage_ros::Fiducials>(mapName(FIDUCIALS,r), 10));
        laser_pubs_.push_back(n_.advertise<sensor_msgs::LaserScan>(mapName(BASE_SCAN,r), 10));
        odom_pubs_.push_back(n_.advertise<nav_msgs::Odometry>(mapName(ODOM,r), 10));
        ground_truth_pubs_.push_back(n_.advertise<nav_msgs::Odometry>(mapName(BASE_POSE_GROUND_TRUTH,r), 10));
        cmdvel_subs_.push_back(n_.subscribe<geometry_msgs::Twist>(mapName(CMD_VEL,r), 10, boost::bind(&StageNode::cmdvelReceived, this, r, _1)));
        wheelcmdvel_subs_.push_back(n_.subscribe<tug_stage_ros::WheelCmdVel>(mapName(WHEEL_CMD_VEL,r), 10, boost::bind(&StageNode::wheelcmdvelCB, this, r, _1)));
        ackermanncmd_subs_.push_back(n_.subscribe<tug_stage_ros::AckermannCmd>(mapName(ACKERMANN_CMD,r), 10, boost::bind(&StageNode::ackermanncmdCB, this, r, _1)));
        set_robot_pose_srvs_.push_back(n_.advertiseService<tug_stage_ros::SetRobotPose::Request, tug_stage_ros::SetRobotPose::Response>(mapName(SET_POSE,r), boost::bind(&StageNode::setRobotPoseCB, this, r, _1, _2)));
    }
    clock_pub_ = n_.advertise<rosgraph_msgs::Clock>("/clock",10);
    return(0);
}

StageNode::~StageNode()
{
    delete[] fiducialsMsgs;
    delete[] laserMsgs;
    delete[] odomMsgs;
    delete[] groundTruthMsgs;
}

bool
StageNode::UpdateWorld()
{
    return this->world->UpdateAll();
}

void
StageNode::WorldCallback()
{
    boost::mutex::scoped_lock lock(msg_lock);

    this->sim_time.fromSec(world->SimTimeNow() / 1e6);
    // We're not allowed to publish clock==0, because it used as a special
    // value in parts of ROS, #4027.
    if(this->sim_time.sec == 0 && this->sim_time.nsec == 0)
    {
        ROS_DEBUG("Skipping initial simulation step, to avoid publishing clock==0");
        return;
    }

    // TODO make this only affect one robot if necessary
    if((this->base_watchdog_timeout.toSec() > 0.0) &&
            ((this->sim_time - this->base_last_cmd) >= this->base_watchdog_timeout))
    {
        for (size_t r = 0; r < this->positionmodels.size(); r++)
            this->positionmodels[r]->SetSpeed(0.0, 0.0, 0.0);
    }


    for (size_t r = 0; r < this->fiducialmodels.size(); r++)
    {
        //ROS_INFO("FOV = %f, MAX = %f", this->fiducialmodels[r]->fov, this->fiducialmodels[r]->max_range_anon);
        std::vector<Stg::ModelFiducial::Fiducial > & fiducials =  this->fiducialmodels[r]->GetFiducials ();
        this->fiducialsMsgs[r].observations.resize(fiducials.size());
        for(unsigned int i=0; i<fiducials.size(); i++)
        {
            this->fiducialsMsgs[r].observations[i].id = fiducials[i].id;
            this->fiducialsMsgs[r].observations[i].bearing = fiducials[i].bearing;
            this->fiducialsMsgs[r].observations[i].range = fiducials[i].range;
        }
        if (fiducials.size() > 0)
        {
            this->fiducialsMsgs[r].header.frame_id = mapName("base_laser_link", r);
            this->fiducialsMsgs[r].header.stamp = sim_time;
            this->fiducials_pubs_[r].publish(this->fiducialsMsgs[r]);
        }
    }

    // Get latest laser data
    for (size_t r = 0; r < this->lasermodels.size(); r++)
    {
        const std::vector<Stg::ModelRanger::Sensor>& sensors = this->lasermodels[r]->GetSensors();

        if( sensors.size() > 1 )
            ROS_WARN( "ROS Stage currently supports rangers with 1 sensor only." );

        // for now we access only the zeroth sensor of the ranger - good
        // enough for most laser models that have a single beam origin
        const Stg::ModelRanger::Sensor& s = sensors[0];

        if( s.ranges.size() )
        {
            // Translate into ROS message format and publish
            this->laserMsgs[r].angle_min = -s.fov/2.0;
            this->laserMsgs[r].angle_max = +s.fov/2.0;
            this->laserMsgs[r].angle_increment = s.fov/(double)(s.sample_count-1);
            this->laserMsgs[r].range_min = s.range.min;
            this->laserMsgs[r].range_max = s.range.max;
            this->laserMsgs[r].ranges.resize(s.ranges.size());
            this->laserMsgs[r].intensities.resize(s.intensities.size());

            for(unsigned int i=0; i<s.ranges.size(); i++)
            {
                this->laserMsgs[r].ranges[s.ranges.size()-i-1] = s.ranges[i];
                this->laserMsgs[r].intensities[s.ranges.size()-i-1] = (uint8_t)s.intensities[i];
            }

            this->laserMsgs[r].header.frame_id = mapName("base_laser_link", r);
            this->laserMsgs[r].header.stamp = sim_time;
            this->laser_pubs_[r].publish(this->laserMsgs[r]);
        }

        // Also publish the base->base_laser_link Tx.  This could eventually move
        // into being retrieved from the param server as a static Tx.
        Stg::Pose lp = this->lasermodels[r]->GetPose();
        tf::Quaternion laserQ;
        laserQ.setRPY(M_PI, 0.0, lp.a);
        tf::Transform txLaser =  tf::Transform(laserQ,
                                               tf::Point(lp.x, lp.y, 0.0));
        tf.sendTransform(tf::StampedTransform(txLaser, sim_time,
                                              mapName("mounting_plate", r),
                                              mapName("base_laser_link", r)));

        // Send the identity transform between base_footprint and base_link
        tf::Transform txIdentity(tf::createIdentityQuaternion(),
                                 tf::Point(0, 0, 0.15));
        tf.sendTransform(tf::StampedTransform(txIdentity,
                                              sim_time,
                                              mapName("base_footprint", r),
                                              mapName("mounting_plate", r)));

        // Get latest odometry data
        // Translate into ROS message format and publish
        this->odomMsgs[r].pose.pose.position.x = this->positionmodels[r]->est_pose.x;
        this->odomMsgs[r].pose.pose.position.y = this->positionmodels[r]->est_pose.y;
        this->odomMsgs[r].pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->positionmodels[r]->est_pose.a);
        Stg::Velocity v = this->positionmodels[r]->GetVelocity();
        this->odomMsgs[r].twist.twist.linear.x = v.x;
        this->odomMsgs[r].twist.twist.linear.y = v.y;
        this->odomMsgs[r].twist.twist.angular.z = v.a;

        //@todo Publish stall on a separate topic when one becomes available
        //this->odomMsgs[r].stall = this->positionmodels[r]->Stall();
        //
        this->odomMsgs[r].header.frame_id = mapName("odom", r);
        this->odomMsgs[r].header.stamp = sim_time;

        this->odom_pubs_[r].publish(this->odomMsgs[r]);

        // broadcast odometry transform
        tf::Quaternion odomQ;
        tf::quaternionMsgToTF(odomMsgs[r].pose.pose.orientation, odomQ);
        tf::Transform txOdom(odomQ,
                             tf::Point(odomMsgs[r].pose.pose.position.x,
                                       odomMsgs[r].pose.pose.position.y, 0.0));
        tf.sendTransform(tf::StampedTransform(txOdom, sim_time,
                                              mapName("odom", r),
                                              mapName("base_footprint", r)));

        // Also publish the ground truth pose and velocity
        Stg::Pose gpose = this->positionmodels[r]->GetGlobalPose();
        tf::Quaternion q_gpose;
        q_gpose.setRPY(0.0, 0.0, gpose.a-M_PI/2.0);
        tf::Transform gt(q_gpose, tf::Point(gpose.y, -gpose.x, 0.0));
        // Velocity is 0 by default and will be set only if there is previous pose and time delta>0
        Stg::Velocity gvel(0,0,0,0);
        if (this->base_last_globalpos.size()>r)
        {
            Stg::Pose prevpose = this->base_last_globalpos.at(r);
            double dT = (this->sim_time-this->base_last_globalpos_time).toSec();
            if (dT>0)
                gvel = Stg::Velocity(
                            (gpose.x - prevpose.x)/dT,
                            (gpose.y - prevpose.y)/dT,
                            (gpose.z - prevpose.z)/dT,
                            Stg::normalize(gpose.a - prevpose.a)/dT
                            );
            this->base_last_globalpos.at(r) = gpose;
        }
        else //There are no previous readings, adding current pose...
            this->base_last_globalpos.push_back(gpose);

        this->groundTruthMsgs[r].pose.pose.position.x     = gt.getOrigin().x();
        this->groundTruthMsgs[r].pose.pose.position.y     = gt.getOrigin().y();
        this->groundTruthMsgs[r].pose.pose.position.z     = gt.getOrigin().z();
        this->groundTruthMsgs[r].pose.pose.orientation.x  = gt.getRotation().x();
        this->groundTruthMsgs[r].pose.pose.orientation.y  = gt.getRotation().y();
        this->groundTruthMsgs[r].pose.pose.orientation.z  = gt.getRotation().z();
        this->groundTruthMsgs[r].pose.pose.orientation.w  = gt.getRotation().w();
        this->groundTruthMsgs[r].twist.twist.linear.x = gvel.x;
        this->groundTruthMsgs[r].twist.twist.linear.y = gvel.y;
        this->groundTruthMsgs[r].twist.twist.linear.z = gvel.z;
        this->groundTruthMsgs[r].twist.twist.angular.z = gvel.a;

        this->groundTruthMsgs[r].header.frame_id = mapName("odom", r);
        this->groundTruthMsgs[r].header.stamp = sim_time;

        this->ground_truth_pubs_[r].publish(this->groundTruthMsgs[r]);
    }

    this->clockMsg.clock = sim_time;
    this->clock_pub_.publish(this->clockMsg);
}

int 
main(int argc, char** argv)
{ 
    if( argc < 2 )
    {
        puts(USAGE);
        exit(-1);
    }

    ros::init(argc, argv, "stageros");

    bool gui = true;
    for(int i=0;i<(argc-1);i++)
    {
        if(!strcmp(argv[i], "-g"))
            gui = false;
    }

    StageNode sn(argc-1,argv,gui,argv[argc-1]);

    if(sn.SubscribeModels() != 0)
        exit(-1);

    boost::thread t = boost::thread(boost::bind(&ros::spin));

    // New in Stage 4.1.1: must Start() the world.
    sn.world->Start();

    // TODO: get rid of this fixed-duration sleep, using some Stage builtin
    // PauseUntilNextUpdate() functionality.
    ros::WallRate r(10.0);
    while(ros::ok() && !sn.world->TestQuit())
    {
        if(gui)
            Fl::wait(r.expectedCycleTime().toSec());
        else
        {
            sn.UpdateWorld();
            r.sleep();
        }
    }
    t.join();

    exit(0);
}

