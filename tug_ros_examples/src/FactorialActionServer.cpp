#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tug_ros_examples/FactorialAction.h>

class FactorialAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<tug_ros_examples::FactorialAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  tug_ros_examples::FactorialFeedback feedback_;
  tug_ros_examples::FactorialResult result_;

public:

  FactorialAction(std::string name) :
    as_(nh_, name, boost::bind(&FactorialAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~FactorialAction(void)
  {
  }

  void executeCB(const tug_ros_examples::FactorialGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.iteration = 0;

    // publish info to the console for the user
    ROS_INFO("%s: Executing, calculation factorial for %i ", action_name_.c_str(), goal->number);

    result_.factorial = 1;

    // start executing the action
    for(int i=1; i<=goal->number; i++)
    {
      ROS_INFO("%s: iteration %i",action_name_.c_str(),i);
      result_.factorial *= i;
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.iteration = i;
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "factorial");

  FactorialAction factorial(ros::this_node::getName());
  ros::spin();

  return 0;
}
