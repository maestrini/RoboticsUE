#include <ros/ros.h>
#include <tug_ros_examples/AddTwoNumbers.h>

bool add(tug_ros_examples::AddTwoNumbers::Request  &req,
         tug_ros_examples::AddTwoNumbers::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_numbers_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_numbers", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
