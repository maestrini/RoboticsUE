#include <ros/ros.h>
#include <tug_ros_examples/AddTwoNumbers.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_numbers_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<tug_ros_examples::AddTwoNumbers>("add_two_numbers");
  tug_ros_examples::AddTwoNumbers srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_numbers");
    return 1;
  }

  return 0;
}
