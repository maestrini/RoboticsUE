#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <eigen3/Eigen/Eigen>
#include <tf/transform_broadcaster.h>

void publishTransformation(Eigen::Matrix4d tm)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(tm(0,3), tm(1,3), tm(1,3)) );
  
  double w= sqrt(1.0 + tm(0,0) + tm(1,1) + tm(2,2)) /2.0;
  double x = (tm(2,1) - tm(1,2))/( 4.0 *w);
  double y = (tm(0,2) - tm(2,0))/( 4.0 *w);
  double z = (tm(1,0) - tm(0,1))/( 4.0 *w);

  tf::Quaternion q(x,y,z,w);

  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "converter_frame"));
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("I heard an odom message");

  // add your manual transformation calculation here
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("I heard an laser message");

  // add your conversion of the sensor messages here. Don't forget to publish the new message.
}
	
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber odom_sub = n.subscribe("odom", 1000, odometryCallback);
  ros::Subscriber laser_sub = n.subscribe("base_scan", 1000, laserCallback);

  ros::spin();

  return 0;
}
