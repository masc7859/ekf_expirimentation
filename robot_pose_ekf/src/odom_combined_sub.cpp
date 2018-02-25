#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sstream>
#include <math.h>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sys/stat.h>

//global BAD but temporary workaround
std::ofstream outfile;

void chatterCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  outfile << msg->header.stamp.sec << ", " << msg->pose.pose.orientation.x << ", "
          << msg->pose.pose.orientation.y << ", " << msg->pose.pose.orientation.z << "\n";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_combined_listener");

  outfile.open("/tmp/RPY.txt", std::ios_base::app);

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("robot_pose_ekf/odom_combined", 1000, chatterCallback);
  ros::spin();

  return 0;
}
