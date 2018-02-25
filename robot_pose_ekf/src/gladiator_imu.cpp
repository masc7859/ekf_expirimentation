#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sstream>
#include <math.h>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sys/stat.h>

struct gyro_point{
  float x;
  float y;
  float z;
};

struct timestamp{
  int _sec;
  int _nsec;
};

int main(int argc, char **argv){

  ros::init(argc, argv, "ekf_hw");

  //read in gyro data from file
  std::vector<gyro_point> gyro_data;
  std::fstream gyro_file("/home/maxrebo/Documents/robotics_class/imu/gyro.txt", std::ios_base::in);
  gyro_point gyro_point_in;
  float x_in,y_in,z_in;
  while ((gyro_file >> x_in) && gyro_file.ignore() && (gyro_file >> y_in) && gyro_file.ignore() && (gyro_file >> z_in)){
      gyro_point_in.x = x_in;
      gyro_point_in.y = y_in;
      gyro_point_in.z = z_in;
      gyro_data.push_back(gyro_point_in);
  }
  //read in timestamps from file
  std::vector<timestamp> timestamp_data;
  std::fstream time_file("/home/maxrebo/Documents/robotics_class/imu/timestamp.txt", std::ios_base::in);
  timestamp timestamp_in;
  float _sec_in, _nsec_in;
  while ((time_file >> _sec_in) && time_file.ignore() && (time_file >> _nsec_in)){
      timestamp_in._sec = static_cast<int>(_sec_in * 1000);
      timestamp_in._nsec = static_cast<int>(_nsec_in * 1000);
      timestamp_data.push_back(timestamp_in);
  }

  ros::NodeHandle n;
  ros::Publisher imu_odom_pub = n.advertise<nav_msgs::Odometry>("pr2_base_odometry/odom",50);
  ros::Publisher imu_imu_pub = n.advertise<sensor_msgs::Imu>("imu_data",50);

  float roll_prev,pitch_prev,yaw_prev;
  float roll_cur,pitch_cur,yaw_cur;
  float prev_time = timestamp_data.front()._sec;
  float cur_time = timestamp_data.front()._sec;
  float time_interval;

  ros::Rate r(1.0);
  while(n.ok()){
    ros::spinOnce();
    cur_time = timestamp_data.front()._sec;
    time_interval = cur_time - prev_time;
    roll_cur = roll_prev + (time_interval * gyro_data.front().x) +
              (sin(roll_prev) * tan(pitch_prev) * (time_interval * gyro_data.front().y)) +
              (cos(roll_prev) * tan(pitch_prev) * (time_interval * gyro_data.front().z));
    pitch_cur = pitch_prev + (cos(roll_prev) * (time_interval * gyro_data.front().y)) -
              (sin(roll_prev) * (time_interval * gyro_data.front().z));
    yaw_cur = yaw_prev + ((sin(roll_prev)/cos(pitch_prev)) * (time_interval * gyro_data.front().y)) +
              ((cos(roll_prev)/cos(pitch_prev)) * (time_interval * gyro_data.front().z));

    ROS_INFO("%s: %d %d", "Published Both at", static_cast<int>(cur_time), static_cast<int>(prev_time));

    //Send odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp.sec = cur_time;
    odom.header.frame_id = "odom_combined";
    odom.child_frame_id = "base_footprint";

    //geometry_msgs::Quaternion odom_quat;
    //odom_quat.x = 0.0;
    //odom_quat.y = 0.0;
    //odom_quat.z = 0.0;
    //odom_quat.w = 1.0;
    tf::Quaternion oq = tf::createQuaternionFromRPY(roll_cur, pitch_cur, yaw_cur);
    //tf::Quaternion q;
    //q.setRPY(gyro_data.front().x, gyro_data.front().y, gyro_data.front().z);
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(oq, odom_quat);
    odom.pose.pose.orientation = odom_quat;

    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom.pose.covariance[0] = 1.0;
    odom.pose.covariance[7] = 1.0;
    odom.pose.covariance[14] = 1.0;
    odom.pose.covariance[21] = 1.0;
    odom.pose.covariance[28] = 1.0;
    odom.pose.covariance[35] = 1.0;

    odom.twist.covariance[0] = 1.0;
    odom.twist.covariance[7] = 1.0;
    odom.twist.covariance[14] = 1.0;
    odom.twist.covariance[21] = 1.0;
    odom.twist.covariance[28] = 1.0;
    odom.twist.covariance[35] = 1.0;

    //Send imu sensor message
    sensor_msgs::Imu imu;
    imu.header.stamp.sec = cur_time;
    imu.header.frame_id = "odom_combined";
    //tf::Quaternion q = tf::createQuaternionFromRPY(roll_cur, pitch_cur, yaw_cur);
    //q.setRPY(gyro_data.front().x, gyro_data.front().y, gyro_data.front().z);
    //tf::quaternionTFToMsg(q, imu_quat);
    geometry_msgs::Quaternion imu_quat;
    //imu_quat.x = gyro_data.front().x;
    imu_quat.x = 0;
    imu_quat.y = 0;
    imu_quat.z = 0;
    imu_quat.w = 1.0;
    imu.orientation = imu_quat;

    imu.angular_velocity.x = gyro_data.front().x;
    imu.angular_velocity.y = gyro_data.front().y;
    imu.angular_velocity.z = gyro_data.front().z;
    imu.linear_acceleration.x = 0.0;
    imu.linear_acceleration.y = 0.0;
    imu.linear_acceleration.z = 0.0;

    imu.linear_acceleration_covariance[0] = 1.0;
    imu.linear_acceleration_covariance[4] = 1.0;
    imu.linear_acceleration_covariance[8] = 1.0;

    imu.angular_velocity_covariance[0] = 1.0;
    imu.angular_velocity_covariance[4] = 1.0;
    imu.angular_velocity_covariance[8] = 1.0;

    imu.orientation_covariance[0] = 1.0;
    imu.orientation_covariance[4] = 1.0;
    imu.orientation_covariance[8] = 1.0;

    //publish both
    imu_odom_pub.publish(odom);
    imu_imu_pub.publish(imu);

    prev_time = timestamp_data.front()._sec;

    gyro_data.erase(gyro_data.begin());
    timestamp_data.erase(timestamp_data.begin());

    r.sleep();
  }
}
