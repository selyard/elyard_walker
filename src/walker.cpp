/** @file listener.cpp
 * @brief ROS Listener; listens to broadcast from talker.cpp
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <cmath>


void getAllRanges(const sensor_msgs::LaserScan::ConstPtr& scan_input) {
  int numIntervals = ceil( (scan_input->angle_max - scan_input->angle_min) /
    scan_input->angle_increment);
  std::vector<float> ranges = scan_input->ranges;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "walker_script");
  ros::NodeHandle n;

  // Base publisher on the topic that the teleoperation program BUSINESS
  // Base subscriber on a rostopic echo of the listed topics from Gazebo

  // Subscribe to "scan" output from Gazebo simulation to return info
  // regarding nearby obstacles
  ros::Subscriber sub_scan = n.subscribe("/scan", 1000, getAllRanges);
  // Publish a geometry_msgs::Twist variable to "/cmd_vel"
  ros::Publisher pub_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  // Mark if currently turning to avoid an obstacle.
  bool turningToAvoid = false;
  // Mark if able to procede forward.
  bool ableToProcede = true;


  while(ros::ok()) {
    geometry_msgs::Twist cmd;


  }
}
