/** @file listener.cpp
 * @brief ROS Listener; listens to broadcast from talker.cpp
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <iostream>

// store number of angle intervals
std::vector<float> ranges;

void getAllRanges(const sensor_msgs::LaserScan::ConstPtr& scan_input) {
  ranges = scan_input->ranges;
}

int safeToProcede(double numIntervals, double min_range) {
  bool isSafe = 1;
  // if ranges does not yet have values
  if (ranges.size() == 0) {
    isSafe = -1;
  } else {
  // if ranges to the right of center are blocked
  for(int i=0; i <= numIntervals; i++) {
    if (ranges.at(i) <= min_range) {
      isSafe = 0;
    }
  }
  // if ranges to the left of center are blocked
  for(int i=(ranges.size()-1); i >= (ranges.size() - numIntervals); i--) {
    if (ranges.at(i) <= min_range) {
      isSafe = 0;
    }
  }
  }
  return isSafe;
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

  // get frequency parameter
  int desired_freq;
  const std::string FREQ_SET_NAME = "freq_set";
  // set frequency or deefault+warn if not set
  if (n.getParam(FREQ_SET_NAME, desired_freq)) {
    ROS_INFO("Got frequency: %i", desired_freq);
  }  else {
    ROS_WARN_STREAM("No frequency found, default to 10");
    desired_freq = 10;
    }
  // %Tag(LOOP_RATE)%
  ros::Rate loop_rate(desired_freq);

    // get sensor angles detect obstacles
    // work on sensor intervals instead of set angle values
    double avoidance_angles;
    const std::string AVOID_ANGLES_SET = "avoid_angles_set";
    // set frequency or deefault+warn if not set
    if (n.getParam(AVOID_ANGLES_SET, avoidance_angles)) {
      ROS_INFO("Got # avoidance intervals plus/minus: %f", avoidance_angles);
    }  else {
      ROS_WARN_STREAM("No interval input, default to plus/minus 10 intervals.");
      avoidance_angles = 10.;
    }

    // get range to obstacles to avoid
    double min_avoidance_distance;
    const std::string MIN_AVOID_DISTANCE = "min_avoid_distance";
    // set frequency or deefault+warn if not set
    if (n.getParam(MIN_AVOID_DISTANCE, min_avoidance_distance)) {
      ROS_INFO("Got min distance of: %f", min_avoidance_distance);
    }  else {
      ROS_WARN_STREAM("No distance input, default to 1");
      min_avoidance_distance = 1.;
    }

  // Mark if able to procede forward.
  bool ableToProcede = true;


  while(ros::ok()) {
    geometry_msgs::Twist cmd;

    // check if path forward is clear
    int ableToProcede = safeToProcede(avoidance_angles, min_avoidance_distance);
    // if path forward clear, then procede forward
    if (ableToProcede == 1) {
      cmd.linear.x = 0.1;
      cmd.linear.y = 0;
      cmd.linear.z = 0;
      cmd.angular.x = 0;
      cmd.angular.y = 0;
      cmd.angular.z = 0;
    } else if (ableToProcede == 0) {
    // otherwise, turn right until clear
      cmd.linear.x = 0;
      cmd.linear.y = 0;
      cmd.linear.z = 0;
      cmd.angular.x = 0;
      cmd.angular.y = 0;
      cmd.angular.z = -0.5;
    } else {
    // if invalid, wait until invalid
    cmd.linear.x = 0;
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = 0;
    }

    pub_cmd.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();

  }
}
