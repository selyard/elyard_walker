/**
* @file walker.cpp
* @brief Simple C&C for rover; move forward until obstacle, then turn.
* @copyright Copyright 2020, Spencer Elyard [MIT License]
*
*Permission is hereby granted, free of charge, to any person obtaining a
*copy of this software and associated documentation files (the "Software"),
*to deal in the Software without restriction, including without limitation
*the rights to use, copy, modify, merge, publish, distribute, sublicense,
*and/or sell copies of the Software, and to permit persons to whom the
*Software is furnished to do so, subject to the following conditions:
*
*The above copyright notice and this permission notice shall be included
*in all copies or substantial portions of the Software.
*THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
*OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
*THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.
*/

#include <cmath>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
* @brief Class describing the Walker algorithm
*/
class WalkerAlgorithm {
 public:
  // Store range data
  std::vector<float> ranges;
  // Store current command
  geometry_msgs::Twist cmd;

  /**
  *@brief Tell the algorithm to continue straight
  */
  void continueStraight();
  /**
  * @brief Tell the algorithm to avoid an obstacle (turn right)
  */
  void avoidObstacle();
  /**
  * @brief Tell the algorithm hold its position
  */
  void holdPos();
  /**
  * @brief Get the stored command from the WalkerAlgorithm
  * @return gemoetry_msgs::Twist command
  */
  geometry_msgs::Twist getCMD();
  /**
  * @brief Get the list of ranges from the ROS subscription
  * @param scan_input Input from ROS Subscriber
  */
  void getAllRanges(const sensor_msgs::LaserScan::ConstPtr& scan_input);
  /**
  * @brief Check if safe to proceed
  * @param numIntervals Number of intervals of the range to check for obstacles
  * @param min_rage Minimum allowable range before attempting to avoid obstacles
  * @return int (-1) if invalid; (0) if blocked; (1) if safe
  */
  int safeToProceed(int numIntervals, double min_range);
};

void WalkerAlgorithm::continueStraight() {
  cmd.linear.x = 0.2;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = 0;
}

void WalkerAlgorithm::avoidObstacle() {
  cmd.linear.x = 0;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = -0.75;
}

void WalkerAlgorithm::holdPos() {
  cmd.linear.x = 0;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = 0;
}

geometry_msgs::Twist WalkerAlgorithm::getCMD() {
  return this->cmd;
}

void WalkerAlgorithm::getAllRanges(const sensor_msgs::LaserScan::ConstPtr&
  scan_input) {
    this->ranges = scan_input->ranges;
}

int WalkerAlgorithm::safeToProceed(int numIntervals, double min_range) {
  bool isSafe = 1;
  // if ranges does not yet have values
  if (ranges.size() == 0) {
    isSafe = -1;
  } else {
    // if ranges to the right of center are blocked
    for (int i=0; i <= numIntervals; i++) {
      if (ranges.at(i) <= min_range) {
        isSafe = 0;
      }
    }
    // if ranges to the left of center are blocked
    for (int i=(ranges.size()-1); i >= (ranges.size() - numIntervals); i--) {
      if (ranges.at(i) <= min_range) {
        isSafe = 0;
      }
    }
  }
  return isSafe;
}

/**
* @brief Main script
*/
int main(int argc, char **argv) {
  // create object with Walker algorithm
  WalkerAlgorithm walker;
  // initiate ROS
  ros::init(argc, argv, "walker_script");
  // defie node
  ros::NodeHandle n;

  // Base publisher on the topic that the teleoperation program publishes
  // Base subscriber on a rostopic echo of the listed topics from Gazebo

  // Subscribe to "scan" output from Gazebo simulation to return info
  // regarding nearby obstacles
  ros::Subscriber sub_scan = n.subscribe("/scan", 1000,
    &WalkerAlgorithm::getAllRanges, &walker);
  // Publish a geometry_msgs::Twist variable to "/cmd_vel"
  ros::Publisher pub_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  // get frequency parameter from launch script
  int desired_freq;
  const std::string FREQ_SET_NAME = "freq_set";
  // set frequency or deefault+warn if not set
  if (n.getParam(FREQ_SET_NAME, desired_freq)) {
    ROS_INFO("Got frequency: %i", desired_freq);
  } else {
    ROS_WARN_STREAM("No frequency found, default to 10");
    desired_freq = 10;
    }
  ros::Rate loop_rate(desired_freq);

  // get sensor angles to detect obstacles from launch script
  // work on sensor intervals instead of set angle values
  int avoidance_angles;
  const std::string AVOID_ANGLES_SET = "avoid_angles_set";
  // set frequency or deefault+warn if not set
  if (n.getParam(AVOID_ANGLES_SET, avoidance_angles)) {
    ROS_INFO("Got # avoidance intervals plus/minus: %i", avoidance_angles);
  } else {
    ROS_WARN_STREAM("No interval input, default to plus/minus 10 intervals.");
    avoidance_angles = 10;
    }

  // get range to obstacles to avoid from launch script
  double min_avoidance_distance;
  const std::string MIN_AVOID_DISTANCE = "min_avoid_distance";
  // set frequency or deefault+warn if not set
  if (n.getParam(MIN_AVOID_DISTANCE, min_avoidance_distance)) {
    ROS_INFO("Got min distance of: %f", min_avoidance_distance);
  } else {
    ROS_WARN_STREAM("No distance input, default to 1");
    min_avoidance_distance = 1.;
  }

  // loop while ROS is OK
  while (ros::ok()) {
    // check if path forward is clear
    int ableToProceed = walker.safeToProceed(avoidance_angles,
      min_avoidance_distance);

    // if path forward clear, then proceed forward
    if (ableToProceed == 1) {
      walker.continueStraight();
    } else if (ableToProceed == 0) {
    // otherwise, turn right until clear
      walker.avoidObstacle();
    } else {
    // if invalid, wait until invalid
      walker.holdPos();
    }
    // publish command
    pub_cmd.publish(walker.getCMD());
    ros::spinOnce();
    // sleep until next loop
    loop_rate.sleep();
  }
}
