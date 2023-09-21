#pragma once

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "std_srvs/Trigger.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class RobotGUI {
private:
  ros::NodeHandle nh_;

  ros::Subscriber robot_info_sub_;
  ros::Subscriber odom_sub_;

  ros::Publisher twist_pub_;

  geometry_msgs::Twist twist_msg;

  cv::Mat frame_;

  std::string robot_info_;

  ros::ServiceClient distance_service_client_;
  std::string distance_service_response_;

  double linear_velocity_;
  double angular_velocity_;
  double position_x_;
  double position_y_;
  double position_z_;

  void
  robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

public:
  RobotGUI();
  ~RobotGUI();

  void run();
};
