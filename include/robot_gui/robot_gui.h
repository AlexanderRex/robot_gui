#pragma once

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class RobotGUI {
private:
  ros::NodeHandle nh_;

  ros::Subscriber robot_info_sub_;
  ros::Subscriber odom_sub_; // Подписка на сообщения одометрии

  ros::Publisher twist_pub_;

  geometry_msgs::Twist twist_msg;

  cv::Mat frame_;

  std::string robot_info_;

  // Величины для одометрии
  double linear_velocity_;
  double angular_velocity_;
  double position_x_;
  double position_y_;
  double position_z_;

  void
  robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr
                        &msg); // Функция обратного вызова для одометрии

public:
  RobotGUI();
  ~RobotGUI();

  void run();
};
