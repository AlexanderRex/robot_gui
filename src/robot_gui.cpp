#include "robot_gui/robot_gui.h"

RobotGUI::RobotGUI() : nh_(), frame_(800, 280, CV_8UC3), twist_msg() {
  robot_info_sub_ =
      nh_.subscribe("robot_info", 10, &RobotGUI::robotInfoCallback, this);
  odom_sub_ = nh_.subscribe("odom", 10, &RobotGUI::odomCallback, this);
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  distance_service_client_ =
      nh_.serviceClient<std_srvs::Trigger>("get_distance");
  cvui::init("Robot GUI");
}

RobotGUI::~RobotGUI() {}

void RobotGUI::robotInfoCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  robot_info_ = msg->data_field_01 + "\n" + msg->data_field_02 + "\n" +
                msg->data_field_03 + "\n" + msg->data_field_04 + "\n" +
                msg->data_field_05 + "\n" + msg->data_field_06 + "\n" +
                msg->data_field_07 + "\n" + msg->data_field_08 + "\n" +
                msg->data_field_09 + "\n" + msg->data_field_10;
}

void RobotGUI::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  linear_velocity_ = msg->twist.twist.linear.x;
  angular_velocity_ = msg->twist.twist.angular.z;
  position_x_ = msg->pose.pose.position.x;
  position_y_ = msg->pose.pose.position.y;
  position_z_ = msg->pose.pose.position.z;
}

void RobotGUI::run() {
  cv::namedWindow("Robot GUI", cv::WINDOW_AUTOSIZE);

  while (ros::ok()) {
    frame_ = cv::Scalar(49, 52, 49);

    // Info window
    cvui::window(frame_, 10, 10, 260, 230, "InfoArea");

    std::istringstream stream(robot_info_);
    std::string line;
    int yOffset = 35;
    while (std::getline(stream, line)) {
      cvui::text(frame_, 20, yOffset, line);
      yOffset += 20;
    }

    // Teleop buttons
    int buttonsY = 250;
    if (cvui::button(frame_, 90, buttonsY, " Forward ")) {
      twist_msg.linear.x += 0.1;
    }
    if (cvui::button(frame_, 90, buttonsY + 30, "   Stop   ")) {
      twist_msg.linear.x = 0;
      twist_msg.angular.z = 0;
    }
    if (cvui::button(frame_, 20, buttonsY + 30, " Left ")) {
      twist_msg.angular.z += 0.1;
    }
    if (cvui::button(frame_, 190, buttonsY + 30, " Right ")) {
      twist_msg.angular.z -= 0.1;
    }
    if (cvui::button(frame_, 90, buttonsY + 60, " Backward ")) {
      twist_msg.linear.x -= 0.1;
    }

    // Velocity windows
    cvui::window(frame_, 10, 360, 105, 100, "Linear Velocity");
    cvui::printf(frame_, 20, 390, " %.2f m/s", linear_velocity_);

    cvui::window(frame_, 150, 360, 115, 100, "Angular Velocity");
    cvui::printf(frame_, 160, 390, " %.2f rad/s", angular_velocity_);

    // Position windows
    cvui::text(frame_, 10, 475, "Estimated position based on odometry");
    cvui::window(frame_, 10, 495, 80, 80, "X");
    cvui::printf(frame_, 20, 520, "%.2f", position_x_);

    cvui::window(frame_, 100, 495, 80, 80, "Y");
    cvui::printf(frame_, 110, 520, "%.2f", position_y_);

    cvui::window(frame_, 190, 495, 80, 80, "Z");
    cvui::printf(frame_, 200, 520, "%.2f", position_z_);

    cvui::text(frame_, 10, 585, "Distance travelled");
    if (cvui::button(frame_, 10, 610, "CALL")) {
      std_srvs::Trigger srv;
      if (distance_service_client_.call(srv)) {
        distance_service_response_ = srv.response.message;
      } else {
        distance_service_response_ = "Service call failed.";
      }
    }

    cvui::window(frame_, 90, 610, 180, 40, "Distance in meters");
    cvui::printf(frame_, 100, 635, "%s", distance_service_response_.c_str());

    // Publish the twist message
    twist_pub_.publish(twist_msg);

    // Update cvui and display the frame
    cvui::update();
    cv::imshow("Robot GUI", frame_);

    // Check for ESC key
    if (cv::waitKey(20) == 27) {
      break;
    }

    ros::spinOnce();
  }
}
