#include <cstdio>
#include <iostream>
#include "file_writer.h"
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GlobalPositionSub : public rclcpp::Node
{
  public:
    GlobalPositionSub()
    : Node("global_position_sub"), fw_("/home/parallels/ros2_ws/src/path_planner/tests", ":")
    {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
      
      global_position_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
          "/mavros/global_position/global", qos, 
          std::bind(&GlobalPositionSub::globalPositionCallback, this, std::placeholders::_1)
      );
    }

  private:
    void globalPositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "globalPositionCallback: MESSAGE RECEIVED");
      std::ostringstream ss;
      // increase precision, decimal places were lost without std::fixed and std::setprecision
      ss << std::fixed << std::setprecision(10) << msg->latitude << "," << msg->longitude;
      const std::string lat_long_str = ss.str();
      fw_.writeToFile(lat_long_str);
    }
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_position_sub_;

    FileWriter fw_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPositionSub>());
  rclcpp::shutdown();
  return 0;
}
