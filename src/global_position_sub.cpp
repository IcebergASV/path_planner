#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"


class GlobalPositionSub : public rclcpp::Node
{
  public:
    GlobalPositionSub()
    : Node("global_position_sub")
    {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
      
      global_position_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
          "/mavros/global_position/global", qos, 
          std::bind(&GlobalPositionSub::globalPositionCallback, this, std::placeholders::_1)
      );
    }

  private:
    void globalPositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Latitude: %f, Longitude: %f", msg->latitude, msg->longitude);
    }
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_position_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPositionSub>());
  rclcpp::shutdown();
  return 0;
}
