#include <cstdio>
#include <iostream>
#include "mavros_msgs/msg/state.hpp"
#include <rclcpp/rclcpp.hpp>

class StateChanger : public rclcpp::Node
{
  public:
    StateChanger()
    : Node("state_changer")
    {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", qos, std::bind(&StateChanger::stateCallback, this, std::placeholders::_1));
    }

  private:
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received MAVROS state: [Connected: %s, Armed: %s, Mode: %s]",
                    msg->connected ? "true" : "false", msg->armed ? "true" : "false", msg->mode.c_str());
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateChanger>());
  rclcpp::shutdown();
  return 0;
}
