#include <cstdio>
#include <iostream>
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/waypoint_reached.hpp"
#include "mavros_msgs/msg/waypoint_list.hpp"
#include <rclcpp/rclcpp.hpp>

class StateChanger : public rclcpp::Node
{
  public:
    StateChanger()
    : Node("state_changer")
    {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", qos, std::bind(&StateChanger::stateCallback, this, std::placeholders::_1));
        mode_change_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        // waypoint_sub_ = this->create_subscription<mavros_msgs::msg::WaypointReached>(
        //     "/mavros/mission/reached", qos, 
        //     std::bind(&StateChanger::waypointCallback, this, std::placeholders::_1));
        // mission_sub_ = this->create_subscription<mavros_msgs::msg::WaypointList>(
        //     "/mavros/mission/waypoints", qos, 
        //     std::bind(&StateChanger::missionCallback, this, std::placeholders::_1));
        // // Wait for the service to be available
        while (!mode_change_client_->wait_for_service(std::chrono::seconds(3))) 
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for /mavros/set_mode service...");
        }
        changeModeToGuidedRequest();
    }
    void changeModeToGuidedRequest() 
    {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "GUIDED";
        auto future = mode_change_client_->async_send_request(request, 
            [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture response) {
                try {
                    if (response.get()->mode_sent) {
                        RCLCPP_INFO(this->get_logger(), "Successfully set mode to GUIDED");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Failed to set mode to GUIDED");
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            });
    }
  private:
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received MAVROS state: [Connected: %s, Armed: %s, Mode: %s]",
                    msg->connected ? "true" : "false", msg->armed ? "true" : "false", msg->mode.c_str());
    }
    // void waypointCallback(const mavros_msgs::msg::WaypointReached::SharedPtr msg) {
    //     if (current_state_.mode == "AUTO.MISSION" && msg->wp_seq == last_waypoint_) {
    //         RCLCPP_INFO(this->get_logger(), "Final waypoint reached, switching to GUIDED mode");
    //         changeModeToGuided();
    //     }
    // }

    // void missionCallback(const mavros_msgs::msg::WaypointList::SharedPtr msg) {
    //     if (!msg->waypoints.empty()) {
    //         last_waypoint_ = msg->waypoints.size() - 1;
    //         RCLCPP_INFO(this->get_logger(), "Mission loaded. Last waypoint index: %d", last_waypoint_);
    //     }
    // }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_change_client_;
    rclcpp::Subscription<mavros_msgs::msg::WaypointReached>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr mission_sub_;
    int last_waypoint_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateChanger>());
//   node->changeModeToGuidedRequest();
//   rclcpp::spin(node)
  rclcpp::shutdown();;
  return 0;
}
