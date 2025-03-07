#include <cstdio>
#include <iostream>
#include <csv_writer.h>
#include <lat_long.h>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "mavros_msgs/msg/state.hpp"

class GlobalPositionSub : public rclcpp::Node
{
public:
    GlobalPositionSub()
        : Node("global_position_sub"), csv_w_("/home/jordyn/Documents/Iceberg/"), current_mode_("UNKNOWN")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        global_position_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global", qos, 
            std::bind(&GlobalPositionSub::globalPositionCallback, this, std::placeholders::_1)
        );

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&GlobalPositionSub::stateCallback, this, std::placeholders::_1)
        );
    }

private:
    void globalPositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "globalPositionCallback: MESSAGE RECEIVED");

        const LatLong lat_long(msg->latitude, msg->longitude);

        // Writing to CSV with mode included
        csv_w_.writeToFile(lat_long, current_mode_);
    }

    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_mode_ = msg->mode; // Update the current flight mode
        RCLCPP_INFO(this->get_logger(), "Flight Mode Updated: %s", current_mode_.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_position_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;

    CSVWriter csv_w_;
    std::string current_mode_;  // Stores the latest flight mode
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPositionSub>());
    rclcpp::shutdown();
    return 0;
}
