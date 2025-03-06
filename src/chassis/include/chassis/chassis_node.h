#include <iostream>
#include <thread>

#include "chassis/chassis.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "dbus_dr16_interface/msg/dr16.hpp"

class ChassisSubscriber : public rclcpp::Node
{
public:
    ChassisSubscriber();

private:
    rclcpp::Subscription<dbus_dr16_interface::msg::DR16>::SharedPtr subscription_;
    void callback(const dbus_dr16_interface::msg::DR16::SharedPtr msg);
};

Chassis chassis;