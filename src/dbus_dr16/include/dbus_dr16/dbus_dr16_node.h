#include <iostream>

#include "dbus_dr16/dbus_dr16.h"
#include "dbus_dr16_interface/msg/dr16.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class DBus_DR16_Publisher : public rclcpp::Node
{
public:
    DBus_DR16_Publisher();

private:
    rclcpp::Publisher<dbus_dr16_interface::msg::DR16>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    const char *port = "/dev/ttyUSB0";
    DBus dbus;
};