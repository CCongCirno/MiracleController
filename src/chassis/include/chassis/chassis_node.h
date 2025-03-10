#include <iostream>
#include <thread>

#include "chassis/chassis.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "dbus_dr16_interface/msg/dr16.hpp"
#include "can_interface/msg/motor_info.hpp"
#include "can_interface/msg/motor_send_info.hpp"

class ChassisSubscriber : public rclcpp::Node
{
public:
    ChassisSubscriber() : Node("Chassis_Subscriber")
    {
        controller_subscription_ = this->create_subscription<dbus_dr16_interface::msg::DR16>(
            "/remote_control", 1, std::bind(&ChassisSubscriber::controller_callback, this, std::placeholders::_1));
        motor_subscription_ = this->create_subscription<can_interface::msg::MotorInfo>(
            "/can0_recv", 1, std::bind(&ChassisSubscriber::motor_callback, this, std::placeholders::_1));
        send_publisher_ = this->create_publisher<can_interface::msg::MotorSendInfo>("/can0_send", 20);
        send_timer_ = this->create_wall_timer(std::chrono::microseconds(100), std::bind(&ChassisSubscriber::send_callback, this));
    }

private:
    rclcpp::Subscription<dbus_dr16_interface::msg::DR16>::SharedPtr controller_subscription_;
    rclcpp::Subscription<can_interface::msg::MotorInfo>::SharedPtr motor_subscription_;
    rclcpp::Publisher<can_interface::msg::MotorSendInfo>::SharedPtr send_publisher_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    void controller_callback(const dbus_dr16_interface::msg::DR16::SharedPtr msg);
    void motor_callback(const can_interface::msg::MotorInfo::SharedPtr msg);
    void send_callback();
};

Chassis chassis;