#include <iostream>
#include <cstdio>
#include <chrono>
#include <unistd.h>
#include <cstring>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "can_interface/msg/motor_info.hpp"
#include "can_interface/msg/motor_send_info.hpp"

#include "can/can.h"

class CanPublisher : public rclcpp::Node
{
public:
    CanPublisher(std::string name, std::string send_topic_name, std::string recv_topic_name, int id) : Node(name), send_topic_name(send_topic_name), recv_topic_name(recv_topic_name), id(id)
    {
        can.canInit(id);
        subscription_ = this->create_subscription<can_interface::msg::MotorSendInfo>(
            send_topic_name.c_str(), 1, std::bind(&CanPublisher::can_send_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<can_interface::msg::MotorInfo>(recv_topic_name.c_str(), 20);
        timer_ = this->create_wall_timer(std::chrono::microseconds(10), std::bind(&CanPublisher::can_timer_callback, this));
    }
    ~CanPublisher()
    {
        // 关闭socket
        can.canClose();
    }

private:
    rclcpp::Publisher<can_interface::msg::MotorInfo>::SharedPtr publisher_;
    rclcpp::Subscription<can_interface::msg::MotorSendInfo>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    can_interface::msg::MotorInfo message;
    CAN can;
    std::string send_topic_name;
    std::string recv_topic_name;
    int id;
    void can_timer_callback()
    {
        can_interface::msg::MotorInfo msg;
        can.canRead(msg);
        if (can.isUpdate())
        {
            publisher_->publish(msg);
        }
    }
    void can_send_callback(const can_interface::msg::MotorSendInfo::SharedPtr msg)
    {
        can.canSend(msg);
    }
};
