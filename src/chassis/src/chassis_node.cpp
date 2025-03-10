#include "chassis/chassis_node.h"

using namespace std;

void ChassisSubscriber::controller_callback(const dbus_dr16_interface::msg::DR16::SharedPtr msg)
{
    chassis.controlerInfo.ch0 = msg->ch0;
    chassis.controlerInfo.ch1 = msg->ch1;
    chassis.controlerInfo.ch2 = msg->ch2;
    chassis.controlerInfo.ch3 = msg->ch3;
}

void ChassisSubscriber::motor_callback(const can_interface::msg::MotorInfo::SharedPtr msg)
{
    if (msg->type == 1)
    {
        chassis.receiveInfo[msg->id].speed = msg->speed;
        chassis.receiveInfo[msg->id].angle = msg->angle;
        chassis.receiveInfo[msg->id].current = msg->current;
        chassis.receiveInfo[msg->id].temp = msg->temp;
        chassis.calculate();
    }
}

void ChassisSubscriber::send_callback()
{
    can_interface::msg::MotorSendInfo msg;
    if (chassis.isUpdate())
    {
        chassis.canSend(msg);
        send_publisher_->publish(msg);
    }
}

int main(int argc, char *argv[])
{
    chassis.init();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisSubscriber>());
    rclcpp::shutdown();

    return 0;
}
