#include "chassis/chassis_node.h"
using namespace std;
ChassisSubscriber::ChassisSubscriber() : Node("Chassis_Subscriber")
{
    subscription_ = this->create_subscription<dbus_dr16_interface::msg::DR16>(
        "/remote_control", 1, std::bind(&ChassisSubscriber::callback, this, std::placeholders::_1));
}

void ChassisSubscriber::callback(const dbus_dr16_interface::msg::DR16::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received: ");
    chassis.controlerInfo.ch0 = msg->ch0;
    chassis.controlerInfo.ch1 = msg->ch1;
    chassis.controlerInfo.ch2 = msg->ch2;
    chassis.controlerInfo.ch3 = msg->ch3;
    // cout << chassis.controlerInfo.ch0 << endl;
    /* cout << "ch0:" << (int)msg->ch0 << " ";
    cout << "ch1:" << (int)msg->ch1 << " ";
    cout << "ch2:" << (int)msg->ch2 << " ";
    cout << "ch3:" << (int)msg->ch3 << " ";
    cout << "s0:" << (int)msg->s0 << " ";
    cout << "s1:" << (int)msg->s1 << " ";
    cout << "x:" << (int)msg->x << " ";
    cout << "y:" << (int)msg->y << " ";
    cout << "z:" << (int)msg->z << " ";
    cout << "l:" << (int)msg->l << " ";
    cout << "r:" << (int)msg->r << " ";
    cout << "key:" << (int)msg->key << " ";
    cout << "wheel:" << (int)msg->wheel << "\n";
    */
};

int main(int argc, char *argv[])
{

    std::thread chassis_thread(start, &chassis);

    chassis_thread.detach();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisSubscriber>());
    rclcpp::shutdown();

    return 0;
}
