#include "dbus_dr16/dbus_dr16_node.h"
using namespace std;
DBus_DR16_Publisher::DBus_DR16_Publisher() : Node("DBus_DR16_Publisher")
{
    // 创建一个发布者，发布类型为 std_msgs::msg::String，发布到 /remote_control

    publisher_ = this->create_publisher<dbus_dr16_interface::msg::DR16>("/remote_control", 1);
    dbus.init(port);
    while (true)
    {
        dbus_dr16_interface::msg::DR16 message = dbus_dr16_interface::msg::DR16();
        dbus.DBusRead();
        if (dbus.isUpdate())
        {
            message.ch0 = dbus.getCh0();
            message.ch1 = dbus.getCh1();
            message.ch2 = dbus.getCh2();
            message.ch3 = dbus.getCh3();
            message.s0 = dbus.getS0();
            message.s1 = dbus.getS1();
            message.x = dbus.getX();
            message.y = dbus.getY();
            message.z = dbus.getZ();
            message.l = dbus.getL();
            message.r = dbus.getR();
            message.wheel = dbus.getWheel();
            cout << "ch0:" << (int)message.ch0 << " ";
            cout << "ch1:" << (int)message.ch1 << " ";
            cout << "ch2:" << (int)message.ch2 << " ";
            cout << "ch3:" << (int)message.ch3 << " ";
            cout << "s0:" << (int)message.s0 << " ";
            cout << "s1:" << (int)message.s1 << " ";
            cout << "x:" << (int)message.x << " ";
            cout << "y:" << (int)message.y << " ";
            cout << "z:" << (int)message.z << " ";
            cout << "l:" << (int)message.l << " ";
            cout << "r:" << (int)message.r << " ";
            cout << "key:" << (int)message.key << " ";
            cout << "wheel:" << (int)message.wheel << "\n";
            // message.data = "Hello, ROS 2!";

            publisher_->publish(message);
        }
        RCLCPP_INFO(this->get_logger(), "Publishing");
        cout << "1\n";
    }
    /*while (true)
    {
        dbus.DBusRead();
        if (dbus.isSuccess())
        {
            msg_.ch0 = dbus.getCh0();
            msg_.ch1 = dbus.getCh1();
            publisher_->publish(msg_);
            RCLCPP_INFO(this->get_logger(), "Publishing!");
        }
    }*/
    // 创建一个定时器，每秒发布一次消息
    /*timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]()
        {
            auto message = std_msgs::msg::String();
            message.data = "Hello, ROS 2!";
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
        });*/
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("DBus_DR16_Publisher");
    //  rclcpp::spin(std::make_shared<DBus_DR16_Publisher>());
    rclcpp::Publisher<dbus_dr16_interface::msg::DR16>::SharedPtr publisher_ = node->create_publisher<dbus_dr16_interface::msg::DR16>("/remote_control", 1);
    DBus dbus;
    const char *port = "/dev/ttyUSB0";
    dbus.init(port);
    // dbus.DBusRead();
    rclcpp::WallRate loop_rate(1000); // 创建时钟，1Hz
    while (rclcpp::ok())
    {
        dbus_dr16_interface::msg::DR16 message = dbus_dr16_interface::msg::DR16();
        dbus.DBusRead();
        if (dbus.isUpdate())
        {
            message.ch0 = dbus.getCh0();
            message.ch1 = dbus.getCh1();
            message.ch2 = dbus.getCh2();
            message.ch3 = dbus.getCh3();
            message.s0 = dbus.getS0();
            message.s1 = dbus.getS1();
            message.x = dbus.getX();
            message.y = dbus.getY();
            message.z = dbus.getZ();
            message.l = dbus.getL();
            message.r = dbus.getR();
            message.wheel = dbus.getWheel();
            cout << "ch0:" << (int)message.ch0 << " ";
            cout << "ch1:" << (int)message.ch1 << " ";
            cout << "ch2:" << (int)message.ch2 << " ";
            cout << "ch3:" << (int)message.ch3 << " ";
            cout << "s0:" << (int)message.s0 << " ";
            cout << "s1:" << (int)message.s1 << " ";
            cout << "x:" << (int)message.x << " ";
            cout << "y:" << (int)message.y << " ";
            cout << "z:" << (int)message.z << " ";
            cout << "l:" << (int)message.l << " ";
            cout << "r:" << (int)message.r << " ";
            cout << "key:" << (int)message.key << " ";
            cout << "wheel:" << (int)message.wheel << "\n";

            publisher_->publish(message);
            loop_rate.sleep();
        }
    }
    return 0;
}
