#include "dbus_dr16/dbus_dr16_node.h"
using namespace std;

class DBusDR16Publisher : public rclcpp::Node
{
public:
    DBusDR16Publisher(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s node has started.", name.c_str());
        publisher_ = this->create_publisher<dbus_dr16_interface::msg::DR16>("/remote_control", 1);
        timer_ = this->create_wall_timer(std::chrono::microseconds(1000), std::bind(&DBusDR16Publisher::timer_callback, this));
    }

private:
    rclcpp::Publisher<dbus_dr16_interface::msg::DR16>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    dbus_dr16_interface::msg::DR16 message = dbus_dr16_interface::msg::DR16();
    DBus dbus;
    bool isDbusInit = false;
    const char *port = "/dev/ttyUSB0";
    void timer_callback()
    {
        if (!isDbusInit)
        {
            dbus.init(port);
            isDbusInit = true;
        }
        dbus.DBusRead();
        if (dbus.isUpdate())
        {
            dbus.getDbusInfo(message);
            RCLCPP_INFO(this->get_logger(), "[DR16] ch0:%4d ch1:%4d ch2:%4d ch3:%4d s0:%1d s0:%1d x:%3d y:%3d z:%3d l:%1d r:%1d key:%5d wheel:%4d",
                        (int)message.ch0, (int)message.ch1, (int)message.ch2, (int)message.ch3, (int)message.s0, (int)message.s1, (int)message.x,
                        (int)message.y, (int)message.z, (int)message.l, (int)message.r, (int)message.key, (int)message.wheel);
            publisher_->publish(message);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DBusDR16Publisher>("DBusDR16Publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
