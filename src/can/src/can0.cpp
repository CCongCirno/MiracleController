#include "can/can_node.h"
using namespace std;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanPublisher>("CanPublisher", "/can0_send", "/can0_recv", 0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
