#include <cstdio>
#include <chrono>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <cmath>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <fmt/core.h>
#include <fmt/color.h>
#include <fmt/format.h>

#include "can_interface/msg/motor_info.hpp"
#include "can_interface/msg/motor_send_info.hpp"

class CAN
{
public:
    CAN() = default;
    ~CAN() = default;

    int canInit(int id);
    void canRead(can_interface::msg::MotorInfo &msg);
    void canSend(const can_interface::msg::MotorSendInfo::SharedPtr msg);
    bool isUpdate()
    {
        return updateFlag;
    }
    void canClose()
    {
        close(socket_id);
    }

    union Current
    {
        short current_data;
        u_char raw_data[2];
    } sendInfo[8];

    union Angle
    {
        short angle_data;
        u_char raw_data[2];
    };

    union Speed
    {
        short speed_data;
        u_char raw_data[2];
    };

    union Temperature
    {
        short temp_data;
        u_char raw_data;
    };
    struct M3508_ReceiveInfo
    {
        Angle angle;
        Speed speed;
        Current current;
        Temperature temp;
    } M3508_receiveInfo[8];

private:
    const char *can_port[2] = {"can0", "can1"};
    int id;
    int socket_id;
    bool updateFlag = false;
    std::string idntifier;
    std::string error_idntifier = fmt::format(fg(fmt::color::red) | fmt::emphasis::bold, "ERROR");
};