#include <cstdio>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

class Chassis
{
public:
    Chassis() = default;
    ~Chassis() = default;
    struct ControlerInfo
    {
        int16_t ch0;
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
    } controlerInfo;

    union Current
    {
        short I_data;
        u_char raw_data[2];
    } M3508_sendInfo[8];

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
};

void start(Chassis *chassis);
