#pragma once
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

private:
};
void updateCh0(int16_t ch0);

void start(Chassis *chassis);
