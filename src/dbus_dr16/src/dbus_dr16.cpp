#include "dbus_dr16/dbus_dr16.h"

#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#define termios asmtermios
#include <asm/termios.h>
#undef termios
using namespace std;
extern "C"
{
    extern int ioctl(int __fd, unsigned long int __request, ...) throw();
}
void DBus::init(const char *serial)
{
    int fd = open(serial, O_RDWR | O_NOCTTY | O_SYNC);

    struct termios2 options{};
    ioctl(fd, TCGETS2, &options);

    options.c_cflag &= ~CBAUD;
    options.c_cflag |= BOTHER;

    options.c_cflag |= PARENB;
    options.c_cflag &= ~PARODD;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_ispeed = 100000;
    options.c_ospeed = 100000;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~IGNBRK; // disable break processing

    /* set input mode (non−canonical, no echo,...) */
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;

    options.c_oflag = 0;                 // no remapping, no delays
    options.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
    ioctl(fd, TCSETS2, &options);

    port_ = fd;
}

void DBus::DBusRead()
{
    uint8_t read_byte;
    int timeout = 0; // time out of one package
    int count = 0;   // count of bit of one package
    while (timeout < 10)
    {
        // Read a byte //
        size_t n = read(port_, &read_byte, sizeof(read_byte));
        if (n == 0)
        {
            timeout++;
        }
        else if (n == 1)
        {
            // Shift the buffer //
            for (int i = 0; i < 17; i++)
            {
                buff_[i] = buff_[i + 1];
            }
            buff_[17] = read_byte;
            count++;
            timeout = 0;
        }
    }
    unpack();
    if (count < 17)
    {
        memset(&d_bus_data_, 0, sizeof(d_bus_data_));
        is_update_ = false;
    }
    else
    {
        is_update_ = true;
    }
}

void DBus::unpack()
{
    d_bus_data_.ch0 = (buff_[0] | buff_[1] << 8) & 0x07FF;
    d_bus_data_.ch0 -= 1024;
    d_bus_data_.ch1 = (buff_[1] >> 3 | buff_[2] << 5) & 0x07FF;
    d_bus_data_.ch1 -= 1024;
    d_bus_data_.ch2 = (buff_[2] >> 6 | buff_[3] << 2 | buff_[4] << 10) & 0x07FF;
    d_bus_data_.ch2 -= 1024;
    d_bus_data_.ch3 = (buff_[4] >> 1 | buff_[5] << 7) & 0x07FF;
    d_bus_data_.ch3 -= 1024;

    if (d_bus_data_.ch0 <= 10 && d_bus_data_.ch0 >= -10)
        d_bus_data_.ch0 = 0;
    if (d_bus_data_.ch1 <= 10 && d_bus_data_.ch1 >= -10)
        d_bus_data_.ch1 = 0;
    if (d_bus_data_.ch2 <= 10 && d_bus_data_.ch2 >= -10)
        d_bus_data_.ch2 = 0;
    if (d_bus_data_.ch3 <= 10 && d_bus_data_.ch3 >= -10)
        d_bus_data_.ch3 = 0;

    d_bus_data_.s0 = ((buff_[5] >> 4) & 0x0003);
    d_bus_data_.s1 = ((buff_[5] >> 4) & 0x000C) >> 2;

    if ((abs(d_bus_data_.ch0) > 660) || (abs(d_bus_data_.ch1) > 660) || (abs(d_bus_data_.ch2) > 660) ||
        (abs(d_bus_data_.ch3) > 660))
    {
        is_success = false;
        return;
    }
    d_bus_data_.x = buff_[6] | (buff_[7] << 8);
    d_bus_data_.y = buff_[8] | (buff_[9] << 8);
    d_bus_data_.z = buff_[10] | (buff_[11] << 8);
    d_bus_data_.l = buff_[12];
    d_bus_data_.r = buff_[13];
    d_bus_data_.key = buff_[14] | buff_[15] << 8; // key board code
    d_bus_data_.wheel = (buff_[16] | buff_[17] << 8) & 0x07FF;
    d_bus_data_.wheel -= 1024;
    int wheel_check = (buff_[16] | buff_[17] << 8);
    if (abs(wheel_check - 1024) > 660)
    {
        d_bus_data_.wheel = 660;
    }
    is_success = true;
    /*cout << "ch0:" << (int)d_bus_data_.ch0 << " ";
    cout << "ch1:" << (int)d_bus_data_.ch1 << " ";
    cout << "ch2:" << (int)d_bus_data_.ch2 << " ";
    cout << "ch3:" << (int)d_bus_data_.ch3 << " ";
    cout << "s0:" << (int)d_bus_data_.s0 << " ";
    cout << "s1:" << (int)d_bus_data_.s1 << " ";
    cout << "x:" << (int)d_bus_data_.x << " ";
    cout << "y:" << (int)d_bus_data_.y << " ";
    cout << "z:" << (int)d_bus_data_.z << " ";
    cout << "l:" << (int)d_bus_data_.l << " ";
    cout << "r:" << (int)d_bus_data_.r << " ";
    cout << "key:" << (int)d_bus_data_.key << " ";
    cout << "wheel:" << (int)d_bus_data_.wheel << "\n";*/
}

bool DBus::isUpdate()
{
    return is_update_;
}

int16_t DBus::getCh0()
{
    return d_bus_data_.ch0;
}

int16_t DBus::getCh1()
{
    return d_bus_data_.ch1;
}

int16_t DBus::getCh2()
{
    return d_bus_data_.ch2;
}

int16_t DBus::getCh3()
{
    return d_bus_data_.ch3;
}

uint8_t DBus::getS0()
{
    return d_bus_data_.s0;
}

uint8_t DBus::getS1()
{
    return d_bus_data_.s1;
}

int16_t DBus::getWheel()
{
    return d_bus_data_.wheel;
}

int16_t DBus::getX()
{
    return d_bus_data_.x;
}

int16_t DBus::getY()
{
    return d_bus_data_.y;
}

int16_t DBus::getZ()
{
    return d_bus_data_.z;
}

uint8_t DBus::getL()
{
    return d_bus_data_.l;
}

uint8_t DBus::getR()
{
    return d_bus_data_.r;
}

uint16_t DBus::getKey()
{
    return d_bus_data_.key;
}