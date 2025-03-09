#include <cstdint>
#include "dbus_dr16_interface/msg/dr16.hpp"

typedef struct
{
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    uint8_t s0;
    uint8_t s1;
    int16_t wheel;

    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
    uint16_t key;

} DBusData_t;

class DBus
{
public:
    DBus() = default;
    ~DBus() = default;
    void init(const char *serial);
    void DBusRead();
    bool isUpdate();
    void getDbusInfo(dbus_dr16_interface::msg::DR16 &msg);

private:
    DBusData_t d_bus_data_;
    int port_;
    int16_t buff_[18];
    bool is_success = false;
    bool is_update_ = false;
    void unpack();
};
