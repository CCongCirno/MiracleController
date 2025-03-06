#include <cstdint>

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
    int16_t getCh0();
    int16_t getCh1();
    int16_t getCh2();
    int16_t getCh3();
    uint8_t getS0();
    uint8_t getS1();
    int16_t getWheel();
    int16_t getX();
    int16_t getY();
    int16_t getZ();
    uint8_t getL();
    uint8_t getR();
    uint16_t getKey();
    void start();

private:
    DBusData_t d_bus_data_;
    int port_;
    int16_t buff_[18];
    bool is_success = false;
    bool is_update_ = false;
    void unpack();
};
