#include <cstdio>
#include <chrono>
#include <unistd.h>
#include <iostream>
#include <cstring>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <fmt/core.h>
#include <fmt/color.h>
#include <fmt/format.h>

#define MAX_OUTPUT_ 4000
#define MAX_INPUT_ 660
#define SCALE_FACTOR (MAX_OUTPUT_ / (float)MAX_INPUT_)

class Chassis;

class PID
{
public:
    PID() = default;
    ~PID() = default;
    void setPID(double Kp, double Ki, double Kd, double min_out, double max_out, int id)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->min_out = min_out;
        this->max_out = max_out;
        this->id = id;
    }

    double compute(double setpoint, double measured_value)
    {
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - prev_time;
        double dt = elapsed.count(); // 计算时间间隔（秒）

        double error = setpoint - measured_value;
        integral += error * dt;                        // 计算积分项
        double derivative = (error - prev_error) / dt; // 计算微分项
        double output = Kp * error + Ki * integral + Kd * derivative;

        if (output > max_out)
            output = max_out;
        if (output < min_out)
            output = min_out;

        prev_error = error;
        prev_time = current_time; // 更新时间戳

        return output;
    }

    void errorEvaluate(double &vx, double &vy, double &rot, Chassis *chassis);
    std::string pid_idntifier = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "PID");

private:
    double Kp, Ki, Kd;
    double prev_error;
    double integral;
    double max_out, min_out;                         // 输出限幅
    std::chrono::steady_clock::time_point prev_time; // 记录上一次时间戳
    int err_list[20], err_cnt;
    int id;
};

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

    PID pid_controller[4];
    std::string error_idntifier = fmt::format(fg(fmt::color::red) | fmt::emphasis::bold, "ERROR");
    std::string chassis_idntifier = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "CHASSIS");

private:
};

void start(Chassis *chassis);