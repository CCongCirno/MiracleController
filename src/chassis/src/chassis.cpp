#include "chassis/chassis.h"

using namespace std;

void PID::errorEvaluate(double &vx, double &vy, double &rot, Chassis *chassis)
{
    int maxn, minn, sum = 0;
    if (err_cnt < 20)
    {
        err_list[err_cnt] = (int)(vx - vy + rot) - (int)chassis->receiveInfo[0].speed;
        err_cnt++;
        for (int j = 0; j < err_cnt; ++j)
        {
            maxn = (maxn < err_list[j]) ? err_list[j] : maxn;
            minn = (minn > err_list[j]) ? err_list[j] : minn;
            sum = sum + err_list[j];
        }
    }
    else
    {
        for (int j = 0; j < 19; ++j)
        {
            err_list[j] = err_list[j + 1];
        }
        err_list[19] = (int)(vx - vy + rot) - (int)chassis->receiveInfo[0].speed;
        maxn = err_list[19], minn = err_list[19], sum = err_list[19];
        for (int j = 0; j < 19; ++j)
        {
            maxn = (maxn < err_list[j]) ? err_list[j] : maxn;
            minn = (minn > err_list[j]) ? err_list[j] : minn;
            sum = sum + err_list[j];
        }
    }
    fmt::print("[{}] M3508[{:1d}] now:{:4d} err:{:4d} maxn:{:4d} minn:{:4d} avg:{:>5.1f}\n", pid_idntifier, id, (int)chassis->receiveInfo[0].speed, (int)(vx - vy + rot) - (int)chassis->receiveInfo[0].speed, maxn, minn, sum * 1.0 / err_cnt);
}

void Chassis::calculate()
{
    // 归一化并缩放输入信号
    updateFlag = false;
    double vx = controlerInfo.ch0 * SCALE_FACTOR;
    double vy = controlerInfo.ch1 * SCALE_FACTOR;
    double rot = controlerInfo.ch2 * SCALE_FACTOR;
    sendCurrentInfo[0] = (int16_t)pid_controller[0].compute((vx - vy + rot) * 1.0, (receiveInfo[0].speed) * 1.0);
    sendCurrentInfo[1] = (int16_t)pid_controller[1].compute((vx + vy + rot) * 1.0, (receiveInfo[1].speed) * 1.0);
    sendCurrentInfo[2] = (int16_t)pid_controller[2].compute((-vx + vy + rot) * 1.0, (receiveInfo[2].speed) * 1.0);
    sendCurrentInfo[3] = (int16_t)pid_controller[3].compute((-vx - vy + rot) * 1.0, (receiveInfo[3].speed) * 1.0);
    for (int i = 0; i < 4; ++i)
    {
        pid_controller[i].errorEvaluate(vx, vy, rot, this);
    }
    updateFlag = true;
}

void Chassis::canSend(can_interface::msg::MotorSendInfo &msg)
{
    msg.type = 1;
    msg.size = 4;
    for (int i = 0; i < 4; i++)
    {
        msg.current[i] = sendCurrentInfo[i];
    }
}

void Chassis::init()
{
    pid_controller[0].setPID(15.5, 0.08, 0.06, -4000.0, 4000.0, 0);
    pid_controller[1].setPID(15.5, 0.08, 0.06, -4000.0, 4000.0, 1);
    pid_controller[2].setPID(15.5, 0.08, 0.06, -4000.0, 4000.0, 2);
    pid_controller[3].setPID(15.5, 0.08, 0.06, -4000.0, 4000.0, 3);
}
