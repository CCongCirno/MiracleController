#include "chassis/chassis.h"
#define MAX_OUTPUT 3000
#define MAX_INPUT 660
#define SCALE_FACTOR (MAX_OUTPUT / (float)MAX_INPUT)

using namespace std;

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

void calculate(int16_t &ch0, int16_t &ch1)
{
    // 归一化并缩放输入信号
    float vx = ch0 * SCALE_FACTOR;
    float vy = ch1 * SCALE_FACTOR;

    // 计算四个电机的控制量（麦轮公式）
    M3508_sendInfo[0].I_data = vx - vy;
    M3508_sendInfo[1].I_data = vx + vy;
    M3508_sendInfo[2].I_data = -vx + vy;
    M3508_sendInfo[3].I_data = -vx - vy;
}

void start(Chassis *chassis)
{
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    // 创建socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0)
    {
        perror("Error while opening socket");
        return;
    }

    // 设置CAN接口
    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 绑定socket
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Error in socket bind");
        return;
    }

    struct can_frame frame;

    while (1)
    {
        calculate(chassis->controlerInfo.ch0, chassis->controlerInfo.ch1);

        // 发送CAN帧
        frame.can_id = 0x200;
        frame.can_dlc = 8;
        frame.data[0] = M3508_sendInfo[0].raw_data[1];
        frame.data[1] = M3508_sendInfo[0].raw_data[0];
        frame.data[2] = M3508_sendInfo[0].raw_data[3];
        frame.data[3] = M3508_sendInfo[0].raw_data[2];
        frame.data[4] = M3508_sendInfo[0].raw_data[5];
        frame.data[5] = M3508_sendInfo[0].raw_data[4];
        frame.data[6] = M3508_sendInfo[0].raw_data[7];
        frame.data[7] = M3508_sendInfo[0].raw_data[6];

        if (write(s, &frame, sizeof(frame)) != sizeof(frame))
        {
            perror("Error in sending frame");
            return;
        }

        frame.can_id = 0x1FF;
        frame.can_dlc = 8;
        frame.data[0] = 0x00;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;

        // 读取CAN帧
        if (read(s, &frame, sizeof(frame)) > 0)
        {
            if ((frame.can_id >> 4) == 0x20)
            {
                int id = (frame.can_id & 0x00F);

                if (id > 8)
                {
                    continue;
                }
                M3508_receiveInfo[id].angle.raw_data[0] = frame.data[1];
                M3508_receiveInfo[id].angle.raw_data[1] = frame.data[0];

                M3508_receiveInfo[id].speed.raw_data[0] = frame.data[3];
                M3508_receiveInfo[id].speed.raw_data[1] = frame.data[2];

                M3508_receiveInfo[id].current.raw_data[0] = frame.data[5];
                M3508_receiveInfo[id].current.raw_data[1] = frame.data[4];

                M3508_receiveInfo[id].temp.raw_data = frame.data[6];

                // printf("M3058 [%d]: Angle=%d Speed=%d Current=%d Temp=%d\n", id, M3508_receiveInfo[id].angle.angle_data, M3508_receiveInfo[id].speed.speed_data, M3508_receiveInfo[id].current.I_data, M3508_receiveInfo[id].temp.temp_data);
            }
        }
    }

    // 关闭socket
    close(s);
}