#include "can/can.h"
using namespace std;
void CAN::canRead(can_interface::msg::MotorInfo &msg)
{
    struct can_frame frame;
    updateFlag = false;
    // 读取CAN帧
    if (read(socket_id, &frame, sizeof(frame)) > 0)
    {
        if ((frame.can_id >> 4) == 0x20)
        {
            int id = (frame.can_id & 0x00F);

            if (id > 8)
            {
                return;
            }
            id--;
            msg.type = 1; // M3508
            msg.id = id;
            M3508_receiveInfo[id].angle.raw_data[0] = frame.data[1];
            M3508_receiveInfo[id].angle.raw_data[1] = frame.data[0];
            msg.angle = M3508_receiveInfo[id].angle.angle_data;

            M3508_receiveInfo[id].speed.raw_data[0] = frame.data[3];
            M3508_receiveInfo[id].speed.raw_data[1] = frame.data[2];
            msg.speed = M3508_receiveInfo[id].speed.speed_data;

            M3508_receiveInfo[id].current.raw_data[0] = frame.data[5];
            M3508_receiveInfo[id].current.raw_data[1] = frame.data[4];
            msg.current = M3508_receiveInfo[id].current.current_data;

            M3508_receiveInfo[id].temp.raw_data = frame.data[6];
            msg.temp = M3508_receiveInfo[id].temp.temp_data;

            updateFlag = true;

            fmt::print("[{}] M3058 [{:1d}]: Angle={:5d} Speed={:5d} Current={:5d} Temp={:2d}\n", idntifier, id + 1, M3508_receiveInfo[id].angle.angle_data, M3508_receiveInfo[id].speed.speed_data, M3508_receiveInfo[id].current.current_data, M3508_receiveInfo[id].temp.temp_data);
        }
    }
}

void CAN::canSend(const can_interface::msg::MotorSendInfo::SharedPtr msg)
{
    struct can_frame frame;
    if (msg->type == 1)
    {
        for (int i = 0; i < 8; i++)
        {
            if (i < msg->size)
            {
                sendInfo[i].current_data = msg->current[i];
            }
            else
            {
                sendInfo[i].current_data = 0;
            }
        }
        frame.can_id = 0x200;
        frame.can_dlc = 8;
        frame.data[0] = sendInfo[0].raw_data[1];
        frame.data[1] = sendInfo[0].raw_data[0];
        frame.data[2] = sendInfo[1].raw_data[1];
        frame.data[3] = sendInfo[1].raw_data[0];
        frame.data[4] = sendInfo[2].raw_data[1];
        frame.data[5] = sendInfo[2].raw_data[0];
        frame.data[6] = sendInfo[3].raw_data[1];
        frame.data[7] = sendInfo[3].raw_data[0];

        if (write(socket_id, &frame, sizeof(frame)) != sizeof(frame))
        {
            fmt::print("[{}] Error in sending frame.\n", error_idntifier);
            return;
        }

        frame.can_id = 0x1FF;
        frame.can_dlc = 8;
        frame.data[0] = sendInfo[4].raw_data[1];
        frame.data[1] = sendInfo[4].raw_data[0];
        frame.data[2] = sendInfo[5].raw_data[1];
        frame.data[3] = sendInfo[5].raw_data[0];
        frame.data[4] = sendInfo[6].raw_data[1];
        frame.data[5] = sendInfo[6].raw_data[0];
        frame.data[6] = sendInfo[7].raw_data[1];
        frame.data[7] = sendInfo[7].raw_data[0];

        if (write(socket_id, &frame, sizeof(frame)) != sizeof(frame))
        {
            fmt::print("[{}] Error in sending frame.\n", error_idntifier);
            return;
        }
    }
}

int CAN::canInit(int id)
{

    struct sockaddr_can addr;
    struct ifreq ifr;
    this->id = id;
    idntifier = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "CAN{}", this->id);
    // 创建socket
    socket_id = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_id < 0)
    {
        fmt::print("[{}] Error while opening socket.\n", error_idntifier);
        return -1;
    }

    // 设置CAN接口
    strcpy(ifr.ifr_name, can_port[id]);
    ioctl(socket_id, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 绑定socket
    if (bind(socket_id, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        fmt::print("[{}] Error in socket bind.\n", error_idntifier);
        return -1;
    }
    return 0;
}