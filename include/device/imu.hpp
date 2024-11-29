#pragma once

#include "device/deviece_base.hpp"
#include "robot.hpp"
#include "types.hpp"
#include "memory"

namespace Device
{
    class IMU : public DeviceBase
    {
       public:
        explicit IMU(const std::string& serial_name);

        fp32 yaw = 0;
        fp32 pitch = 0;
        fp32 roll = 0;
        fp32 yaw_rate = 0;
        fp32 pitch_rate = 0;
        fp32 roll_rate = 0;

        void enable();
        void unpack(const Types::ReceivePacket &pkg);
        void unpack1(const Types::ReceivePacket &pkg);
        void unpack2(const Types::ReceivePacket &pkg);
        void unpack3(const Types::ReceivePacket &pkg);

       private:
        std::string serial_name;
        std::shared_ptr<Robot::Robot_set> robot_set;
    };
}
