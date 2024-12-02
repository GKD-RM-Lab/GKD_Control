#pragma once

#include "gimbal/gimbal_temp.hpp"
#include "gimbal/gimbal_config.hpp"
#include "device/M9025.hpp"

namespace Gimbal
{
    class GimbalSentry
    {
    public:
        GimbalSentry();
        ~GimbalSentry() = default;
        void init(const std::shared_ptr<Robot::Robot_set> &robot);
        void init_task();
        [[noreturn]] void task();
        void update_data();

    public:
        std::shared_ptr<Robot::Robot_set> robot_set;

        bool inited = false;

        Device::IMU imu;
        Device::M9025 yaw_motor;

        ControllerList yaw_rate_pid;
        ControllerList yaw_absolute_pid;
        ControllerList yaw_relative_pid;

        fp32 yaw_relative = 0.f;
        fp32 yaw_gyro = 0.f;
        fp32 yaw_motor_speed = 0.f;

        int init_stop_times = 0;
    };
}
