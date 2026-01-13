#pragma once

#include "device/M9025.hpp"
#include "gimbal/gimbal_config.hpp"
#include "gimbal/gimbal_temp.hpp"

namespace Gimbal
{
    class GimbalSentry
    {
       public:
        GimbalSentry(const GimbalConfig& config);
        ~GimbalSentry() = default;
        void init(const std::shared_ptr<Robot::Robot_set>& robot);
        void init_task();
        [[noreturn]] void task();
        void update_data();

        void test_yaw_speed_pid();
        void test_yaw_position_pid();

       public:
        std::shared_ptr<Robot::Robot_set> robot_set;
        GimbalConfig config;

        Device::IMU imu;
        Device::M9025 yaw_motor;

        ControllerList yaw_rate_pid;
        ControllerList yaw_absolute_pid;
        ControllerList yaw_relative_pid;
        ControllerList yaw_relative_with_head_pid;

        fp32 yaw_relative = 0.f;
        fp32 yaw_relative_with_head = 0.f;
        fp32 yaw_gyro = 0.f;
        fp32 yaw_motor_speed = 0.f;
        fp32* yaw_set;

        int init_stop_times = 0;
        std::chrono::steady_clock::time_point search_start_time;
        Types::ROBOT_MODE last_mode;
        fp32 search_start_yaw;
    };
}  // namespace Gimbal
