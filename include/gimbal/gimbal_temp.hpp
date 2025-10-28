#pragma once

#include <memory>

#include "device/imu.hpp"
#include "dji_motor.hpp"
#include "gimbal/gimbal_config.hpp"
#include "robot.hpp"
#include "shoot.hpp"

namespace Gimbal
{

    class GimbalT
    {
       public:
        explicit GimbalT(const GimbalConfig& config);
        ~GimbalT() = default;
        void init(const std::shared_ptr<Robot::Robot_set>& robot);
        void init_task();
        [[noreturn]] void task();
        void update_data();

       public:
        uint32_t init_stop_times = 0;

        fp32 yaw_gyro = 0.f;
        fp32 pitch_gyro = 0.f;
        fp32 yaw_relative = 0.f;
        fp32 fake_yaw_abs;

        fp32* yaw_set;
        fp32* pitch_set;
        fp32* yaw_rela;

        std::shared_ptr<Robot::Robot_set> robot_set;
        GimbalConfig config;

        Device::IMU imu;

        Hardware::DJIMotor yaw_motor;
        Hardware::DJIMotor pitch_motor;

        ControllerList yaw_relative_pid;
        ControllerList yaw_absolute_pid;
        ControllerList pitch_absolute_pid;

        Shoot::Shoot shoot;

        std::chrono::_V2::steady_clock::time_point receive_auto_aim;

    };

}  // namespace Gimbal
