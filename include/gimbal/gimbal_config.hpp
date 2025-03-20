#pragma once

#include "dji_motor.hpp"
#include "pid_controller.hpp"
#include "shoot_config.hpp"

namespace Gimbal
{
    struct GimbalConfig
    {
        std::string imu_serial_port;
        Hardware::DJIMotorConfig yaw_motor_config;
        Hardware::DJIMotorConfig pitch_motor_config;
        Pid::PidConfig yaw_rate_pid_config{};
        Pid::PidConfig pitch_rate_pid_config{};
        Pid::PidConfig yaw_relative_pid_config{};
        Pid::PidConfig yaw_absolute_pid_config{};
        Pid::PidConfig pitch_relative_pid_config{};
        Pid::PidConfig pitch_absolute_pid_config{};
        float gimbal_motor_dir;
        int gimbal_id;
        const int ControlTime{};
        const fp32 YawOffSet{};
        const fp32 PitchOffSet{};
        Shoot::ShootConfig shoot_config;
    };
}  // namespace Gimbal
