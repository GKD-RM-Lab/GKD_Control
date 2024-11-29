#pragma once

#include "dji_motor.hpp"
#include "pid_controller.hpp"

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
        Pid::PidConfig pitch_absolute_pid_config{};
        const int ControlTime{};
        const fp32 YawOffSet{};
    };
}
