#pragma once

#include "dji_motor.hpp"
#include <iostream>
#include <array>
#include "pid_controller.hpp"

#include "io.hpp"

namespace Chassis
{
    struct ChassisConfig
    {
        std::array<Hardware::DJIMotorConfig, 4> wheels_config;
        Pid::PidConfig chassis_follow_gimbal_pid_config{};
        Pid::PidConfig wheel_speed_pid_config{};
        const int ChassisControlTime{};
    };
}  // namespace Chassis
