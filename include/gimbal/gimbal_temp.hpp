#pragma once

#include <memory>

#include "can.hpp"
#include "config.hpp"
#include "io.hpp"
#include "dji_motor.hpp"
#include "robot.hpp"


namespace Gimbal
{

    class Gimbal
    {
    public:
        Gimbal();
        ~Gimbal() = default;
        void init(const std::shared_ptr<Robot::Robot_set> &robot);
        void init_task();
        [[noreturn]] void task();
        void update_data();

    public:
        bool no_force = true;
        bool searching = true;
        bool inited = false;

        uint32_t init_stop_times = 0;

        fp32 init_yaw_set = 0.f;
        fp32 init_pitch_set = 0.f;

        fp32 yaw_gyro = 0.f;
        fp32 pitch_gyro = 0.f;

        std::shared_ptr<Robot::Robot_set> robot_set;

        Hardware::DJIMotor yaw_motor;
        Hardware::DJIMotor pitch_motor;

        ControllerList yaw_absolute_pid;
        ControllerList pitch_absolute_pid;
        ControllerList yaw_relative_pid;
        ControllerList pitch_relative_pid;
    };

}  // namespace Gimbal