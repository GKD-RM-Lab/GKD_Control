#pragma once
#include "pid_controller.hpp"
#include "robot.hpp"

#include <memory>
#include <array>

namespace Chassis {
    struct MecConfig {
        Pid::Pid_config speed_pid_config;
        Pid::Pid_config follow_gimbal_pid_config;
    };

    struct SwerveConfig {
        Pid::Pid_config speed_pid_config;
        Pid::Pid_config turn_pid_config;
        Pid::Pid_config follow_gimbal_pid_config;
        Pid::Pid_config turn_pos_pid_config;   //May I have your attention please:hello,this is position pid
        std::array<uint16_t, 4> turn_init_ecd;
    };

    class ChassisBase {
       protected:
        std::shared_ptr<Robot::Robot_set> robot_set_;

       public:
        virtual ~ChassisBase() = default; 
        virtual void task() = 0;
        virtual void init_task() {};
        virtual void init(const std::shared_ptr<Robot::Robot_set> &robot_set) = 0;
    };
}
