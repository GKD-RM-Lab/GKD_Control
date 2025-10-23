#pragma once

#include <memory>

#include "dji_motor.hpp"
#include "ramp.hpp"
#include "robot.hpp"
#include "shoot_config.hpp"

namespace Shoot
{
    class Shoot
    {
       public:
        Shoot(const ShootConfig& config);
        void init(const std::shared_ptr<Robot::Robot_set>& robot);
        ~Shoot() = default;
        [[noreturn]] void task();
        bool isJam();

       public:
        bool friction_finish = false;

        int jam_time = 0;
        int back_time = 0;
        int gimbal_id = 0;

        Hardware::DJIMotor left_friction;
        Hardware::DJIMotor right_friction;
        Hardware::DJIMotor trigger;

        std::shared_ptr<Robot::Robot_set> robot_set;

       private:
        UserLib::Ramp friction_ramp;
    };
}  // namespace Shoot
