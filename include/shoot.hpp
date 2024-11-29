#pragma once

#include "dji_motor.hpp"
#include "ramp.hpp"
#include "robot.hpp"
#include <memory>

namespace Shoot
{
    class Shoot
    {
       public:
        Shoot();
        void init(const std::shared_ptr<Robot::Robot_set> &robot);
        ~Shoot() = default;
        void update_speed();
        void decomposition_speed();
        [[noreturn]] void task();
        bool isJam();

       public:
        bool friction_finish = false;

        int jam_time = 0;
        int back_time = 0;

        //std::vector<Hardware::Motor> friction;
        //std::vector<Hardware::Motor> trigger;

        std::shared_ptr<Robot::Robot_set> robot_set;
       private:
        UserLib::Ramp friction_ramp;
    };
}  // namespace Shoot
