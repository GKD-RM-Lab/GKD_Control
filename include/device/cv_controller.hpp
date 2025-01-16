#pragma once

#include "device/deviece_base.hpp"
#include "types.hpp"
#include "bullet_solver.hpp"
#include "robot.hpp"
#include "memory"

namespace Device
{
    class Cv_controller : public DeviceBase
    {
       public:
				 Cv_controller() : control_ramp(20, 1.f/1000),control_ramp2(20, 1.f/1000){
				 };

        void init(const std::shared_ptr<Robot::Robot_set> &robot);
        void unpack(const Robot::ReceiveGimbalPacket &pkg);
        [[noreturn]] void task();

       public:
        Control::BulletSolver bullet_solver_;
        UserLib::Ramp control_ramp;
        UserLib::Ramp control_ramp2;

       private:
        std::shared_ptr<Robot::Robot_set> robot_set;
    };
}  // namespace Device
