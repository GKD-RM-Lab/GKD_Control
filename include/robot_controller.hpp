#pragma once
#include <memory>
#include <thread>

#include "chassis/chassis.hpp"
#include "device/super_cap.hpp"
#include "gimbal/gimbal_sentry.hpp"
#include "gimbal/gimbal_temp.hpp"
#include "rc_controller.hpp"
#include "referee.hpp"
#include "robot.hpp"
#include "robot_type_config.hpp"
#include "serial_interface.hpp"
#include "shoot.hpp"
#include "socket_interface.hpp"
#include "logger.hpp"

namespace Robot
{

    class Robot_ctrl
    {
       public:
        Robot_ctrl();
        ~Robot_ctrl();

        void load_hardware();
        void start_init();
        void init_join();
        void start();
        void join();

       public:
        std::vector<std::jthread> threads;

        std::shared_ptr<Robot_set> robot_set;

        Device::Rc_Controller rc_controller;
        Device::Dji_referee referee;
        Chassis::Chassis chassis;
        Config::GimbalType gimbal;
        IFDEF(CONFIG_SENTRY, Gimbal::GimbalT gimbal_sentry);

        Device::Super_Cap super_cap;
    };

}  // namespace Robot
