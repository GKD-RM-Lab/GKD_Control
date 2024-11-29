#pragma once
#include <memory>
#include <thread>

#include "can.hpp"
#include "chassis/chassis.hpp"
#include "device/cv_controller.hpp"
#include "device/imu.hpp"
#include "gimbal/gimbal.hpp"
#include "robot.hpp"
#include "serial_interface.hpp"
#include "shoot.hpp"
#include "socket_interface.hpp"

namespace Robot
{

    class Robot_ctrl
    {
       public:
        Robot_ctrl();
        ~Robot_ctrl();

        void load_hardware();
        void start_init();
        void init_join() const;
        void start();
        void join() const;

       public:
        std::unique_ptr<std::thread> chassis_thread;
        std::unique_ptr<std::thread> gimbal_thread;
        std::unique_ptr<std::thread> gimbal_l_thread;
        std::unique_ptr<std::thread> gimbal_big_yaw_thread;
        std::unique_ptr<std::thread> vision_thread;
        std::unique_ptr<std::thread> shoot_thread;
        std::unique_ptr<std::thread> gimbal_init_thread;
        std::unique_ptr<std::thread> gimbal_l_init_thread;
        std::unique_ptr<std::thread> gimbal_big_yaw_init_thread;

        std::shared_ptr<Robot_set> robot_set;

        Device::IMU imu;
        Device::Cv_controller cv_controller_;
        Chassis::Chassis chassis;
        Gimbal::Gimbal gimbal;
        Gimbal::Gimbal_L gimbal_l;
        Gimbal::Gimbal_big_yaw gimbal_big_yaw;
        Shoot::Shoot shoot;
    };

}  // namespace Robot
