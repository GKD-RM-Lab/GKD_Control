#ifndef __GIMBAL__
#define __GIMBAL__

#include <memory>

#include "can.hpp"
#include "config.hpp"
#include "hardware.hpp"
#include "motor.hpp"
#include "robot.hpp"

namespace Gimbal
{
    class Gimbal_base
    {
       public:
        Gimbal_base() = default;
        ~Gimbal_base() = default;
        void init(const std::shared_ptr<Robot::Robot_set> &robot);
        void init_task();
        [[noreturn]] void task();
        void update_data();

        bool no_force = true;
        bool searching = true;
        bool inited = false;
        uint32_t init_stop_times = 0;
        fp32 init_yaw_set = 0.f;
        fp32 init_pitch_set = 0.f;
        fp32 yaw_gyro = 0.f;
        fp32 yaw_abs = 0.f;
        fp32 pitch_gyro = 0.f;

        std::shared_ptr<Robot::Robot_set> robot_set;
    };

    class Gimbal : public Gimbal_base
    {
       public:
        Gimbal();
        ~Gimbal() = default;
        void init(const std::shared_ptr<Robot::Robot_set> &robot);
        void init_task();
        [[noreturn]] void task();
        void update_data();

       public:
        Hardware::Motor yaw_motor;
        Hardware::Motor pitch_motor;

        Pid::Pid_rad pitch_absolute_pid;
        Pid::Pid_rad yaw_absolute_pid;
        Pid::Pid_rad yaw_relative_pid;
    };

    class Gimbal_L : public Gimbal
    {
       public:
        Gimbal_L();
        ~Gimbal_L() = default;

        void init(const std::shared_ptr<Robot::Robot_set> &robot);
        void init_task();
        [[noreturn]] void task();
        void update_data();
    };

    class Gimbal_big_yaw : public Gimbal_base
    {
       public:
        Gimbal_big_yaw();
        ~Gimbal_big_yaw() = default;

        void init(const std::shared_ptr<Robot::Robot_set> &robot);
        void init_task();
        [[noreturn]] void task();
        void update_data();

        bool inited = false;
        Hardware::Motor_9025 yaw_motor;
        Pid::Pid_rad yaw_absolute_pid;
        Pid::Pid_rad yaw_relative_pid;
    };

}  // namespace Gimbal
#endif
