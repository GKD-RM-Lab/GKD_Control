#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include <deque>
#include <memory>

#include "chassis/chassis_config.hpp"
#include "controller.hpp"
#include "dji_motor.hpp"
#include "pid_controller.hpp"
#include "power_controller.hpp"
#include "robot.hpp"
#include "types.hpp"

namespace Chassis
{
    class Chassis
    {
       public:
        Chassis(const ChassisConfig& config);
        ~Chassis() = default;
        void update_data();
        void init(const std::shared_ptr<Robot::Robot_set>& robot);
        void decomposition_speed();
        [[noreturn]] void task();
        void power_daemon(void* pvParam);

       public:
        // chassis set vertical speed,positive means forward,unit m/s.底盘设定速度 前进方向
        // 前为正，单位 m/s
        fp32 vx_set = 0.f;
        // chassis set horizontal speed,positive means left,unit m/s.底盘设定速度 左右方向
        // 左为正，单位 m/s
        fp32 vy_set = 0.f;
        // chassis set rotation speed,positive means counterclockwise,unit
        // rad/s.底盘设定旋转角速度，逆时针为正 单位 rad/s
        fp32 wz_set = 0.f;

       private:
        ChassisConfig config;
        Power::Manager power_manager;

        // chassis vertical speed, positive means forward,unit m/s. 底盘速度 前进方向 前为正，单位
        // m/s
        fp32 vx = 0.f;
        // chassis horizontal speed, positive means letf,unit m/s.底盘速度 左右方向 左为正  单位 m/s
        fp32 vy = 0.f;
        // chassis rotation speed, positive means counterclockwise,unit
        // rad/s.底盘旋转角速度，逆时针为正
        fp32 wz = 0.f;
        fp32 wheel_speed[4] = {};

        fp32 max_wheel_speed = 2.5f;
        ControllerList chassis_angle_pid;
        ControllerList wheels_pid[4];
        Power::PowerObj objs[4];

        std::deque<Hardware::DJIMotor> motors;
        std::shared_ptr<Robot::Robot_set> robot_set;
    };
}  // namespace Chassis

#endif
