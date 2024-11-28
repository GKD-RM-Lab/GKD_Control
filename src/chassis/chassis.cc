#include "chassis/chassis.hpp"
#include "chassis_config.hpp"

#include "config.hpp"
#include "user_lib.hpp"

namespace Chassis
{
    Chassis::Chassis(const ChassisConfig &config) : config(config),
    motors(config.wheels_config.begin(), config.wheels_config.end()) {}

    void Chassis::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;
        chassis_angle_pid = Pid::PidRad(config.chassis_follow_gimbal_pid_config, robot_set->gimbal3_yaw_relative);
        for (auto &motor : motors) {
            motor.setCtrl(Pid::PidPosition(config.wheel_speed_pid_config, motor.data_.output_linear_velocity));
            motor.enable();
        }
    }

    [[noreturn]] void Chassis::task() {
        while (true) {
            decomposition_speed();
            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                for (auto &motor : motors) {
                    motor.set(0.f);
                }
            } else {
                fp32 max_speed = 0.f;
                for (int i = 0; i < 4; i++) {
                    max_speed = std::max(max_speed, fabsf(wheel_speed[i]));
                }
                if (max_speed > max_wheel_speed) {
                    fp32 speed_rate = max_wheel_speed / max_speed;
                    for (int i = 0; i < 4; i++) {
                        wheel_speed[i] *= speed_rate;
                    }
                }
                for (int i = 0; i < 4; i++) {
                    motors[i].set(wheel_speed[i]);
                }
            }
            UserLib::sleep_ms(config.ChassisControlTime);
        }
    }

    void Chassis::decomposition_speed() {
        if (robot_set->mode != Types::ROBOT_MODE::ROBOT_NO_FORCE) {
            fp32 sin_yaw, cos_yaw;
            sincosf(robot_set->gimbal3_yaw_relative, &sin_yaw, &cos_yaw);
            vx_set = cos_yaw * robot_set->vx_set - sin_yaw * robot_set->vy_set;
            vy_set = sin_yaw * robot_set->vx_set + cos_yaw * robot_set->vy_set;

            if (robot_set->wz_set == 0.f) {
                chassis_angle_pid.set(0.f);
                wz_set = chassis_angle_pid.out;
            } else {
                wz_set = robot_set->wz_set;
            }
        }

        wheel_speed[0] = -vx_set + vy_set + wz_set;
        wheel_speed[1] = vx_set + vy_set + wz_set;
        wheel_speed[2] = vx_set - vy_set + wz_set;
        wheel_speed[3] = -vx_set - vy_set + wz_set;
    }
}  // namespace Chassis
