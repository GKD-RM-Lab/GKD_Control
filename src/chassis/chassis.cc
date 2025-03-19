#include "chassis/chassis.hpp"

#include <thread>

#include "chassis_config.hpp"
#include "robot_type_config.hpp"
#include "user_lib.hpp"

namespace Chassis
{
    Chassis::Chassis(const ChassisConfig &config)
        : config(config),
          motors(config.wheels_config.begin(), config.wheels_config.end()),
          power_manager(motors, Power::Division::HERO) {
    }

    void Chassis::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;

        power_manager.init(robot);
        power_manager.setMode(1);

        chassis_angle_pid = Pid::PidRad(
                                config.chassis_follow_gimbal_pid_config,
                                MUXDEF(
                                    CONFIG_SENTRY,
                                    robot_set->gimbal_sentry_yaw_reletive,
                                    robot_set->gimbalT_1_yaw_reletive)) >>
                            Pid::Invert(config.follow_dir);

        for (auto &motor : motors) {
            motor.setCtrl(Pid::PidPosition(
                config.wheel_speed_pid_config, motor.data_.output_linear_velocity));
            motor.enable();
        }

        for (int i = 0; i < 4; i++) {
            wheels_pid[i] = Pid::PidPosition(
                config.wheel_speed_pid_config, motors[i].data_.output_linear_velocity);
        }
    }

    [[noreturn]] void Chassis::task() {
        std::jthread power_daemon(&Power::Manager::powerDaemon, &power_manager);
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
                    wheels_pid[i].set(wheel_speed[i]);
                }

                // Power Limit
                for (int i = 0; i < 4; ++i) {
                    objs[i].curAv = motors[i].motor_measure_.speed_rpm * M_PIf / 30;
                    objs[i].pidOutput = wheels_pid[i].out;
                    objs[i].setAv = wheel_speed[i];
                    objs[i].pidMaxOutput = 14000;
                }
                static Power::PowerObj *pObjs[4] = { &objs[0], &objs[1], &objs[2], &objs[3] };
                float *cmd_power = power_manager.getControlledOutput(pObjs);

                for (int i = 0; i < 4; ++i) {
                    motors[i].give_current = cmd_power[i];
                }

                // for (int i = 0; i < 4; ++i) {
                //     motors[i].give_current = wheels_pid[i].out;
                // }
            }
            UserLib::sleep_ms(config.ControlTime);
        }
    }

    void Chassis::decomposition_speed() {
        if (robot_set->mode != Types::ROBOT_MODE::ROBOT_NO_FORCE) {
            fp32 sin_yaw, cos_yaw;
            MUXDEF(
                CONFIG_SENTRY,
                sincosf(robot_set->gimbal_sentry_yaw_reletive, &sin_yaw, &cos_yaw),
                sincosf(robot_set->gimbalT_1_yaw_reletive, &sin_yaw, &cos_yaw));
            vx_set = cos_yaw * robot_set->vx_set + sin_yaw * robot_set->vy_set;
            vy_set = -sin_yaw * robot_set->vx_set + cos_yaw * robot_set->vy_set;

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
