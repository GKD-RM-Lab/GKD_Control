#include "gimbal/gimbal_sentry.hpp"

#include <future>

#include "robot_type_config.hpp"
#include "socket_interface.hpp"
#include "types.hpp"
#include "user_lib.hpp"
#include "utils.hpp"

namespace Gimbal
{
    GimbalSentry::GimbalSentry(const GimbalConfig& config)
        : config(config),
          imu(config.imu_serial_port),
          yaw_motor("CAN_BULLET", 1)

    {
    }

    void GimbalSentry::init(const std::shared_ptr<Robot::Robot_set>& robot) {
        robot_set = robot;

        yaw_absolute_pid = Pid::PidRad(config.yaw_absolute_pid_config, imu.yaw) >> Pid::Invert(-1);
        yaw_relative_pid = Pid::PidRad(config.yaw_relative_pid_config, yaw_relative);
        yaw_relative_with_two_head_pid =
            Pid::PidRad(Config::GIMBAL_9025_YAW_RELATIVE_PID_CONFIG, yaw_relative_with_two_head) >>
            Pid::Invert(-1);

        yaw_motor.setCtrl(Pid::PidPosition(config.yaw_rate_pid_config, imu.yaw_rate));
        yaw_set = &robot_set->gimbal_sentry_yaw_set;

        imu.enable();
        yaw_motor.enable();

        // IO::io<SOCKET>["AUTO_AIM_CONTROL"] -> register_callback([this](const
        // Robot::ReceiveNavigationInfo& vc) {
        //     // LOG_INFO("navigation receive %f %f\n", vc.vx, vc.vy);
        //     robot_set->vx_set = vc.vx;
        //     robot_set->vy_set = vc.vy;
        // });

        // IO::io<SOCKET>["AUTO_AIM_CONTROL"]->add_client(0x37, "192.168.10.2", 11453);
    }

    void GimbalSentry::init_task() {
        while (robot_set->inited != Types::Init_status::INIT_FINISH) {
            update_data();
            0.f >> yaw_relative_pid >> yaw_motor;
            // LOG_INFO("big yaw %d %f\n", yaw_motor.motor_measure.ecd, yaw_relative);
            // LOG_INFO("yaw r %f\n", yaw_relative);
            *yaw_set = imu.yaw;
            if (fabs(yaw_relative) < Config::GIMBAL_INIT_EXP) {
                init_stop_times += 1;
            } else {
                init_stop_times = 0;
            }
            if (init_stop_times >= static_cast<int>(Config::GIMBAL_INIT_STOP_TIME))
                robot_set->inited |= 1 << 2;
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
    }

    [[noreturn]] void GimbalSentry::task() {
        while (true) {
            update_data();
            switch (robot_set->mode) {
                case Types::ROBOT_MODE::ROBOT_NO_FORCE: 0 >> yaw_motor; break;
                case Types::ROBOT_MODE::ROBOT_FINISH_INIT:
                case Types::ROBOT_MODE::ROBOT_IDLE:
                case Types::ROBOT_MODE::ROBOT_SEARCH:
                    *yaw_set >> yaw_absolute_pid >> yaw_motor;
                    break;
                default: 0.f >> yaw_relative_with_two_head_pid >> yaw_motor; break;
            };

            Robot::SendNavigationInfo gimbal_info;
            gimbal_info.header = 0x37;
            gimbal_info.yaw = imu.yaw;
            gimbal_info.pitch = imu.pitch;
            gimbal_info.hp = robot_set->referee_info.game_robot_status_data.remain_hp * 1. /
                             robot_set->referee_info.game_robot_status_data.max_hp;
            gimbal_info.start =
                (robot_set->referee_info.game_status_data.game_progress & 0x0f) == 4;

            // FIXME: random robot_set used
            if (gimbal_info.start) {
                robot_set->wz_set = 0.3;
                robot_set->friction_open = true;
            }
            // LOG_INFO("game progress %d\n", robot_set->referee_info.game_status_data.game_progress
            // & 0x0f); IO::io<SOCKET>["AUTO_AIM_CONTROL"]->send(gimbal_info);

            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
    }

    void GimbalSentry::update_data() {
        yaw_motor_speed = Config::RPM_TO_RAD_S * (fp32)yaw_motor.motor_measure.speed_rpm;
        yaw_relative = UserLib::rad_format(
            Config::M9025_ECD_TO_RAD *
            ((fp32)yaw_motor.motor_measure.ecd - Config::GIMBAL3_YAW_OFFSET_ECD));
        yaw_relative_with_two_head =
            robot_set->gimbalT_1_yaw_reletive + robot_set->gimbalT_2_yaw_reletive;
        robot_set->gimbal_sentry_yaw_reletive = yaw_relative;
        robot_set->gimbal_sentry_yaw = imu.yaw;
    }
}  // namespace Gimbal
