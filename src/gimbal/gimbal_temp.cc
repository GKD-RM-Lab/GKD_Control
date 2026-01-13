#include "gimbal/gimbal_temp.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>

#include "UI.hpp"
#include "gimbal/gimbal_config.hpp"
#include "macro_helpers.hpp"
#include "pid_controller.hpp"
#include "robot_controller.hpp"
#include "robot_type_config.hpp"
#include "serial/serial.h"
#include "types.hpp"
#include "user_lib.hpp"
#include "utils.hpp"
namespace Gimbal
{
    GimbalT::GimbalT(const GimbalConfig& config)
        : config(config),
          imu(config.imu_serial_port),
          yaw_motor(config.yaw_motor_config),
          pitch_motor(config.pitch_motor_config),
          yaw_set(nullptr),
          pitch_set(nullptr),
          yaw_rela(nullptr),
          shoot(config.shoot_config){
        receive_auto_aim = std::chrono::steady_clock::now();
    }

    void GimbalT::init(const std::shared_ptr<Robot::Robot_set>& robot) {
        robot_set = robot;
        shoot.init(robot);
        // TODO:gimbal_id should delete
        if (config.gimbal_id == 1) {
            yaw_set = &robot_set->gimbalT_1_yaw_set;
            pitch_set = &robot_set->gimbalT_1_pitch_set;
            yaw_rela = &robot_set->gimbalT_1_yaw_reletive;
        }

        yaw_motor.setCtrl(Pid::PidPosition(config.yaw_rate_pid_config, yaw_gyro));
        pitch_motor.setCtrl(
            Pid::PidPosition(config.pitch_rate_pid_config, pitch_gyro) >>
            Pid::Invert(config.gimbal_motor_dir));
        yaw_relative_pid = Pid::PidRad(config.yaw_relative_pid_config, yaw_relative);
        MUXDEF(
            CONFIG_SENTRY,
            yaw_absolute_pid =
                // Pid::PidRad(config.yaw_absolute_pid_config, fake_yaw_abs) >> Pid::Invert(-1),
            Pid::PidRad(config.yaw_absolute_pid_config, robot_set->gimbal_sentry_yaw) >>
            Pid::Invert(-1),
            yaw_absolute_pid =
                Pid::PidRad(config.yaw_absolute_pid_config, imu.yaw) >> Pid::Invert(-1));

        pitch_absolute_pid = Pid::PidRad(config.pitch_absolute_pid_config, imu.pitch);

        imu.enable();
        yaw_motor.enable();
        pitch_motor.enable();

        IO::io<SOCKET>["AUTO_AIM_CONTROL"]->add_client(
            config.header, config.auto_aim_ip, config.auto_aim_port);

        IO::io<SOCKET>["AUTO_AIM_CONTROL"]->register_callback_key(
            config.header, [this](const Robot::Auto_aim_control& vc) {
                LOG_INFO(
                    "socket recive %f %f %d %d\n",
                    vc.yaw_set,
                    vc.pitch_set,
                    vc.fire,
                    config.gimbal_id);
                receive_auto_aim = std::chrono::steady_clock::now();
                robot_set->set_mode(Types::ROBOT_MODE::ROBOT_FOLLOW_GIMBAL);
                robot_set->cv_fire = vc.fire;
                if (vc.fire && ISDEF(CONFIG_SENTRY)) {
                    robot_set->shoot_open |= config.gimbal_id;
                }

                // if (!ISDEF(CONFIG_SENTRY) && !robot_set->auto_aim_status)
                //     return;
                *yaw_set = vc.yaw_set;
                *pitch_set = vc.pitch_set;
            });

    }
    void GimbalT::init_task() {
        static int delta = 0;
        while (imu.offline() || yaw_motor.offline() || pitch_motor.offline()) {
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
            LOG_INFO(
                "offline %d %d %d\n", imu.offline(), yaw_motor.offline(), pitch_motor.offline());
            delta++;
            if (delta > 1000){
                exit(-1);
            }
        }

        while (robot_set->inited != Types::Init_status::INIT_FINISH) {
            update_data();
            0.f >> yaw_relative_pid >> yaw_motor;
            0.f >> pitch_absolute_pid >> pitch_motor;
            // pitch_motor.set(1);

            // LOG_INFO(
            //    "imu : %6f %6f %6f %6d\n",
            //    imu.yaw,
            //    imu.pitch,
            //    imu.roll,
            //    yaw_motor.motor_measure_.ecd);
            if (fabs(yaw_relative) < Config::GIMBAL_INIT_EXP
                // && fabs(imu.pitch) < Config::GIMBAL_INIT_EXP
            ) {
                init_stop_times += 1;
            } else {
                init_stop_times = 0;
            }
            MUXDEF(CONFIG_SENTRY, *yaw_set = robot_set->gimbal_sentry_yaw, *yaw_set = imu.yaw);
            *pitch_set = 0;

            if (init_stop_times >= Config::GIMBAL_INIT_STOP_TIME)
                robot_set->inited |= 1;
            UserLib::sleep_ms(config.ControlTime);
        }
    }

    [[noreturn]] void GimbalT::task() {
        std::jthread shoot_thread(&Shoot::Shoot::task, &shoot);
        last_mode = robot_set->mode;

        while (true) {
            update_data();
            //LOG_INFO("mode:%d",robot_set->mode);
            // LOG_INFO("%d: yaw set %f, imu yaw %f\n", config.header, *yaw_set, imu.yaw);
            // logger.push_value("gimbal.yaw.set", (double)*yaw_set);
            // logger.push_value("gimbal.yaw.imu", (double)imu.yaw);
            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                yaw_motor.set(0);
                pitch_motor.set(0);
            } else if (robot_set->mode == Types::ROBOT_MODE::ROBOT_SEARCH) {
                IFDEF(CONFIG_SENTRY,
                    if(last_mode != Types::ROBOT_MODE::ROBOT_SEARCH) {
                        search_start = std::chrono::steady_clock::now();
                    }

                    auto duration = std::chrono::duration<double>(std::chrono::steady_clock::now() - search_start);

                    fp32 y = std::sin(duration.count() * config.search_pitch_speed);
                    fp32 pitch = y * config.search_pitch_amplitude + 0.165f;

                    *pitch_set = std::clamp((double)pitch, -0.18, 0.51);

                   // LOG_INFO("pitch_speed=%f,yaw = %f,pitch = %f,start = %d,dur = %lf",config.search_pitch_speed,yaw,pitch,search_start.time_since_epoch(),duration.count());

                    *pitch_set >> pitch_absolute_pid >> pitch_motor;
                )
            } else {
                MUXDEF(
                    CONFIG_SENTRY,
                    // static float yr; static float ty;
                    // yr = -UserLib::rad_format(*yaw_set - robot_set->gimbal_sentry_yaw);
                    // if (config.gimbal_id == 1 && (yr < -M_1_PIf / 2.f || yr > M_1_PIf / 2.f)) {
                    //     if (yr > 0)
                    //         ty = robot_set->gimbal_sentry_yaw - (M_1_PIf / 2.f);
                    //     else
                    //         ty = robot_set->gimbal_sentry_yaw - (-M_1_PIf / 2.f);
                    // }

                    // ty >>
                    0 >> yaw_relative_pid >> yaw_motor;
                    // LOG_INFO("fake:%f\n", fake_yaw_abs);
                    // LOG_INFO("yaw_relative:%f\n", yaw_relative);
                    // LOG_INFO("yaw_sentry:%f\n", robot_set->gimbal_sentry_yaw);

                    ,
                    *yaw_set >> yaw_absolute_pid >> yaw_motor;)
                *pitch_set >> pitch_absolute_pid >> pitch_motor;
                // LOG_INFO("pitch_set:%f\n", *pitch_set);
            }
            last_mode = robot_set->mode;


            Robot::SendAutoAimInfo pkg;
            pkg.header = config.header;
            MUXDEF(CONFIG_SENTRY, pkg.yaw = imu.yaw, pkg.yaw = imu.yaw);
            pkg.pitch = imu.pitch;
            pkg.red = robot_set->referee_info.game_robot_status_data.robot_id < 100;
            IO::io<SOCKET>["AUTO_AIM_CONTROL"]->send(pkg);

            UserLib::sleep_ms(config.ControlTime);
        }
    }

    void GimbalT::update_data() {
        yaw_relative = UserLib::rad_format(
            yaw_motor.data_.rotor_angle - Hardware::DJIMotor::ECD_8192_TO_RAD * config.YawOffSet);
        
        // LOG_INFO("%f - %f * %f = %f\n", yaw_motor.data_.rotor_angle,
        // Hardware::DJIMotor::ECD_8192_TO_RAD, config.YawOffSet, yaw_relative);

        // auto newYawOffSet = yaw_motor.data_.rotor_angle / Hardware::DJIMotor::ECD_8192_TO_RAD;
        // LOG_INFO("Yawoffset:%f\n", newYawOffSet);

        yaw_gyro = (std::cos(imu.pitch) * imu.yaw_rate - std::sin(imu.pitch) * imu.roll_rate);
        pitch_gyro = imu.pitch_rate;
        // gimbal sentry follow needs
        *yaw_rela = yaw_relative;
        fake_yaw_abs = robot_set->gimbal_sentry_yaw - yaw_relative;
    }
}  // namespace Gimbal

