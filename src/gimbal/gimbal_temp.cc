#include "gimbal/gimbal_temp.hpp"

#include <algorithm>

#include "gimbal/gimbal_config.hpp"
#include "robot_controller.hpp"
#include "robot_type_config.hpp"
#include "serial/serial.h"
#include "types.hpp"
#include "user_lib.hpp"
#include "utils.hpp"

namespace Gimbal
{
    GimbalT::GimbalT(const GimbalConfig &config)
        : config(config),
          imu(config.imu_serial_port),
          c_imu(config.c_imu_serial_port),
          yaw_motor(config.yaw_motor_config),
          pitch_motor(config.pitch_motor_config),
          yaw_set(nullptr),
          pitch_set(nullptr),
          yaw_rela(nullptr),
          shoot(config.shoot_config) {
    }

    void GimbalT::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;
        shoot.init(robot);
        if (config.gimbal_id == 1) {
            yaw_set = &robot_set->gimbalT_1_yaw_set;
            pitch_set = &robot_set->gimbalT_1_pitch_set;
            yaw_rela = &robot_set->gimbalT_1_yaw_reletive;
        } else {
            yaw_set = &robot_set->gimbalT_2_yaw_set;
            pitch_set = &robot_set->gimbalT_2_pitch_set;
            yaw_rela = &robot_set->gimbalT_2_yaw_reletive;
        }

        yaw_motor.setCtrl(Pid::PidPosition(config.yaw_rate_pid_config, yaw_gyro));
        pitch_motor.setCtrl(
            Pid::PidPosition(config.pitch_rate_pid_config, pitch_gyro) >> Pid::Invert(config.gimbal_motor_dir));

        yaw_relative_pid = Pid::PidRad(config.yaw_relative_pid_config, yaw_relative);
        MUXDEF(
            CONFIG_SENTRY,
            yaw_absolute_pid = Pid::PidRad(config.yaw_absolute_pid_config, fake_yaw_abs) >> Pid::Invert(-1),
            yaw_absolute_pid = Pid::PidRad(config.yaw_absolute_pid_config, c_imu.yaw) >> Pid::Invert(-1));

        pitch_absolute_pid = Pid::PidRad(config.pitch_absolute_pid_config, c_imu.pitch);

        imu.enable();
        c_imu.enable();
        yaw_motor.enable();
        pitch_motor.enable();
    }

    void GimbalT::init_task() {
        while (imu.offline() || yaw_motor.offline() || pitch_motor.offline() || c_imu.offline()) {
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
            LOG_INFO("offline %d %d %d\n", imu.offline(), yaw_motor.offline(), pitch_motor.offline());
        }
        while (robot_set->inited != Types::Init_status::INIT_FINISH) {
        std::cout << "yaw now: " << imu.yaw << ", " << imu.pitch << std::endl;
            update_data();
            if (config.gimbal_id == 2) {
                robot_set->inited |= 1 << 1;
            }

            0.f >> yaw_relative_pid >> yaw_motor;
            0.f >> pitch_absolute_pid >> pitch_motor;
            // LOG_INFO("imu : %6f %6f %6f %6d\n", imu.yaw, imu.pitch, imu.roll, yaw_motor.motor_measure_.ecd);

            if (fabs(yaw_relative) < Config::GIMBAL_INIT_EXP && fabs(c_imu.pitch) < Config::GIMBAL_INIT_EXP) {
                init_stop_times += 1;
            } else {
                init_stop_times = 0;
            }

            MUXDEF(CONFIG_SENTRY, *yaw_set = robot_set->gimbal_sentry_yaw, *yaw_set = c_imu.yaw);
            *pitch_set = 0;

            if (init_stop_times >= Config::GIMBAL_INIT_STOP_TIME) {
                if (config.gimbal_id == 1)
                    robot_set->inited |= 1;
                else
                    robot_set->inited |= 1 << 1;
            }
            UserLib::sleep_ms(config.ControlTime);
        }
    }

#include "deque"

    [[noreturn]] void GimbalT::task() {
        std::jthread shoot_thread(&Shoot::Shoot::task, &shoot);
        std::deque<std::pair<std::chrono::system_clock::time_point, float>> q1, q2;
        int cnt = 0;
        while (true) {
            // auto val = pitch_motor.data_.rotor_angle;
            auto val = c_imu.pitch;
            auto now_time = std::chrono::system_clock::now();
            while (!q1.empty() && q1.back().second > val) {
                q1.pop_back();
            }
            while (!q2.empty() && q2.back().second < val) {
                q2.pop_back();
            }
            q1.emplace_back(now_time, val);
            q2.emplace_back(now_time, val);
            while (!q1.empty() && q1.front().first < now_time - std::chrono::milliseconds(500)) {
                q1.pop_front();
            }
            while (!q2.empty() && q2.front().first < now_time - std::chrono::milliseconds(500)) {
                q2.pop_front();
            }
            std::cout << imu.roll_rate << ' ' << c_imu.roll_rate << std::endl;
            // std::cout << (q2.front().second - q1.front().second) * 100 << std::endl;
            update_data();
            // LOG_INFO("yaw set %f, imu yaw %f\n", *yaw_set, imu.yaw);
            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                yaw_motor.give_current = 0;
                pitch_motor.give_current = 0;
            } else if (robot_set->mode == Types::ROBOT_MODE::ROBOT_SEARCH) {
                static float delta = 0;
                static float delta_1 = 0;

                float yaw = (sin(delta) - 1) * (M_PIf / 2);
                float pitch = sin(delta_1) * 0.30 + 0.165;
                delta += 0.001;
                delta_1 += 0.003;

                if (config.gimbal_id == 1) {
                    yaw >> yaw_relative_pid >> yaw_motor;
                } else {
                    -yaw >> yaw_relative_pid >> yaw_motor;
                }
                *pitch_set = std::clamp((double)pitch, -0.18, 0.51);
                *pitch_set >> pitch_absolute_pid >> pitch_motor;
            } else {
                *yaw_set >> yaw_absolute_pid >> yaw_motor;
                *pitch_set >> pitch_absolute_pid >> pitch_motor;
            }

            if (config.gimbal_id == 1) {
                Robot::SendGimbalInfo gimbal_info;
                gimbal_info.header = 0xA6;
                MUXDEF(CONFIG_SENTRY, gimbal_info.yaw = fake_yaw_abs, gimbal_info.yaw = c_imu.yaw);
                gimbal_info.pitch = c_imu.pitch;
                IO::io<SOCKET>["AUTO_AIM_CONTROL"]->send(gimbal_info);
            }

            UserLib::sleep_ms(config.ControlTime);
        }
    }

    void GimbalT::update_data() {
        yaw_relative =
            UserLib::rad_format(yaw_motor.data_.rotor_angle - Hardware::DJIMotor::ECD_8192_TO_RAD * config.YawOffSet);
        yaw_gyro = (std::cos(c_imu.pitch) * c_imu.yaw_rate - std::sin(c_imu.pitch) * c_imu.roll_rate);
        pitch_gyro = c_imu.pitch_rate;
        // gimbal sentry follow needs
        *yaw_rela = yaw_relative;
        fake_yaw_abs = robot_set->gimbal_sentry_yaw - yaw_relative;
    }

}  // namespace Gimbal
