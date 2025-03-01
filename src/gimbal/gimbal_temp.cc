#include "gimbal/gimbal_temp.hpp"

#include "config.hpp"
#include "gimbal/gimbal_config.hpp"
#include "user_lib.hpp"

namespace Gimbal
{
    GimbalT::GimbalT(const GimbalConfig &config)
        : config(config),
          imu(config.imu_serial_port),
          yaw_motor(config.yaw_motor_config),
          pitch_motor(config.pitch_motor_config),
          yaw_set(nullptr),
          pitch_set(nullptr),
          yaw_rela(nullptr) {
    }

    void GimbalT::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;
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

        // standard && hero
        // yaw_absolute_pid = Pid::PidRad(config.yaw_absolute_pid_config, imu.yaw) >> Pid::Invert(-1);
        // sentry
        yaw_absolute_pid = Pid::PidRad(config.yaw_absolute_pid_config, fake_yaw_abs) >> Pid::Invert(-1);

        pitch_absolute_pid = Pid::PidRad(config.pitch_absolute_pid_config, imu.pitch);

        imu.enable();
        yaw_motor.enable();
        pitch_motor.enable();
    }

    void GimbalT::init_task() {
        while (imu.offline() || yaw_motor.offline() || pitch_motor.offline()) {
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
        while (!inited) {
            update_data();
            0.f >> yaw_relative_pid >> yaw_motor;
            0.f >> pitch_absolute_pid >> pitch_motor;
            // LOG_INFO("yaw offset %d\n",yaw_motor.motor_measure_.ecd);
            if (fabs(yaw_relative) < Config::GIMBAL_INIT_EXP && fabs(imu.pitch) < Config::GIMBAL_INIT_EXP) {
                init_stop_times += 1;
            } else {
                //*yaw_set = imu.yaw;
                *yaw_set = robot_set->gimbal_sentry_yaw;
                *pitch_set = 0;
                init_stop_times = 0;
            }
            inited = init_stop_times >= Config::GIMBAL_INIT_STOP_TIME;
            UserLib::sleep_ms(config.ControlTime);
        }
    }

    [[noreturn]] void GimbalT::task() {
        while (true) {
            update_data();
            LOG_INFO("yaw set %f, imu yaw %f\n", *yaw_set, fake_yaw_abs);
            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                yaw_motor.give_current = 0;
                pitch_motor.give_current = 0;
            } else {
                *yaw_set >> yaw_absolute_pid >> yaw_motor;
                *pitch_set >> pitch_absolute_pid >> pitch_motor;
            }
            UserLib::sleep_ms(config.ControlTime);
        }
    }

    void GimbalT::update_data() {
        yaw_relative =
            UserLib::rad_format(yaw_motor.data_.rotor_angle - Hardware::DJIMotor::ECD_8192_TO_RAD * config.YawOffSet);
        yaw_gyro = std::cos(imu.pitch) * imu.yaw_rate - std::sin(imu.pitch) * imu.roll_rate;
        pitch_gyro = imu.pitch_rate;
        // gimbal sentry follow needs
        *yaw_rela = yaw_relative;
        fake_yaw_abs = robot_set->gimbal_sentry_yaw - yaw_relative;
    }

}  // namespace Gimbal
