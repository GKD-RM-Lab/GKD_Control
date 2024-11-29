#include "gimbal/gimbal_config.hpp"

#include "gimbal/gimbal_temp.hpp"
#include "config.hpp"

#include "user_lib.hpp"

namespace Gimbal
{
    GimbalT::GimbalT()
        : yaw_motor(6020, "CAN_RIGHT_HEAD", 2), pitch_motor(6020, "CAN_RIGHT_HEAD", 1),
            imu("/dev/IMU_RIGHT") {
    }

    void GimbalT::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;
        yaw_relative_pid = Pid::PidRad(Config::GIMBAL_YAW_RELATIVE_PID_CONFIG, yaw_relative);
        pitch_absolute_pid = Pid::PidRad(Config::GIMBAL_PITCH_ABSOLUTE_PID_CONFIG, imu.pitch);
        yaw_absolute_pid = Pid::PidRad(Config::GIMBAL_YAW_ABSOLUTE_PID_CONFIG, imu.yaw);

        yaw_motor.setCtrl(Pid::PidPosition(Config::YAW_SPEED_PID_CONFIG, yaw_gyro));
        pitch_motor.setCtrl(Pid::PidPosition(Config::PITCH_SPEED_PID_CONFIG, pitch_gyro));

        imu.enable();
        yaw_motor.enable();
        pitch_motor.enable();
    }

    void GimbalT::init_task() {
        while(imu.offline() || yaw_motor.offline() || pitch_motor.offline()) {
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
        while (!inited) {
            update_data();
            0.f >> yaw_relative_pid >> yaw_motor;
            0.f >> pitch_absolute_pid >> pitch_motor;
            if (fabs(yaw_relative) < Config::GIMBAL_INIT_EXP &&
                fabs(imu.pitch) < Config::GIMBAL_INIT_EXP) {
                init_stop_times += 1;
            } else {
                robot_set->gimbal1_yaw_set = robot_set->gimbal1_yaw_relative;
                init_stop_times = 0;
            }
            inited = init_stop_times >= Config::GIMBAL_INIT_STOP_TIME;
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
        robot_set->gimbal1_yaw_set = imu.yaw;
    }

    [[noreturn]] void GimbalT::task() {
        while (true) {
            update_data();
            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                yaw_motor.give_current = 0;
                pitch_motor.give_current = 0;
            } else {
                ((robot_set->gimbal1_yaw_set >> yaw_absolute_pid) * -1) >> yaw_motor;
                robot_set->gimbal1_pitch_set >> pitch_absolute_pid >> pitch_motor;
            }
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
    }

    void GimbalT::update_data() {
        yaw_relative = UserLib::rad_format(
            yaw_motor.data_.rotor_angle - Config::M6020_ECD_TO_RAD * (Config::GIMBAL1_YAW_OFFSET_ECD));
        yaw_gyro = -std::sin(imu.pitch) * imu.roll_rate +
                   std::cos(imu.pitch) * imu.yaw_rate;
        pitch_gyro = imu.pitch_rate;
    }

}  // namespace Gimbal
