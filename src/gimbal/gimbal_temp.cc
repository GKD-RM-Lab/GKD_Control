#include "gimbal/gimbal_config.hpp"

#include "gimbal/gimbal_temp.hpp"
#include "config.hpp"

#include "user_lib.hpp"

namespace Gimbal
{
    Gimbal::Gimbal()
        : yaw_motor(6020, "CAN_RIGHT_HEAD", 2),
          pitch_motor(6020, "CAN_RIGHT_HEAD", 1) {
    }

    void Gimbal::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;
        yaw_motor.setCtrl(
            Pid::PidRad(Config::GIMBAL_YAW_RELATIVE_PID_CONFIG, robot_set->gimbal1_yaw_relative) >>
            Pid::PidPosition(Config::YAW_SPEED_PID_CONFIG, yaw_gyro));

        pitch_motor.setCtrl(
            Pid::PidRad(Config::GIMBAL_PITCH_ABSOLUTE_PID_CONFIG, robot_set->gyro1_ins_pitch) >>
            Pid::PidPosition(Config::PITCH_SPEED_PID_CONFIG, pitch_gyro));

        yaw_motor.enable();
        pitch_motor.enable();
    }

    void Gimbal::init_task() {
        update_data();
        init_yaw_set = robot_set->gimbal1_yaw_relative;
        init_pitch_set = robot_set->gyro1_ins_pitch;

        while (!inited) {
            update_data();
            init_yaw_set += UserLib::rad_format(0.f - robot_set->gimbal1_yaw_relative) * Config::GIMBAL_INIT_YAW_SPEED;
            init_pitch_set += UserLib::rad_format(0.f - robot_set->gyro1_ins_pitch) * Config::GIMBAL_INIT_PITCH_SPEED;
            yaw_motor.set(init_yaw_set);
            pitch_motor.set(init_pitch_set);

            if (fabs(robot_set->gimbal1_yaw_relative) < Config::GIMBAL_INIT_EXP &&
                fabs(robot_set->gyro1_ins_pitch) < Config::GIMBAL_INIT_EXP) {
                init_stop_times += 1;
            } else {
                robot_set->gimbal1_yaw_set = robot_set->gimbal1_yaw_relative;
                init_stop_times = 0;
            }
            inited = init_stop_times >= Config::GIMBAL_INIT_STOP_TIME;
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
    }

    [[noreturn]] void Gimbal::task() {
        while (true) {
            update_data();
            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                yaw_motor.give_current = 0.f;
                pitch_motor.give_current = 0.f;
            } else {
                yaw_motor.set(robot_set->gimbal1_yaw_set);
                pitch_motor.set(robot_set->gimbal1_pitch_set);
            }
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
    }

    void Gimbal::update_data() {
        robot_set->gimbal1_yaw_relative = UserLib::rad_format(
            Config::M6020_ECD_TO_RAD * ((fp32)yaw_motor.motor_measure_.ecd - Config::GIMBAL1_YAW_OFFSET_ECD));
        // robot_set->gimbal1_pitch_relative = UserLib::rad_format(
        //     Config::M6020_ECD_TO_RAD * -((fp32)pitch_motor.motor_measure.ecd - Config::GIMBAL1_PITCH_OFFSET_ECD));

        yaw_gyro = -std::sin(robot_set->gimbal1_pitch_relative) * robot_set->gyro1_ins_roll_v +
                   std::cos(robot_set->gimbal1_pitch_relative) * robot_set->gyro1_ins_yaw_v;
        pitch_gyro = robot_set->gyro1_ins_pitch_v;
    }

}  // namespace Gimbal
