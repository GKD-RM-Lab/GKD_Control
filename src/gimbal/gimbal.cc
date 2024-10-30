#include "gimbal.hpp"

namespace Gimbal
{

    Gimbal::Gimbal()
        : yaw_motor(Config::YAW_SPEED_PID_CONFIG),
          pitch_motor(Config::PITCH_SPEED_PID_CONFIG),
          yaw_absolute_pid(Config::GIMBAL_YAW_ABSOLUTE_PID_CONFIG),
          pitch_absolute_pid(Config::GIMBAL_PITCH_ABSOLUTE_PID_CONFIG),
          yaw_relative_pid(Config::GIMBAL_YAW_RELATIVE_PID_CONFIG) {
    }

    void Gimbal::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;

        Robot::hardware->register_callback<CAN1>(0x206, [&](const auto &frame) { yaw_motor.unpack(frame); });
        Robot::hardware->register_callback<CAN1>(0x205, [&](const auto &frame) { pitch_motor.unpack(frame); });
    }

    void Gimbal::init_task() {
        while (!inited) {
            update_data();
            init_yaw_set = 0;
            init_pitch_set = 0;

            yaw_relative_pid.calc(robot_set->gimbal1_yaw_relative, init_yaw_set);
            yaw_motor.speed_set = yaw_relative_pid.out;
            yaw_motor.pid_ctrler.calc(-yaw_gyro, yaw_motor.speed_set);
            yaw_motor.give_current = (int16_t)yaw_motor.pid_ctrler.out;

            pitch_absolute_pid.calc(robot_set->gyro1_ins_pitch, init_pitch_set);
            pitch_motor.speed_set = pitch_absolute_pid.out;
            pitch_motor.pid_ctrler.calc(-pitch_gyro, pitch_motor.speed_set);
            pitch_motor.give_current = (int16_t)-pitch_motor.pid_ctrler.out;

            robot_set->gimbal1_yaw_set = robot_set->gyro1_ins_yaw;
            robot_set->gimbal1_yaw_offset = robot_set->gyro1_ins_yaw;
            if (fabs(robot_set->gimbal1_yaw_relative) < Config::GIMBAL_INIT_EXP &&
                fabs(robot_set->gyro1_ins_pitch - init_pitch_set) < Config::GIMBAL_INIT_EXP) {
                init_stop_times += 1;
            } else if (init_stop_times > Config::GIMBAL_INIT_STOP_TIME) {
                robot_set->gimbal1_pitch_set = 0;
                init_stop_times = 0;
            }

            inited = init_stop_times >= Config::GIMBAL_INIT_STOP_TIME;
            Robot::hardware->send<CAN1>(Hardware::get_frame(0x1FF, pitch_motor, yaw_motor));
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
        LOG_INFO("gimbal1 init finished\n");
    }

    [[noreturn]] void Gimbal::task() {
        while (true) {
            update_data();
            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                yaw_motor.give_current = 0.f;
                pitch_motor.give_current = 0.f;
            } else if (robot_set->mode == Types::ROBOT_MODE::ROBOT_FINISH_INIT) {
                static fp32 delta1 = M_PIf / 4;
                float p = .0;
                // 仰角9.57 俯角27.42
                p = robot_set->gimbal1_pitch_set +
                    (std::sin(delta1) * 0.10550000000000002 - 0.08233333333333336) * M_PIf;
                delta1 += 0.008;

                if (robot_set->gimbal1_yaw_relative < -(M_PIf * 7 / 8)) {
                    robot_set->set_mode(Types::ROBOT_MODE::ROBOT_IDLE);
                    continue;
                }
                yaw_relative_pid.calc(robot_set->gimbal1_yaw_relative, robot_set->gimbal1_yaw_relative - 0.1);
                yaw_motor.speed_set = yaw_relative_pid.out;
                yaw_motor.pid_ctrler.calc(-yaw_gyro, yaw_motor.speed_set);
                yaw_motor.give_current = (int16_t)yaw_motor.pid_ctrler.out;

                pitch_absolute_pid.calc(robot_set->gyro1_ins_pitch, p);
                pitch_motor.speed_set = pitch_absolute_pid.out;
                pitch_motor.pid_ctrler.calc(-pitch_gyro, pitch_motor.speed_set);
                pitch_motor.give_current = (int16_t)-pitch_motor.pid_ctrler.out;

            } else if (robot_set->mode == Types::ROBOT_MODE::ROBOT_IDLE) {
                yaw_absolute_pid.calc(robot_set->gyro1_ins_yaw, robot_set->gimbal1_yaw_set);
                yaw_motor.speed_set = yaw_absolute_pid.out;
                yaw_motor.pid_ctrler.calc(yaw_gyro, yaw_motor.speed_set);
                yaw_motor.give_current = -(int16_t)yaw_motor.pid_ctrler.out;

                pitch_absolute_pid.calc(robot_set->gyro1_ins_pitch, robot_set->gimbal1_pitch_set);
                pitch_motor.speed_set = pitch_absolute_pid.out;
                pitch_motor.pid_ctrler.calc(-pitch_gyro, pitch_motor.speed_set);
                pitch_motor.give_current = (int16_t)-pitch_motor.pid_ctrler.out;
            } else if (robot_set->mode == Types::ROBOT_MODE::ROBOT_SEARCH) {
                static fp32 delta1 = M_PIf / 4;
                static int8_t dir = -2;
                float p = .0;
                // 仰角9.57 俯角27.42
                p = robot_set->gimbal1_pitch_set +
                    (std::sin(delta1) * 0.10550000000000002 - 0.08233333333333336) * M_PIf;
                delta1 += 0.008;

                // LOG_INFO("big yaw %f %f\n", robot_set->gimbal1_yaw_relative, delta);
                if (robot_set->gimbal1_yaw_relative < -(M_PIf * 8 / 9) ||
                    robot_set->gimbal1_yaw_relative > (M_PIf * 1 / 2)) {
                    dir = 2;
                } else if (
                    robot_set->gimbal1_yaw_relative > (M_PIf * 1 / 8) &&
                    robot_set->gimbal1_yaw_relative < (M_PIf * 1 / 2)) {
                    dir = -2;
                }

                yaw_relative_pid.calc(robot_set->gimbal1_yaw_relative, robot_set->gimbal1_yaw_relative + 0.1 * dir);
                yaw_motor.speed_set = yaw_relative_pid.out;
                yaw_motor.pid_ctrler.calc(-yaw_gyro, yaw_motor.speed_set);
                yaw_motor.give_current = (int16_t)yaw_motor.pid_ctrler.out;

                pitch_absolute_pid.calc(robot_set->gyro1_ins_pitch, p);
                pitch_motor.speed_set = pitch_absolute_pid.out;
                pitch_motor.pid_ctrler.calc(-pitch_gyro, pitch_motor.speed_set);
                pitch_motor.give_current = (int16_t)-pitch_motor.pid_ctrler.out;

            } else {  // ROBOT_FOLLOW_GIMBAL
                if (robot_set->mode_changed()) {
                    robot_set->sync_head();
                }
                yaw_absolute_pid.calc(robot_set->gyro1_ins_yaw, robot_set->gimbal1_yaw_set);
                yaw_motor.speed_set = yaw_absolute_pid.out;
                yaw_motor.pid_ctrler.calc(yaw_gyro, yaw_motor.speed_set);
                yaw_motor.give_current = -(int16_t)yaw_motor.pid_ctrler.out;

                pitch_absolute_pid.calc(robot_set->gyro1_ins_pitch, robot_set->gimbal1_pitch_set);
                pitch_motor.speed_set = pitch_absolute_pid.out;
                pitch_motor.pid_ctrler.calc(-pitch_gyro, pitch_motor.speed_set);
                pitch_motor.give_current = (int16_t)-pitch_motor.pid_ctrler.out;
            }
            Robot::hardware->send<CAN1>(Hardware::get_frame(0x1FF, pitch_motor, yaw_motor));
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
    }

    void Gimbal::update_data() {
        yaw_motor.speed = Config::RPM_TO_RAD_S * (fp32)yaw_motor.motor_measure.speed_rpm;
        pitch_motor.speed = Config::RPM_TO_RAD_S * (fp32)pitch_motor.motor_measure.speed_rpm;

        robot_set->gimbal1_yaw_relative = UserLib::rad_format(
            Config::M6020_ECD_TO_RAD * ((fp32)yaw_motor.motor_measure.ecd - Config::GIMBAL1_YAW_OFFSET_ECD));
        robot_set->gimbal1_pitch_relative = UserLib::rad_format(
            Config::M6020_ECD_TO_RAD * -((fp32)pitch_motor.motor_measure.ecd - Config::GIMBAL1_PITCH_OFFSET_ECD));

        yaw_gyro = std::sin(robot_set->gimbal1_pitch_relative) * robot_set->gyro1_ins_roll_v -
                   std::cos(robot_set->gimbal1_pitch_relative) * robot_set->gyro1_ins_yaw_v;
        pitch_gyro = robot_set->gyro1_ins_pitch_v;
    }

}  // namespace Gimbal
