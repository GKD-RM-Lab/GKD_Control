#include "config.hpp"
#include "gimbal/gimbal.hpp"
#include "user_lib.hpp"

namespace Gimbal
{
    Gimbal_big_yaw::Gimbal_big_yaw()
    // :yaw_motor(Config::YAW_9025_SPEED_PID_CONFIG),
    // yaw_absolute_pid(Config::GIMBAL_9025_YAW_ABSOLUTE_PID_CONFIG),
    // yaw_relative_pid(Config::GIMBAL_9025_YAW_RELATIVE_PID_CONFIG)
    {
    }

    void Gimbal_big_yaw::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;
        // Robot::hardware->register_callback<CAN2>(0x141, [&](const auto &frame) {
        //     // LOG_INFO("9025 %d\n", yaw_motor.motor_measure.ecd);
        //     yaw_motor.unpack(frame);
        // });
    }

    void Gimbal_big_yaw::init_task() {
        while (!inited) {
            update_data();
            // yaw_relative_pid.calc(robot_set->gimbal3_yaw_relative, 0);
            // yaw_motor.speed_set = yaw_relative_pid.out;
            // yaw_motor.pid_ctrler.calc(yaw_gyro, yaw_motor.speed_set);
            // yaw_motor.give_current = (int16_t)yaw_motor.pid_ctrler.out;

            // LOG_INFO(
            //     "big yaw %d %f %f %f\n",
            //     yaw_motor.motor_measure.ecd,
            //     robot_set->gimbal3_yaw_relative,
            //     yaw_gyro,
            //     yaw_motor.speed_set);

            robot_set->gimbal3_yaw_set = robot_set->gyro3_ins_yaw;
            if (fabs(robot_set->gimbal3_yaw_relative) < Config::GIMBAL_INIT_EXP) {
                init_stop_times += 1;
            }

            inited = init_stop_times >= Config::GIMBAL_INIT_STOP_TIME;
            // Robot::hardware->send<CAN2>(Hardware::get_frame(0x141, yaw_motor));
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
    }

    [[noreturn]] void Gimbal_big_yaw::task() {
        while (true) {
            update_data();
            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                // yaw_motor.give_current = 0.f;
            } else if (
                robot_set->mode == Types::ROBOT_MODE::ROBOT_FINISH_INIT ||
                robot_set->mode == Types::ROBOT_MODE::ROBOT_IDLE ||
                robot_set->mode == Types::ROBOT_MODE::ROBOT_SEARCH) {
                if (robot_set->mode_changed()) {
                    robot_set->gimbal3_yaw_set = robot_set->gyro3_ins_yaw;
                }
                // yaw_absolute_pid.calc(robot_set->gyro3_ins_yaw, robot_set->gimbal3_yaw_set);
                // yaw_motor.speed_set = yaw_absolute_pid.out;
                // yaw_motor.pid_ctrler.calc(-yaw_gyro, yaw_motor.speed_set);
                // yaw_motor.give_current = -(int16_t)yaw_motor.pid_ctrler.out;
            } else {
                // yaw_relative_pid.calc(robot_set->gimbal1_yaw_relative, 0);
                // yaw_motor.speed_set = yaw_relative_pid.out;
                // yaw_relative_pid.calc(robot_set->gimbal2_yaw_relative, 0);
                // yaw_motor.speed_set += yaw_relative_pid.out;
                // yaw_motor.pid_ctrler.calc(-yaw_gyro, yaw_motor.speed_set);
                // yaw_motor.give_current = -(int16_t)yaw_motor.pid_ctrler.out;
            }
            // Robot::hardware->send<CAN2>(Hardware::get_frame(0x141, yaw_motor));
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
    }

    void Gimbal_big_yaw::update_data() {
        // yaw_motor.speed = Config::RPM_TO_RAD_S * (fp32)yaw_motor.motor_measure.speed_rpm;

        // robot_set->gimbal3_yaw_relative = UserLib::rad_format(
        //     Config::M9025_ECD_TO_RAD * ((fp32)yaw_motor.motor_measure.ecd - Config::GIMBAL3_YAW_OFFSET_ECD));

        yaw_gyro = robot_set->gyro3_ins_yaw_v;
    }
}  // namespace Gimbal
