#include "chassis/swerve.hpp"
#include <cmath>
#include "hardware.hpp"

namespace Chassis
{
    Swerve::Swerve(const SwerveConfig &config) : chassis_angle_pid(config.follow_gimbal_pid_config) {
        speed_motors_.assign(4, Hardware::Motor{ config.speed_pid_config });
        turn_motors_.assign(4, Hardware::Motor{ config.turn_pid_config });
        turn_init_ecd_ = config.turn_init_ecd;
        for (int i = 0; i < 4; i++) {
            turn_pid_.push_back(Pid::Pid_rad(config.turn_pos_pid_config));
        }
    }

    void Swerve::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set_ = robot;
        for (size_t i = 0; i < speed_motors_.size(); i++) {
            auto &mot = speed_motors_[i];
            Robot::hardware->register_callback<CAN0>(0x201 + i, [&mot](const auto &frame) { mot.unpack(frame); });
        }
        for (size_t i = 0; i < turn_motors_.size(); i++) {
            auto &mot = turn_motors_[i];
            Robot::hardware->register_callback<CAN1>(0x205 + i, [&mot](const auto &frame) { mot.unpack(frame); });
        }
    }

    void Swerve::init_task() {
        bool init_finish = false;
        UserLib::sleep_ms(Config::CHASSIS_CONTROL_TIME);
        update_data();
        while (!init_finish) {
            while (turn_angle_[0] > 0.1 || turn_angle_[1] > 0.1 || turn_angle_[2] > 0.1 || turn_angle_[3] > 0.1) {
                LOG_INFO(
                    "wheel1: %f, wheel2: %f, wheel3: %f, wheel4: %f",
                    turn_angle_[0],
                    turn_angle_[1],
                    turn_angle_[2],
                    turn_angle_[3]);
                update_data();
                for (int i = 0; i < 4; i++) {
                    auto &mot = turn_motors_[i];
                    float mot_rad = UserLib::rad_format(Config::M6020_ECD_TO_RAD * (fp32)mot.motor_measure.ecd);
                    float turn_rad = UserLib::rad_format(Config::M6020_ECD_TO_RAD * (fp32)turn_init_ecd_[i]);
                    turn_pid_[i].calc(mot_rad, turn_rad);
                    mot.speed_set = turn_pid_[i].out;
                    mot.pid_ctrler.calc(mot.speed, mot.speed_set);
                    mot.give_current = (int16_t)mot.pid_ctrler.out;
                }
                Robot::hardware->send<CAN1>(Hardware::get_frame(0x1FF, turn_motors_));
                UserLib::sleep_ms(Config::CHASSIS_CONTROL_TIME);
            }
            init_finish = true;
        }
    }

    [[noreturn]] void Swerve::task() {
        while (true) {
            update_data();
            decomposition_speed();
            if (robot_set_->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                for (int i = 0; i < 4; i++) {
                    speed_motors_[i].give_current = 0;
                    turn_motors_[i].give_current = 0;
                }
            } else {
                for (auto &m : speed_motors_) {
                    m.pid_ctrler.calc(m.speed, m.speed_set);
                    m.give_current = (int16_t)(m.pid_ctrler.out);
                }
                for (int i = 0; i < 4; i++) {
                    auto &mot = turn_motors_[i];
                    turn_pid_[i].calc(robot_set_->ins_pitch, turn_angle_set_[i]);
                    mot.speed_set = turn_pid_[i].out;
                    mot.pid_ctrler.calc(mot.speed, mot.speed_set);
                    mot.give_current = (int16_t)-mot.pid_ctrler.out;
                }
            }
            Robot::hardware->send<CAN0>(Hardware::get_frame(0x200, speed_motors_));
            Robot::hardware->send<CAN1>(Hardware::get_frame(0x1FF, turn_motors_));
            UserLib::sleep_ms(Config::CHASSIS_CONTROL_TIME);
        }
    }

    void Swerve::decomposition_speed() {
        if (robot_set_->mode != Types::ROBOT_MODE::ROBOT_NO_FORCE) {
            fp32 sin_yaw, cos_yaw;
            sincosf(robot_set_->yaw_relative, &sin_yaw, &cos_yaw);
            vx_set_ = cos_yaw * robot_set_->vx_set - sin_yaw * robot_set_->vy_set;
            vy_set_ = sin_yaw * robot_set_->vx_set + cos_yaw * robot_set_->vy_set;

            if (robot_set_->mode == Types::ROBOT_MODE::ROBOT_FOLLOW_GIMBAL) {
                chassis_angle_pid.calc(robot_set_->yaw_relative, 0.f);
                wz_set_ = chassis_angle_pid.out;
            } else {
                wz_set_ = robot_set_->wz_set;
            }
        }
        for (int i = 0; i < 4; i++) {
            set_speed(i, { wz_set_ * turn_norm_vector_[i].x + vx_set_, wz_set_ * turn_norm_vector_[i].y + vy_set_ });
        }
    }

    fp32 Swerve::get_turn_cost(fp32 angle_change, fp32 speed_change) {
        return angle_change > 3*M_PIf/4 ? 1: 0;
    }

    void Swerve::set_speed(uint32_t idx, const Vec2f &speed) {
        auto w = std::atan2(speed.y, speed.x);
        auto v = std::sqrt(speed.x * speed.x + speed.y * speed.y);
        if (get_turn_cost(UserLib::rad_format(turn_angle_[idx] - w), speed_motors_[idx].speed - v) <
            get_turn_cost(UserLib::rad_format(turn_angle_[idx] - w + M_PIf), speed_motors_[idx].speed + v)) {
            turn_angle_set_[idx] = w;
            speed_motors_[idx].speed_set = v;
        } else {
            turn_angle_set_[idx] = UserLib::rad_format(w + M_PIf);
            speed_motors_[idx].speed_set = -v;
        }
    }

    void Swerve::update_data() {
        for (auto &m : speed_motors_) {
            m.speed = Config::CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * (fp32)m.motor_measure.speed_rpm;
            m.accel = Config::CHASSIS_CONTROL_FREQUENCE * m.pid_ctrler.Dbuf;
        }
        for (int i = 0; i < 4; i++) {
            auto &mot = turn_motors_[i];
            turn_angle_[i] = Config::M6020_ECD_TO_RAD * (fp32)(mot.motor_measure.ecd - turn_init_ecd_[i]);
            mot.speed = Config::RPM_TO_RAD_S * (fp32)mot.motor_measure.speed_rpm;
        }
    }
}  // namespace Chassis
