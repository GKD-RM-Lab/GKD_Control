#include "shoot.hpp"
#include "hardware.hpp"

namespace Shoot
{

    Shoot::Shoot() : friction_ramp(Config::FRICTION_ADD_SPEED, Config::SHOOT_CONTROL_TIME * 1e-3f) {
        friction.assign(Config::FRICTION_NUM, Hardware::Motor{ Config::FRIC_SPEED_PID_CONFIG });
        trigger.assign(Config::TRIGGER_NUM, Hardware::Motor{ Config::TRIGGER_SPEED_PID_CONFIG });
    }

    void Shoot::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;
        Robot::hardware->register_callback<CAN0>(0x201, [&](const auto &frame) { friction[0].unpack(frame); });
        Robot::hardware->register_callback<CAN0>(0x202, [&](const auto &frame) { friction[1].unpack(frame); });
        Robot::hardware->register_callback<CAN1>(0x201, [&](const auto &frame) { friction[2].unpack(frame); });
        Robot::hardware->register_callback<CAN1>(0x202, [&](const auto &frame) { friction[3].unpack(frame); });
        Robot::hardware->register_callback<CAN3>(0x201, [&](const auto &frame) { trigger[0].unpack(frame); });
        Robot::hardware->register_callback<CAN3>(0x202, [&](const auto &frame) { trigger[1].unpack(frame); });
    }

    void Shoot::update_speed() {
        for (auto &m : friction) {
            m.speed = m.motor_measure.speed_rpm;
        }
        for (auto &m : trigger) {
            m.speed = Config::SHOOT_MOTOR_RPM_TO_SPEED * (fp32)m.motor_measure.speed_rpm;
        }
    }

    void Shoot::decomposition_speed() {
        fp32 trigger_speed = 0.f;
        if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
            friction_ramp.clear();
            trigger_speed = 0.f;
        } else {
            friction_ramp.update(robot_set->friction_open ? Config::FRICTION_MAX_SPEED : 0.f);
            if(back_time) {
                back_time--;
                trigger_speed = robot_set->shoot_open ? -Config::CONTINUE_TRIGGER_SPEED : 0.f;
            }
            else {
                if(isJam()) {
                    jam_time++;
                }
                if(jam_time > 500) {
                    back_time = 1000;
                    jam_time = 0;
                }
                trigger_speed = trigger[1].speed_set = robot_set->shoot_open ? Config::CONTINUE_TRIGGER_SPEED : 0.f;
            }
        }

        friction[0].speed_set = -friction_ramp.out;
        friction[1].speed_set = friction_ramp.out;
        friction[2].speed_set = -friction_ramp.out;
        friction[3].speed_set = friction_ramp.out;
        for(auto & mot : trigger) {
            mot.speed_set = trigger_speed;
        }
    }

    [[noreturn]] void Shoot::task() {
        while(true) {
            update_speed();
            decomposition_speed();
            if(robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                for(auto & mot : friction) {
                    mot.give_current = 0;
                }
            } else {
                for (auto &mot : friction) {
                    mot.pid_ctrler.calc(mot.speed, mot.speed_set);
                    mot.give_current = (int16_t)mot.pid_ctrler.out;
                }
								printf("\n");
            }
            if(robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE || !robot_set->shoot_open) {
                for(auto & mot : trigger) {
                    mot.give_current = 0;
                }
            }
            else {
                for (auto &mot : trigger) {
                    mot.pid_ctrler.calc(mot.speed, mot.speed_set);
                    mot.give_current = (int16_t)mot.pid_ctrler.out;
                }
            }

						Robot::hardware->send<CAN0>(Hardware::get_frame(0x200, friction[0], friction[1]));
						Robot::hardware->send<CAN1>(Hardware::get_frame(0x200, friction[2], friction[3]));
						Robot::hardware->send<CAN3>(Hardware::get_frame(0x200, trigger));
            UserLib::sleep_ms(Config::SHOOT_CONTROL_TIME);
        }
    }

    bool Shoot::isJam() {
        return (trigger[0].give_current > 4000) && (trigger[1].give_current > 4000) &&
               (trigger[0].speed < 1.f) && (trigger[1].speed < 1.f);
    }
}  // namespace Shoot
