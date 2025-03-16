#include "shoot.hpp"

#include "pid_controller.hpp"
#include "robot_type_config.hpp"
#include "user_lib.hpp"
#include "utils.hpp"

namespace Shoot
{

    Shoot::Shoot(const ShootConfig& config)
        : friction_ramp(Config::FRICTION_ADD_SPEED, Config::SHOOT_CONTROL_TIME * 1e-3f),
          left_friction(config.left_friction_motor_config),
          right_friction(config.right_friction_motor_config),
          trigger(config.trigger_motor_config) {
        left_friction.setCtrl(
            Pid::PidPosition(config.friction_speed_pid_config, left_friction.data_.output_linear_velocity));
        right_friction.setCtrl(
            Pid::PidPosition(config.friction_speed_pid_config, right_friction.data_.output_linear_velocity));
        trigger.setCtrl(Pid::PidPosition(config.trigger_speed_pid_config, trigger.data_.output_angular_velocity));
    }

    void Shoot::init(const std::shared_ptr<Robot::Robot_set>& robot) {
        robot_set = robot;

        left_friction.enable();
        right_friction.enable();
        trigger.enable();
    }

    [[noreturn]] void Shoot::task() {
        while (true) {
            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                left_friction.set(0);
                right_friction.set(0);
                trigger.set(0);
            }

            friction_ramp.update(robot_set->friction_open ? Config::FRICTION_MAX_SPEED : 0.f);

            // LOG_INFO("ramp %f %f\n", friction_ramp.out, left_friction.data_.output_linear_velocity);
            left_friction.set(-friction_ramp.out);
            right_friction.set(friction_ramp.out);

            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE || !robot_set->shoot_open) {
                trigger.set(0);
            } else {
                trigger.set(7);
            }
            UserLib::sleep_ms(Config::SHOOT_CONTROL_TIME);
        }
    }

    bool Shoot::isJam() {
        return trigger.motor_measure_.given_current > 4000 && trigger.motor_measure_.speed_rpm < 1;
    }
}  // namespace Shoot
