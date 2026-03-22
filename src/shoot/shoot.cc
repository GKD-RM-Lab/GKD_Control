#include "shoot.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>

#include "logger.hpp"
#include "macro_helpers.hpp"
#include "pid_controller.hpp"
#include "robot_type_config.hpp"
#include "types.hpp"
#include "user_lib.hpp"
#include "utils.hpp"

namespace Shoot
{
    namespace {
        constexpr uint64_t REFEREE_OFFLINE_TIMEOUT_MS = 300U;
        constexpr uint8_t REF_GAME_TYPE_INFANTRY_DUEL = 5U;
        constexpr float INFANTRY_DUEL_HEAT_BLOCK_RATIO = 0.6f;

        enum class HeatMode : uint8_t
        {
            Unknown = 0,
            Burst = 1,
            Cooling = 2
        };

        HeatMode infer_heat_mode(uint16_t heat_limit, uint16_t cooling_rate) {
            if (heat_limit >= 200U && cooling_rate <= 16U) {
                return HeatMode::Burst;
            }
            if (heat_limit <= 120U && cooling_rate >= 20U) {
                return HeatMode::Cooling;
            }
            return HeatMode::Unknown;
        }

        const char* heat_mode_to_cstr(HeatMode mode) {
            switch (mode) {
                case HeatMode::Burst:
                    return "burst";
                case HeatMode::Cooling:
                    return "cooling";
                default:
                    return "unknown";
            }
        }

        int32_t heat_block_margin(HeatMode mode) {
            switch (mode) {
                case HeatMode::Burst:
                    return 45;
                case HeatMode::Cooling:
                    return 30;
                default:
                    return 40;
            }
        }

        int32_t heat_release_margin(HeatMode mode) {
            switch (mode) {
                case HeatMode::Burst:
                    return 70;
                case HeatMode::Cooling:
                    return 50;
                default:
                    return 60;
            }
        }

        bool is_infantry_duel_mode(const Robot::Robot_set& robot) {
            return robot.referee_info.game_status_data.game_type == REF_GAME_TYPE_INFANTRY_DUEL;
        }
    }  // namespace

    Shoot::Shoot(const ShootConfig& config)
        : friction_ramp(Config::FRICTION_ADD_SPEED, Config::SHOOT_CONTROL_TIME * 1e-3f),
          left_friction(config.left_friction_motor_config),
          right_friction(config.right_friction_motor_config),
          trigger(config.trigger_motor_config),
          gimbal_id(config.gimbal_id) {
        left_friction.setCtrl(
            Pid::PidPosition(
                config.friction_speed_pid_config, left_friction.data_.output_linear_velocity));
        right_friction.setCtrl(
            Pid::PidPosition(
                config.friction_speed_pid_config, right_friction.data_.output_linear_velocity));
        trigger.setCtrl(
            Pid::PidPosition(
                config.trigger_speed_pid_config, trigger.data_.output_angular_velocity));
    }

    void Shoot::init(const std::shared_ptr<Robot::Robot_set>& robot) {
        robot_set = robot;

        left_friction.enable();
        right_friction.enable();
        trigger.enable();
    }

    [[noreturn]] void Shoot::task() {
        static int delta = 0;
        static uint16_t heat_log_div = 0U;
        static bool heat_blocked = false;
        static bool last_shoot_heat = true;
        static HeatMode last_heat_mode = HeatMode::Unknown;
        auto timest = std::chrono::steady_clock::now();
        bool isJamFlag = false;
        while (true) {
            if(!robot_set->referee_info.game_robot_status_data.mains_power_shooter_output) {
                trigger.set_zero();
                left_friction.set_zero();
                right_friction.set_zero();
                if (!friction_ramp.out) {
                    friction_ramp.out = 0;
                }
                robot_set->friction_real_state = false;
                UserLib::sleep_ms(Config::SHOOT_CONTROL_TIME);
                continue;
            }
            // LOG_INFO("%d\n", trigger.motor_measure_.given_current);
            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                left_friction.set(0);
                right_friction.set(0);
                trigger.set(0);
            }
            friction_ramp.update(robot_set->friction_open ? Config::FRICTION_MAX_SPEED : 0.f);

            // friction really open?
            robot_set->friction_real_state =
                left_friction.data_.output_linear_velocity < 0.5 &&
                        right_friction.data_.output_linear_velocity < 0.5
                    ? false
                    : true;
            robot_set->fric_led_open = left_friction.data_.output_linear_velocity < -2.0 && 
                right_friction.data_.output_linear_velocity > 2.0;
            // LOG_INFO(
            //     "ramp %f %f\n", friction_ramp.out, right_friction.data_.output_linear_velocity);

            left_friction.set(-friction_ramp.out);
            right_friction.set(friction_ramp.out);

            // if(left_friction.data_.output_linear_velocity ||
            // right_friction.data_.output_linear_velocity )
            // {
            //     LOG_INFO("set: %f,left: %f, right: %f\n", friction_ramp.out,
            //     left_friction.data_.output_linear_velocity,
            //     right_friction.data_.output_linear_velocity); std::stringstream ss;
            //      ss << "set: " << friction_ramp.out
            //     << ", left: " << left_friction.data_.output_linear_velocity
            //     << ", right: " << right_friction.data_.output_linear_velocity
            //     << "\n";
            //     std::string log_content = ss.str();
            //     logger.into_txt("../../../../log/fric_log.txt", log_content);

            // }
            
            const uint64_t now_ms = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count());
            const bool referee_connected =
                robot_set->referee_last_rx_ms > 0 &&
                now_ms >= robot_set->referee_last_rx_ms &&
                now_ms - robot_set->referee_last_rx_ms <= REFEREE_OFFLINE_TIMEOUT_MS;

            const uint16_t heat_limit =
                robot_set->referee_info.game_robot_status_data.shooter_cooling_limit;
            const uint16_t cooling_rate =
                robot_set->referee_info.game_robot_status_data.shooter_cooling_rate;
            const uint16_t current_heat = MUXDEF(
                CONFIG_HERO,
                robot_set->referee_info.power_heat_data.shooter_id_1_42_mm_cooling_heat,
                robot_set->referee_info.power_heat_data.shooter_id_1_17_mm_cooling_heat);
            const int32_t heat_margin =
                static_cast<int32_t>(heat_limit) - static_cast<int32_t>(current_heat);
            const HeatMode heat_mode = infer_heat_mode(heat_limit, cooling_rate);
            int32_t block_margin = heat_block_margin(heat_mode);
            int32_t release_margin = heat_release_margin(heat_mode);
            const bool infantry_duel_mode = is_infantry_duel_mode(*robot_set);
            if (infantry_duel_mode && heat_limit > 0U) {
                const int32_t duel_margin = std::max(
                    1,
                    static_cast<int32_t>(
                        static_cast<float>(heat_limit) * (1.0f - INFANTRY_DUEL_HEAT_BLOCK_RATIO)));
                block_margin = duel_margin;
                release_margin = duel_margin;
            }

            if (referee_connected && heat_limit > 0U) {
                if (heat_blocked) {
                    if (heat_margin >= release_margin) {
                        heat_blocked = false;
                    }
                } else if (heat_margin <= block_margin) {
                    heat_blocked = true;
                }
            } else {
                heat_blocked = false;
            }
            const bool shoot_heat = !heat_blocked;

            bool remain_bullet = MUXDEF(
                CONFIG_HERO,
                robot_set->referee_info.bullet_allowance_data.bullet_allowance_num_42_mm > 0,
                MUXDEF(
                    CONFIG_INFANTRY,
                    robot_set->referee_info.bullet_allowance_data.bullet_allowance_num_17_mm > 0,
                    robot_set->referee_info.bullet_allowance_data.bullet_allowance_num_17_mm > 0));

            bool referee_fire_allowance = 
                (shoot_heat && remain_bullet) ||
                !((robot_set->referee_info.game_status_data.game_progress & 0x0f) == 4) && 
                (robot_set->auto_aim_status != 1 || robot_set->cv_fire == 1);
            const bool friction_ok = isFrictionOK();

            if (last_shoot_heat != shoot_heat || last_heat_mode != heat_mode) {
                LOG_INFO(
                    "[HEAT_CTRL] allow:%s match:%s mode:%s heat:%u/%u margin:%d blk:%d rel:%d ref:%s\n",
                    shoot_heat ? "on" : "off",
                    infantry_duel_mode ? "1v1" : "3v3",
                    heat_mode_to_cstr(heat_mode),
                    current_heat,
                    heat_limit,
                    static_cast<int>(heat_margin),
                    static_cast<int>(block_margin),
                    static_cast<int>(release_margin),
                    referee_connected ? "on" : "off");
                last_shoot_heat = shoot_heat;
                last_heat_mode = heat_mode;
            }

            if (++heat_log_div >= 100U) {
                heat_log_div = 0U;
                const uint16_t remain_bullets = MUXDEF(
                    CONFIG_HERO,
                    robot_set->referee_info.bullet_allowance_data.bullet_allowance_num_42_mm,
                    robot_set->referee_info.bullet_allowance_data.bullet_allowance_num_17_mm);

                // LOG_INFO(
                //     "[HEAT_MON] ref:%s mode:%s heat:%u/%u margin:%d cool:%u bullet:%u allow:%s prog:%u fric:%s fric_ok:%s shoot:%s no_force:%s\n",
                //     referee_connected ? "on" : "off",
                //     heat_mode_to_cstr(heat_mode),
                //     current_heat,
                //     heat_limit,
                //     static_cast<int>(heat_margin),
                //     cooling_rate,
                //     remain_bullets,
                //     referee_fire_allowance ? "on" : "off",
                //     static_cast<unsigned>(robot_set->referee_info.game_status_data.game_progress & 0x0FU),
                //     robot_set->friction_real_state ? "on" : "off",
                //     friction_ok ? "on" : "off",
                //     (robot_set->shoot_open & gimbal_id) ? "on" : "off",
                //     robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE ? "on" : "off");
            }

            // LOG_INFO(
            //     "referee fire allowance %d %d %d %d %d\n",
            //     referee_fire_allowance,
            //     remain_bullet,
            //     shoot_heat,
            //     robot_set->referee_info.power_heat_data.shooter_id_1_17_mm_cooling_heat,
            //     robot_set->referee_info.game_robot_status_data.shooter_cooling_limit);

            // if(robot_set->shoot_open)
            // {
            //     LOG_INFO("set: %f,left: %f, right: %f\n", friction_ramp.out,
            //     left_friction.data_.output_linear_velocity,
            //     right_friction.data_.output_linear_velocity); std::stringstream ss; ss << "set: "
            //     << Config::CONTINUE_TRIGGER_SPEED
            //     << ", trigger: " << trigger.data_.output_angular_velocity
            //     << "\n";
            //     std::string log_content = ss.str();
            //     logger.into_txt("../../../../log/trigger_log.txt", log_content);
            // }

            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE ||
                !(robot_set->shoot_open & gimbal_id) || !referee_fire_allowance ||
                !robot_set->friction_real_state || !friction_ok) {
                trigger.set_zero();
            } else {
                if (isJamFlag) {
                    LOG_INFO("%d\n", trigger.motor_measure_.given_current);
                    LOG_INFO("jam%d\n", delta++);
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - timest)
                            .count() > 50) {
                        isJamFlag = false;
                    }
                } else if (isJam()) {
                    trigger.set_zero();
                    LOG_INFO("%d\n", trigger.motor_measure_.given_current);
                    LOG_INFO("jam%d\n", delta++);

                    isJamFlag = true;
                    timest = std::chrono::steady_clock::now();

                } else {
                    trigger.set(Config::CONTINUE_TRIGGER_SPEED);
                }
            }
            // LOG_INFO("friic velo:%f\n", right_friction.data_.output_linear_velocity);
            UserLib::sleep_ms(Config::SHOOT_CONTROL_TIME);
        }
    }

    bool Shoot::isJam() {
        return trigger.motor_measure_.given_current > 4000 && trigger.motor_measure_.speed_rpm < 1;
    }

    bool Shoot::isFrictionOK() {
        return std::abs(left_friction.data_.output_linear_velocity) > 1.7 &&
               std::abs(right_friction.data_.output_linear_velocity) > 1.7;
    }

}  // namespace Shoot
