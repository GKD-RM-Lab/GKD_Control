#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>

#include "power_controller.hpp"

namespace Power::detail
{
    inline bool float_equal(float a, float b) {
        return fabs(a - b) < 1e-5f;
    }

    inline float rpm_to_angular_velocity(float rpm) {
        return rpm * static_cast<float>(M_PI) / 30.0f;
    }

    inline void set_error_flag(uint8_t &curFlag, Manager::ErrorFlags setFlag) {
        curFlag |= static_cast<uint8_t>(setFlag);
    }

    inline void clear_error_flag(uint8_t &curFlag, Manager::ErrorFlags clearFlag) {
        curFlag &= (~static_cast<uint8_t>(clearFlag));
    }

    inline uint64_t now_ms() {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
    }

    inline bool is_cap_connected(
        const Manager &manager,
        uint64_t nowMs,
        uint64_t timeoutMs) {
        return manager.robot_set->super_cap_last_rx_ms > 0 &&
               nowMs >= manager.robot_set->super_cap_last_rx_ms &&
               nowMs - manager.robot_set->super_cap_last_rx_ms <= timeoutMs;
    }

    inline bool is_referee_connected(
        const Manager &manager,
        uint64_t nowMs,
        uint64_t timeoutMs) {
        return manager.robot_set->referee_last_rx_ms > 0 &&
               nowMs >= manager.robot_set->referee_last_rx_ms &&
               nowMs - manager.robot_set->referee_last_rx_ms <= timeoutMs;
    }

    inline bool is_cap_feedback_healthy(
        const Manager &manager,
        bool capConnected) {
        return capConnected && manager.robot_set->super_cap_info.errorCode == 0;
    }

    inline bool is_motor_connected(const Hardware::DJIMotor &motor) {
        return !motor.offline();
    }

    inline bool are_all_motors_connected(const Manager &manager) {
        for (int i = 0; i < 4; ++i) {
            if (!is_motor_connected(manager.motors[i])) {
                return false;
            }
        }
        return true;
    }

    inline uint8_t get_game_type(const Manager &manager) {
        if (manager.robot_set == nullptr) {
            return RefGameTypeAlliance3v3;
        }
        return manager.robot_set->referee_info.game_status_data.game_type;
    }

    inline bool is_infantry_duel_game(const Manager &manager) {
        return get_game_type(manager) == RefGameTypeInfantryDuel;
    }

    inline float fallback_referee_limit(
        const Manager &manager,
        uint8_t latestLevel) {
        uint8_t level = std::clamp<uint8_t>(latestLevel, 1U, maxLevel);
        switch (manager.division) {
            case Division::HERO:
                return HeroChassisPowerLimit_HP_FIRST[level - 1U];
            case Division::INFANTRY:
                return is_infantry_duel_game(manager)
                           ? InfantryDuelChassisPowerLimit
                           : static_cast<float>(InfantryChassisPowerLimit_HP_FIRST[level - 1U]);
            case Division::SENTRY:
                return SentryChassisPowerLimit;
            default:
                return InfantryChassisPowerLimit_HP_FIRST[0];
        }
    }

    inline float offline_referee_limit(
        const Manager &manager,
        uint8_t latestLevel) {
        uint8_t level = std::clamp<uint8_t>(latestLevel, 1U, maxLevel);
        switch (manager.division) {
            case Division::HERO:
                return HeroChassisPowerLimit_HP_FIRST[level - 1U];
            case Division::INFANTRY:
                return static_cast<float>(InfantryChassisPowerLimit_HP_FIRST[level - 1U]);
            case Division::SENTRY:
                return SentryChassisPowerLimit;
            default:
                return InfantryChassisPowerLimit_HP_FIRST[0];
        }
    }
}  // namespace Power::detail
