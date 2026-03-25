#pragma once

#include <chrono>
#include <cstdint>
#include <memory>

#include "robot.hpp"

namespace RefereeRuntime
{
    constexpr uint64_t kOfflineTimeoutMs = 300U;

    inline uint64_t now_ms() {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
    }

    inline bool is_connected(const Robot::Robot_set& robot, uint64_t current_ms = now_ms()) {
        return robot.referee_last_rx_ms > 0 && current_ms >= robot.referee_last_rx_ms &&
               current_ms - robot.referee_last_rx_ms <= kOfflineTimeoutMs;
    }

    inline bool is_connected(
        const std::shared_ptr<Robot::Robot_set>& robot,
        uint64_t current_ms = now_ms()) {
        return robot != nullptr && is_connected(*robot, current_ms);
    }

    inline bool chassis_output_enabled(const Robot::Robot_set& robot, bool referee_connected) {
        return !referee_connected || robot.referee_info.game_robot_status_data.mains_power_chassis_output;
    }

    inline bool shooter_output_enabled(const Robot::Robot_set& robot, bool referee_connected) {
        return !referee_connected || robot.referee_info.game_robot_status_data.mains_power_shooter_output;
    }
}  // namespace RefereeRuntime
