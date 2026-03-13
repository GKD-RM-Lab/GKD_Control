#include "device/super_cap.hpp"

#include <algorithm>
#include <chrono>

#include "macro_helpers.hpp"
#include "power_controller.hpp"
#include "utils.hpp"

namespace Device
{
    static constexpr uint64_t REFEREE_OFFLINE_TIMEOUT_MS = 300U;
    static constexpr float CAP_POWER_LIMIT_SCALE = 0.9f;

    void Super_Cap::init(
        const std::string& can_name,
        const std::shared_ptr<Robot::Robot_set>& robot) {
        robot_set = robot;
        can_name_ = can_name;
        can = IO::io<CAN>[can_name];
        if (can == nullptr) {
            LOG_ERR("[CAP_INIT] can lookup failed: %s\n", can_name.c_str());
            return;
        }
        // 0x51: 超级电容上报状态帧
        can->register_callback_key(
            0x51, std::bind(&Super_Cap::unpack, this, std::placeholders::_1));
    }

    void Super_Cap::unpack(const can_frame& frame) {
        static int delta = 0;
        delta++;

        const uint16_t robot_level = robot_set->referee_info.game_robot_status_data.robot_level;
        const uint16_t level_index = static_cast<uint16_t>(
            std::clamp<int>(static_cast<int>(robot_level), 1, static_cast<int>(Power::maxLevel)) - 1);
        const uint8_t game_type = robot_set->referee_info.game_status_data.game_type;
        const float referee_limit =
            static_cast<float>(robot_set->referee_info.game_robot_status_data.chassis_power_limit);
        const float fallback_limit = MUXDEF(
            CONFIG_HERO,
            static_cast<float>(Power::HeroChassisPowerLimit_HP_FIRST[level_index]),
            MUXDEF(
                CONFIG_INFANTRY,
                (game_type == Power::RefGameTypeInfantryDuel)
                    ? Power::InfantryDuelChassisPowerLimit
                    : static_cast<float>(Power::InfantryChassisPowerLimit_HP_FIRST[level_index]),
                static_cast<float>(Power::SentryChassisPowerLimit)));
        const float target_limit = referee_limit > 0.0f ? referee_limit : fallback_limit;
        const uint16_t power_limit = static_cast<uint16_t>(
            std::clamp(target_limit * CAP_POWER_LIMIT_SCALE, 1.0f, 65535.0f));

        if (delta >= 500) {
            set(true, power_limit);
            delta = 0;
        }

        std::memcpy(&robot_set->super_cap_info, frame.data, 8);
        update_time();
        robot_set->super_cap_last_rx_ms = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
    }

    void Super_Cap::set(bool enable, uint16_t power_limit) {
        if (can == nullptr) {
            can = IO::io<CAN>[can_name_];
            if (can == nullptr) {
                LOG_ERR("[CAP_TX] can is null, skip send, can_name=%s\n", can_name_.c_str());
                return;
            }
        }

        can_frame send{};
        const uint64_t now_ms = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
        const bool referee_connected =
            robot_set->referee_last_rx_ms > 0 &&
            now_ms >= robot_set->referee_last_rx_ms &&
            now_ms - robot_set->referee_last_rx_ms <= REFEREE_OFFLINE_TIMEOUT_MS;
        const uint16_t referee_buffer_energy = referee_connected
                                                   ? robot_set->referee_info.power_heat_data.chassis_power_buffer
                                                   : 60U;
        send.can_id = 0x061;
        send.can_dlc = 8;
        send.data[0] = enable ? 0x01 : 0x00;
        send.data[1] = power_limit & 0xff;
        send.data[2] = power_limit >> 8;
        send.data[3] = referee_buffer_energy & 0xff;
        send.data[4] = referee_buffer_energy >> 8;

        // LOG_INFO(
        //     "[CAP_TX] en: %s set_limit=%u ref_limit=%u ref_buf=%u tx_limit=%u tx_buf=%u\n",
        //     enable ? "on" : "off",
        //     power_limit,
        //     robot_set->referee_info.game_robot_status_data.chassis_power_limit,
        //     referee_buffer_energy,
        //     static_cast<uint16_t>(send.data[1] | (send.data[2] << 8)),
        //     static_cast<uint16_t>(send.data[3] | (send.data[4] << 8)));
        can->send(send);
    }
}  // namespace Device
