#include "referee.hpp"

#include <chrono>

#include "io.hpp"
#include "serial_interface.hpp"
#include "utils.hpp"

namespace Device
{
    namespace
    {
        constexpr int kMinFrameLength = 5 + 2 + 2;
        constexpr uint16_t kPowerHeatDataLength = 14;
        constexpr uint16_t kBulletRemainingLength = 8;
        constexpr uint16_t kBulletRemainingLengthLegacy = 6;
        constexpr uint64_t kRefereeOfflineTimeoutMs = 300U;
        constexpr uint16_t kBulletAllowanceDeltaGuard = 1000U;
        constexpr uint8_t kGameProgressSettlement = 5U;

        inline uint16_t readU16LE(const uint8_t *data) {
            return static_cast<uint16_t>(data[0]) |
                   (static_cast<uint16_t>(data[1]) << 8);
        }

#ifdef CONFIG_INFANTRY
        constexpr char kLedSerialName[] = "/dev/LED";

        struct LedFrame
        {
            uint8_t head = 0xAA;
            uint8_t cmd = 0x10;
            uint8_t led_mask = 0x00;
            uint8_t tail = 0x55;
        } __attribute__((packed));

        void sendLedFrame(const std::shared_ptr<Robot::Robot_set> &robot_set) {
            static SERIAL *led_serial = IO::io<SERIAL>[kLedSerialName];
            static uint8_t last_led_mask = 0xFF;
            static auto last_send_time = std::chrono::steady_clock::now();

            if (led_serial == nullptr) {
                return;
            }

            uint8_t led_mask = 0x00;
            if (robot_set->spin_state) {
                led_mask |= 0x01;
            }
            if (robot_set->fric_led_open) {
                led_mask |= 0x02;
            }

            auto now = std::chrono::steady_clock::now();
            if (led_mask == last_led_mask &&
                now - last_send_time < std::chrono::milliseconds(200)) {
                return;
            }

            LedFrame frame; 
            frame.led_mask = led_mask;

            led_serial->send(frame);            
            
            last_led_mask = led_mask;
            last_send_time = now;
        }
#endif
    }  // namespace

    // read data from referee
    void Dji_referee::read() {
        if (base_.serial_.available()) {
            rx_len_ = static_cast<int>(base_.serial_.available());
            base_.serial_.read(rx_buffer_, rx_len_);
            // printf("%d len\n", rx_len_);
        } else
            return;
        uint8_t temp_buffer[256] = { 0 };
        int frame_len;
        if (rx_len_ < k_unpack_buffer_length_) {
            for (int k_i = 0; k_i < k_unpack_buffer_length_ - rx_len_; ++k_i)
                temp_buffer[k_i] = unpack_buffer_[k_i + rx_len_];
            for (int k_i = 0; k_i < rx_len_; ++k_i)
                temp_buffer[k_i + k_unpack_buffer_length_ - rx_len_] = rx_buffer_[k_i];
            for (int k_i = 0; k_i < k_unpack_buffer_length_; ++k_i)
                unpack_buffer_[k_i] = temp_buffer[k_i];
        }
        for (int k_i = 0; k_i < k_unpack_buffer_length_ - k_header_length_; ++k_i) {
            if (unpack_buffer_[k_i] == 0xA5) {
                frame_len = unpack(&unpack_buffer_[k_i], k_unpack_buffer_length_ - k_i);
                if (frame_len > 0)
                    k_i += frame_len - 1;
            }
        }
        clearRxBuffer();
    }

    void Dji_referee::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;
    }

    int Dji_referee::unpack(uint8_t *rx_data, int remaining_len) {
        if (rx_data == nullptr || remaining_len < kMinFrameLength) {
            return -1;
        }

        uint16_t cmd_id;
        int frame_len;
        Referee::FrameHeader frame_header;
        bool parsed = false;

        memcpy(&frame_header, rx_data, k_header_length_);
        if (frame_header.sof != 0xA5 ||
            !static_cast<bool>(base_.verifyCRC8CheckSum(rx_data, k_header_length_))) {
            return -1;
        }

        frame_len = frame_header.data_length + k_header_length_ + k_cmd_id_length_ + k_tail_length_;
        if (frame_header.data_length > k_unpack_buffer_length_ - k_header_length_ - k_cmd_id_length_ -
                                         k_tail_length_) {
            return -1;
        }
        if (frame_len > remaining_len || frame_len > k_unpack_buffer_length_) {
            return -1;
        }
        if (base_.verifyCRC16CheckSum(rx_data, frame_len) != 1) {
            return -1;
        }

        cmd_id = (rx_data[6] << 8 | rx_data[5]);
        switch (cmd_id) {
            case Referee::RefereeCmdId::GAME_STATUS_CMD: {
                if (frame_header.data_length == sizeof(Referee::GameStatus)) {
                    memcpy(
                        &robot_set->referee_info.game_status_data,
                        rx_data + 7,
                        sizeof(Referee::GameStatus));
                    parsed = true;
                }
                break;
            }
            case Referee::RefereeCmdId::GAME_RESULT_CMD: {
                if (frame_header.data_length == sizeof(Referee::GameResult)) {
                    memcpy(
                        &robot_set->referee_info.game_result_ref,
                        rx_data + 7,
                        sizeof(Referee::GameResult));
                    parsed = true;
                }
                break;
            }
            case Referee::RefereeCmdId::REFEREE_WARNING_CMD: {
                if (frame_header.data_length == sizeof(Referee::RefereeWarning)) {
                    memcpy(
                        &robot_set->referee_info.referee_warning_ref,
                        rx_data + 7,
                        sizeof(Referee::RefereeWarning));
                    parsed = true;
                }
                break;
            }
            case Referee::RefereeCmdId::ROBOT_STATUS_CMD: {
                if (frame_header.data_length == sizeof(Referee::GameRobotStatus)) {
                    memcpy(
                        &robot_set->referee_info.game_robot_status_data,
                        rx_data + 7,
                        sizeof(Referee::GameRobotStatus));
                    parsed = true;
                }
                break;
            }
            case Referee::RefereeCmdId::POWER_HEAT_DATA_CMD: {
                if (frame_header.data_length == kPowerHeatDataLength) {
                    const uint8_t *data = rx_data + 7;
                    auto &power_heat_data = robot_set->referee_info.power_heat_data;
                    power_heat_data.chassis_power_buffer = readU16LE(data + 8);
                    power_heat_data.shooter_id_1_17_mm_cooling_heat = readU16LE(data + 10);
                    power_heat_data.shooter_id_1_42_mm_cooling_heat = readU16LE(data + 12);
                    parsed = true;
                    // LOG_INFO("gimbal power:%d\n", robot_set->referee_info.power_heat_data.shooter_id_1_17_mm_cooling_heat);

                }
                break;
            }
             case Referee::RefereeCmdId::BULLET_REMAINING_CMD: {
                if (frame_header.data_length == kBulletRemainingLength ||
                    frame_header.data_length == kBulletRemainingLengthLegacy) {
                    const uint8_t *data = rx_data + 7;
                    auto &bullet_allowance_data = robot_set->referee_info.bullet_allowance_data;
                    bullet_allowance_data.bullet_allowance_num_17_mm = readU16LE(data);
                    bullet_allowance_data.bullet_allowance_num_42_mm = readU16LE(data + 2);
                    bullet_allowance_data.coin_remaining_num = readU16LE(data + 4);
                    bullet_allowance_data.projectile_allowance_fortress =
                        frame_header.data_length == kBulletRemainingLength ? readU16LE(data + 6) : 0;
                    parsed = true;
                }
                break;
            }
            case Referee::RefereeCmdId::SHOOT_DATA_CMD: {
                if (frame_header.data_length == sizeof(Referee::ShootData)) {
                    parsed = true;
                }
                break;
            }
            default:
                break;
        }

        if (parsed) {
            base_.referee_data_is_online_ = true;
            robot_set->referee_last_rx_ms = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count());
        }
        return frame_len;
    }

    void Dji_referee::task() {
        static uint32_t fired_bullet_total = 0;
        static uint16_t last_remain_bullet_num = 0;
        static bool bullet_counter_inited = false;

        while (1) {
            read();

            const uint64_t now_ms = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count());
            const bool referee_connected =
                robot_set->referee_last_rx_ms > 0 &&
                now_ms >= robot_set->referee_last_rx_ms &&
                now_ms - robot_set->referee_last_rx_ms <= kRefereeOfflineTimeoutMs;
            const uint16_t remain_bullet_num = MUXDEF(
                CONFIG_HERO,
                robot_set->referee_info.bullet_allowance_data.bullet_allowance_num_42_mm,
                robot_set->referee_info.bullet_allowance_data.bullet_allowance_num_17_mm);
            const uint8_t game_progress =
                robot_set->referee_info.game_status_data.game_progress & 0x0FU;
            uint16_t stable_remain_bullet_num = remain_bullet_num;
            if (game_progress == kGameProgressSettlement) {
                fired_bullet_total = 0U;
                if (referee_connected) {
                    last_remain_bullet_num = remain_bullet_num;
                    stable_remain_bullet_num = remain_bullet_num;
                    bullet_counter_inited = true;
                } else {
                    last_remain_bullet_num = 0U;
                    stable_remain_bullet_num = 0U;
                    bullet_counter_inited = false;
                }
            } else if (referee_connected) {
                if (!bullet_counter_inited) {
                    fired_bullet_total = 0U;
                    last_remain_bullet_num = remain_bullet_num;
                    bullet_counter_inited = true;
                } else {
                    const int32_t remain_delta = static_cast<int32_t>(remain_bullet_num) -
                                                static_cast<int32_t>(last_remain_bullet_num);
                    const bool delta_is_valid =
                        remain_delta <= static_cast<int32_t>(kBulletAllowanceDeltaGuard) &&
                        remain_delta >= -static_cast<int32_t>(kBulletAllowanceDeltaGuard);
                    if (delta_is_valid && remain_delta < 0) {
                        const uint32_t consumed = static_cast<uint32_t>(-remain_delta);
                        if (consumed > 0U) {
                            fired_bullet_total += consumed;
                        }
                    }
                    if (delta_is_valid) {
                        last_remain_bullet_num = remain_bullet_num;
                    } else {
                        stable_remain_bullet_num = last_remain_bullet_num;
                    }
                }
            } else if (bullet_counter_inited) {
                stable_remain_bullet_num = last_remain_bullet_num;
            }

            const uint32_t purchased_bullet_num =
                bullet_counter_inited ? fired_bullet_total + stable_remain_bullet_num : 0U;
            const uint32_t remain_bullet_num_for_ui =
                bullet_counter_inited ? stable_remain_bullet_num : 0U;

            bool referee_fire_allowance = MUXDEF(
                CONFIG_HERO,
                robot_set->referee_info.bullet_allowance_data.bullet_allowance_num_42_mm > 0,
                robot_set->referee_info.bullet_allowance_data.bullet_allowance_num_17_mm > 0);
            // LOG_INFO("ui update\n");
            update_ui_data(
                &base_,
                robot_set->fric_led_open,
                robot_set->cv_fire,
                robot_set->spin_state,
                ((float)robot_set->super_cap_info.capEnergy / 250) * 100,
                purchased_bullet_num,
                remain_bullet_num_for_ui);

#ifdef CONFIG_INFANTRY
            sendLedFrame(robot_set);
#endif
    
            // LOG_INFO("game status:%d\n", robot_set->referee_info.game_status_data.game_progress);
            //  LOG_INFO(
            //     "status: gimbal:%s | chassis:%s | shooter:%s\n",
            //     (robot_set->referee_info.game_robot_status_data.mains_power_gimbal_output == 0) ? "off" : "on",
            //     (robot_set->referee_info.game_robot_status_data.mains_power_chassis_output == 0) ? "off" : "on",
            //     (robot_set->referee_info.game_robot_status_data.mains_power_shooter_output == 0) ? "off" : "on");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void Dji_referee::task_ui() {
        custom_ui_task(&base_, robot_set->referee_info.game_robot_status_data.robot_id);
    }
}  // namespace Device
