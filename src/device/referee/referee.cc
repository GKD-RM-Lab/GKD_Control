#include "referee.hpp"

#include <chrono>

#include "utils.hpp"

namespace Device
{
    namespace
    {
        constexpr int kMinFrameLength = 5 + 2 + 2;
        constexpr uint16_t kPowerHeatDataLength = 14;
        constexpr uint16_t kBulletRemainingLength = 8;

        inline uint16_t readU16LE(const uint8_t *data) {
            return static_cast<uint16_t>(data[0]) |
                   (static_cast<uint16_t>(data[1]) << 8);
        }
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
                if (frame_header.data_length == kBulletRemainingLength) {
                    const uint8_t *data = rx_data + 7;
                    auto &bullet_allowance_data = robot_set->referee_info.bullet_allowance_data;
                    bullet_allowance_data.bullet_allowance_num_17_mm = readU16LE(data);
                    bullet_allowance_data.bullet_allowance_num_42_mm = readU16LE(data + 2);
                    bullet_allowance_data.coin_remaining_num = readU16LE(data + 4);
                    bullet_allowance_data.projectile_allowance_fortress = readU16LE(data + 6);
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
        while (1) {
            read();
            bool referee_fire_allowance = MUXDEF(
                CONFIG_HERO,
                robot_set->referee_info.bullet_allowance_data.bullet_allowance_num_42_mm > 0,
                robot_set->referee_info.bullet_allowance_data.bullet_allowance_num_17_mm > 0);
            // LOG_INFO("ui update\n");
            update_ui_data(
                &base_,
                robot_set->friction_real_state && referee_fire_allowance,
                robot_set->cv_fire,
                robot_set->spin_state,
                ((float)robot_set->super_cap_info.capEnergy / 250) * 100);
    
            
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
