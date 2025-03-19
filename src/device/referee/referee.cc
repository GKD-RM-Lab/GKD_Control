#include "referee.hpp"

#include "utils.hpp"

namespace Device
{
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
        for (int k_i = 0; k_i < k_unpack_buffer_length_ - k_frame_length_; ++k_i) {
            if (unpack_buffer_[k_i] == 0xA5) {
                frame_len = unpack(&unpack_buffer_[k_i]);
                if (frame_len != -1)
                    k_i += frame_len;
            }
        }
        clearRxBuffer();
    }

    void Dji_referee::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;
    }

    int Dji_referee::unpack(uint8_t *rx_data) {
        uint16_t cmd_id;
        int frame_len;
        Referee::FrameHeader frame_header;

        memcpy(&frame_header, rx_data, k_header_length_);
        if (static_cast<bool>(base_.verifyCRC8CheckSum(rx_data, k_header_length_))) {
            if (frame_header.data_length > 256)  // temporary and inaccurate value
            {
                printf(
                    "discard possible wrong frames, data length: %d\n", frame_header.data_length);
                return 0;
            }
            // printf("data len %x %d %x\n", frame_header.sof, frame_header.data_length, rx_data[6]
            // << 8 | rx_data[5]);
            frame_len =
                frame_header.data_length + k_header_length_ + k_cmd_id_length_ + k_tail_length_;
            if (base_.verifyCRC16CheckSum(rx_data, frame_len) == 1) {
                cmd_id = (rx_data[6] << 8 | rx_data[5]);
                switch (cmd_id) {
                    case Referee::RefereeCmdId::GAME_STATUS_CMD: {
                        memcpy(
                            &robot_set->referee_info.game_status_data,
                            rx_data + 7,
                            sizeof(Referee::GameStatus));

                        printf("game status\n");
                        break;
                    }
                    case Referee::RefereeCmdId::GAME_RESULT_CMD: {
                        memcpy(
                            &robot_set->referee_info.game_result_ref,
                            rx_data + 7,
                            sizeof(Referee::GameResult));
                        printf("game result\n");
                        break;
                    }
                    case Referee::RefereeCmdId::REFEREE_WARNING_CMD: {
                        memcpy(
                            &robot_set->referee_info.referee_warning_ref,
                            rx_data + 7,
                            sizeof(Referee::RefereeWarning));
                        break;
                    }
                    case Referee::RefereeCmdId::ROBOT_STATUS_CMD: {
                        memcpy(
                            &robot_set->referee_info.game_robot_status_data,
                            rx_data + 7,
                            sizeof(Referee::GameRobotStatus));
                        printf("robot result\n");
                        break;
                    }
                    case Referee::RefereeCmdId::POWER_HEAT_DATA_CMD: {
                        memcpy(
                            &robot_set->referee_info.power_heat_data,
                            rx_data + 7,
                            sizeof(Referee::PowerHeatData));
                        printf("power heat\n");
                        break;
                    }
                    case Referee::RefereeCmdId::BULLET_REMAINING_CMD: {
                        memcpy(
                            &robot_set->referee_info.bullet_allowance_data,
                            rx_data + 7,
                            sizeof(Referee::BulletAllowance));
                        printf("bullet remaining \n");
                        break;
                    }
                    default: printf("Referee command ID %d not found.\n", cmd_id); break;
                }
                base_.referee_data_is_online_ = true;
                return frame_len;
            }
        }
        return -1;
    }

    void Dji_referee::task() {
        while (1) {
            read();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}  // namespace Device
