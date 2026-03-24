#include "device/M9025.hpp"

#include "io.hpp"

namespace Device
{
    namespace
    {
        constexpr uint8_t kRunCommand = 0x88;
        constexpr uint8_t kReadStatus2Command = 0x9C;

        bool is_status_2_frame(const can_frame& frame) {
            const auto command = frame.data[0];
            return command == kReadStatus2Command || (command >= 0xA0 && command <= 0xA8);
        }
    }

    M9025::M9025(const std::string& can_name, const int id, const uint8_t command_header)
        : id(id), can_name(can_name), command_header(command_header) {
    }

    void M9025::send_command(const uint8_t command, const int16_t value) {
        can_frame frame{};
        frame.can_id = 0x140 + id;
        frame.can_dlc = 8;
        frame.data[0] = command;
        frame.data[4] = value & 0xFF;
        frame.data[5] = value >> 8;
        IO::io<CAN>[can_name] -> send(frame);
    }

    void M9025::set(float x) {
        x = x >> controller;
        give_current = static_cast<int16_t>(x);
        send_command(command_header, give_current);
    }
    void M9025::set_zero() {
        give_current = 0;
        send_command(command_header, give_current);
    }

    void M9025::Message::unpack(const can_frame& frame) {
        ecd = (uint16_t)(frame.data[7] << 8 | frame.data[6]);
        speed_rpm = (int16_t)(frame.data[5] << 8 | frame.data[4]);
        given_current = (int16_t)(frame.data[3] << 8 | frame.data[2]);
        temperate = frame.data[1];
    }

    void M9025::unpack(const can_frame& frame) {
        if (is_status_2_frame(frame)) {
            motor_measure.unpack(frame);
            data_.rotor_angle = ECD_65535_TO_RAD * static_cast<float>(motor_measure.ecd);
            data_.rotor_angular_velocity = RPM_TO_RAD_S * static_cast<float>(motor_measure.speed_rpm);
        }
        update_time();
    }

    void M9025::enable() {
        IO::io<CAN>[can_name] -> register_callback_key(
                                  0x140 + id, [&](const can_frame& frame) { unpack(frame); });
    }

    void M9025::run() {
        send_command(kRunCommand);
    }

    void M9025::request_status_2() {
        send_command(kReadStatus2Command);
    }

}  // namespace Device
