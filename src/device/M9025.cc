#include "device/M9025.hpp"

#include "io.hpp"

namespace Device
{
    M9025::M9025(const std::string& can_name, const int id, const uint8_t command_header)
        : id(id), can_name(can_name), command_header(command_header) {
    }

    void M9025::set(float x) {
        x = x >> controller;
        give_current = static_cast<int16_t>(x);
        can_frame frame{};
        frame.can_id = 0x140 + id;
        frame.can_dlc = 8;
        frame.data[0] = command_header;
        frame.data[4] = give_current & 0xFF;
        frame.data[5] = give_current >> 8;
        IO::io<CAN>[can_name] -> send(frame);
    }

    void M9025::Message::unpack(const can_frame& frame) {
        ecd = (uint16_t)(frame.data[7] << 8 | frame.data[6]);
        speed_rpm = (int16_t)(frame.data[5] << 8 | frame.data[4]);
        given_current = (int16_t)(frame.data[3] << 8 | frame.data[2]);
        temperate = frame.data[1];
    }

    void M9025::unpack(const can_frame& frame) {
        motor_measure.unpack(frame);
        data_.rotor_angle = ECD_65535_TO_RAD * static_cast<float>(motor_measure.ecd);
        data_.rotor_angular_velocity = RPM_TO_RAD_S * static_cast<float>(motor_measure.speed_rpm);
        update_time();
    }

    void M9025::enable() {
        IO::io<CAN>[can_name] -> register_callback_key(
                                  0x140 + id, [&](const can_frame& frame) { unpack(frame); });
    }

}  // namespace Device
