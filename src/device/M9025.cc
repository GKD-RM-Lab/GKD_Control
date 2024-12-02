#include "device/M9025.hpp"

#include "io.hpp"

namespace Device
{
    M9025::M9025(const std::string &can_name, const int id) : id(id), can_name(can_name) {}

    void M9025::set(float x) {
        give_current = static_cast<int16_t>(x);
        can_frame frame{};
        frame.can_id = 0x140 + id;
        frame.can_dlc = 8;
        frame.data[0] = 0xA0;
        frame.data[4] = give_current & 0xFF;
        frame.data[5] = give_current >> 8;
        IO::io<CAN>[can_name]->send(frame);
    }

    void M9025::Message::unpack(const can_frame& frame) {
        ecd = (uint16_t)(frame.data[7] << 8 | frame.data[6]);
        speed_rpm = (uint16_t)(frame.data[5] << 8 | frame.data[4]);
        given_current = (uint16_t)(frame.data[3] << 8 | frame.data[2]);
        temperate = frame.data[1];

    }

    void M9025::unpack(const can_frame& frame) {
        motor_measure.unpack(frame);
        update_time();
    }

    void M9025::enable() {
        IO::io<CAN>[can_name]->register_callback_key(0x140 + id, [&](const can_frame& frame) {
            unpack(frame);
        });
    }

}