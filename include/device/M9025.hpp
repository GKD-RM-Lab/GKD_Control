#pragma once
#include <string>
#include <linux/can.h>

#include "actuator.hpp"
#include "deviece_base.hpp"

namespace Device
{
    class M9025 final : public DeviceBase, public Actuator
    {
    public:

        struct Message {
            uint16_t ecd = 0;
            int16_t speed_rpm = 0;
            int16_t given_current = 0;
            uint8_t temperate = 0;

            void unpack(const can_frame &frame);
        };

        M9025(const std::string &can_name, int id);
        ~M9025() override = default;
        void set(float x) override;
        void unpack(const can_frame& frame);
        void enable();

        const int id = 0;
        const std::string can_name;

        Message motor_measure;
        int16_t give_current = 0;
    };
}