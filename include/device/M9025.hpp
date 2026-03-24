#pragma once
#include <linux/can.h>

#include <cmath>
#include <cstdint>
#include <string>

#include "actuator.hpp"
#include "deviece_base.hpp"

namespace Device
{
    class M9025 final : public DeviceBase, public Actuator
    {
    public:
        constexpr static float RPM_TO_RAD_S = 2.f * M_PIf / 60.f;
        constexpr static float ECD_65535_TO_RAD = 2.f * M_PIf / 65535.f;

        struct Message {
            uint16_t ecd = 0;
            int16_t speed_rpm = 0;
            int16_t given_current = 0;
            uint8_t temperate = 0;

            void unpack(const can_frame &frame);
        };

        struct Data {
            float rotor_angle = 0.f;
            float rotor_angular_velocity = 0.f;
        };

        M9025(const std::string &can_name, int id, uint8_t command_header = 0xA0);
        ~M9025() override = default;
        void set(float x) override;
        void set_zero();
        void unpack(const can_frame& frame);
        void enable();
        void run();
        void request_status_2();

        const int id = 0;
        const std::string can_name;
        const uint8_t command_header = 0xA0;

        Message motor_measure;
        Data data_{};
        int16_t give_current = 0;

    private:
        void send_command(uint8_t command, int16_t value = 0);
    };
}
