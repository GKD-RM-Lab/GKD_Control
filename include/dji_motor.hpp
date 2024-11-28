#pragma once

#include "dji_motor.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <linux/can.h>

#include "actuator.hpp"
#include "can.hpp"
#include "types.hpp"

namespace Hardware {
    enum class DJIMotorCanID {
        ID1FF = 0,
        ID200 = 1,
        ID2FF = 2,
        ID_NULL = 3
    };

    enum class DJIMotorType {
        M2006 = 2006,
        M3508 = 3508,
        M6020 = 6020,
        INVALID = 0
    };

    struct DJIMotorConfig {
        DJIMotorType type_;
        std::string can_name_;
        int id_;
        float radius_;

        struct TypeCast {
            DJIMotorType type_;
            consteval TypeCast(const int type) {
                switch (type) {
                    case 2006:
                        type_ = DJIMotorType::M2006;
                    break;
                    case 3508:
                        type_ = DJIMotorType::M3508;
                    break;
                    case 6020:
                        type_ = DJIMotorType::M6020;
                    break;
                    default:
                        throw std::invalid_argument("Motor Config Error: Invalid motor type");
                        type_ = DJIMotorType::INVALID;
                }
            }

            TypeCast(const DJIMotorType type) : type_(type) {}

            operator DJIMotorType() const {
                return type_;
            }
        };

        DJIMotorConfig(const TypeCast type, const std::string &name, const int id, const float radius = 1.f) :
            type_(type), can_name_(name), id_(id), radius_(radius) {}
    };

    class DJIMotor final : public Actuator {
    public:
        constexpr static fp32 RPM_TO_RAD_S = 2.f * M_PIf / 60.f;
        constexpr static fp32 ECD_8192_TO_RAD = 2.f * M_PIf / 8192.f;

        struct Message {
            uint16_t ecd = 0;
            int16_t speed_rpm = 0;
            int16_t given_current = 0;
            uint8_t temperate = 0;

            void unpack(const can_frame &frame);
        };

        struct Data {
            float reduction_ratio = 1.f;            // 减速比
            float radius = 1.f;                     // 半径 (m)

            float rotor_angle = 0.f;                // 转子角度 (rad)
            float rotor_angular_velocity = 0.f;     // 转子角速度 (rad/s)
            float rotor_linear_velocity = 0.f;      // 转子线速度 (m/s, 转子角速度 * 半径)

            float output_angular_velocity = 0.f;    // 输出角速度 (rad/s)
            float output_linear_velocity = 0.f;     // 输出线速度 (输出角速度 * 半径)
        };

        struct Can_info {
            std::string can_name_;
            DJIMotorCanID can_id_ = DJIMotorCanID::ID_NULL;
            int data_bias = 0;
            int callback_flag = 0;
        };

        Can_info can_info;
        std::string motor_name_;
        Message motor_measure_{};
        Data data_{};

        int motor_id_ = 0;
        bool motor_enabled_ = false;
        int16_t give_current = 0;

        explicit DJIMotor(const DJIMotorConfig &config);

        template<typename ...Args>
        explicit DJIMotor(DJIMotorConfig::TypeCast type, Args && ... args) :
        DJIMotor(DJIMotorConfig(type, std::forward<Args>(args)...)) {}

        explicit DJIMotor(const DJIMotor &other) = delete;

        explicit DJIMotor(DJIMotor &&other) = delete;

        void unpack(const can_frame &frame);

        void set(float x) override;

        void enable();
    };

    namespace DJIMotorManager {
        struct CanBlock {
            IO::Can_interface *can_ = nullptr;
            std::vector<DJIMotor *> motors_;
        };

        extern void register_motor(DJIMotor &motor);

        extern void start();
    }
}
