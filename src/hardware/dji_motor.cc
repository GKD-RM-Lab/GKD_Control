#include "dji_motor.hpp"

#include "io.hpp"

#include <mutex>
#include <chrono>

namespace Hardware {

    DJIMotor::DJIMotor(const DJIMotorConfig &config) {
        auto &[type, can_name, motor_id, radius] = config;
        motor_id_ = motor_id;
        can_info.can_name_ = can_name;
        data_.radius = radius;
        if(type == DJIMotorType::M2006 || type == DJIMotorType::M3508) { // 构造 M3508 or M2006
            if(type == DJIMotorType::M2006) {
                motor_name_ = "{M2006#" + can_name + "#" + std::to_string(motor_id) + "}";
                data_.reduction_ratio = 1.f / 36.f;
            } else {
                motor_name_ = "{M3508#" + can_name + "#" + std::to_string(motor_id) + "}";
                data_.reduction_ratio = 1.f / 19.f;
            }
            can_info.callback_flag = 0x200 + motor_id;
            if (motor_id > 0 && motor_id <= 4) {
                can_info.can_id_ = DJIMotorCanID::ID200;
                can_info.data_bias = (motor_id - 1) << 1;
            } else if (motor_id > 4 && motor_id <= 8) {
                can_info.can_id_ = DJIMotorCanID::ID1FF;
                can_info.data_bias = (motor_id - 5) << 1;
            } else {
                LOG_ERR("Motor error[%s]: invalid M3508/M2006 id\n", motor_name_.c_str());
                can_info.can_id_ = DJIMotorCanID::ID_NULL;
                motor_id_ = 0;
            }
        } else if(type == DJIMotorType::M6020){ // 构造 M6020
            motor_name_ = "{M6020#" + can_name + "#" + std::to_string(motor_id) + "}";
            can_info.callback_flag = 0x204 + motor_id;
            data_.reduction_ratio = 1.f;
            if (motor_id > 0 && motor_id <= 4) {
                can_info.can_id_ = DJIMotorCanID::ID1FF;
                can_info.data_bias = (motor_id - 1) << 1;
            } else if (motor_id > 4 && motor_id <= 7) {
                can_info.can_id_ = DJIMotorCanID::ID2FF;
                can_info.data_bias = (motor_id - 5) << 1;
            } else {
                LOG_ERR("Motor error[%s]: invalid M6020 id\n", motor_name_.c_str());
                can_info.can_id_ = DJIMotorCanID::ID_NULL;
                motor_id_ = 0;
            }
        }
    }

    void DJIMotor::Message::unpack(const can_frame &frame) {
        ecd = static_cast<uint16_t>(frame.data[0] << 8 | frame.data[1]);
        speed_rpm = static_cast<int16_t>(frame.data[2] << 8 | frame.data[3]);
        given_current = static_cast<int16_t>(frame.data[4] << 8 | frame.data[5]);
        temperate = frame.data[6];
    }

    void DJIMotor::unpack(const can_frame &frame) {
        motor_measure_.unpack(frame);
        data_.rotor_angle = ECD_8192_TO_RAD * static_cast<float>(motor_measure_.ecd);
        data_.rotor_angular_velocity = RPM_TO_RAD_S * static_cast<float>(motor_measure_.speed_rpm);
        data_.rotor_linear_velocity = data_.rotor_angular_velocity * data_.radius;

        data_.output_angular_velocity = data_.rotor_angular_velocity * data_.reduction_ratio;
        data_.output_linear_velocity = data_.rotor_linear_velocity * data_.reduction_ratio;
        update_time();
    }

    void DJIMotor::set(float x) {
        x = x >> controller;
        give_current = static_cast<int16_t>(x);
    }

    void DJIMotor::enable() {
        DJIMotorManager::register_motor(*this);
    }

    namespace DJIMotorManager {

        std::unordered_map<std::string, CanBlock> motors_map;
        std::thread task_handle;
        std::mutex data_lock;

        bool can_conflict(const DJIMotor &motor1, const DJIMotor &motor2) {
            return (motor1.can_info.can_id_ == motor2.can_info.can_id_ &&
                    motor1.can_info.data_bias == motor2.can_info.data_bias) ||
                   motor1.can_info.callback_flag == motor2.can_info.callback_flag;
        }

        void register_motor(DJIMotor &motor) {
            std::unique_lock lock(data_lock);
            if (motor.motor_id_ == 0) {
                return;
            }
            auto can_interface = IO::io<CAN>[motor.can_info.can_name_];
            if(can_interface == nullptr) {
                LOG_ERR("Motor error[%s]: can device is invalid\n", motor.motor_name_.c_str());
                return;
            }
            auto &[can_, motors_] = motors_map[motor.can_info.can_name_];
            can_ = can_interface;
            for (const auto &other_motor: motors_) {
                if (can_conflict(*other_motor, motor)) {
                    LOG_ERR("Motor error[%s, %s]: A can conflict occurred when registering motor\n",
                            other_motor->motor_name_.c_str(), motor.motor_name_.c_str());
                    return;
                }
            }
            motor.motor_enabled_ = true;
            motors_.push_back(&motor);
            can_->register_callback_key(motor.can_info.callback_flag, [&](const can_frame &frame) {
                motor.unpack(frame);
            });
        }

        [[noreturn]] void task() {
            static can_frame frame[3] = {{}, {}, {}};
            static bool valid[3] = {false, false, false};
            while (true) {
                data_lock.lock();
                auto now = std::chrono::steady_clock::now();
                for (auto &[can_name, can_block]: motors_map) {
                    if (can_block.can_ == nullptr) {
                        can_block.can_ = IO::io<CAN>[can_name];
                    }
                    if (can_block.can_ == nullptr) {
                        continue;
                    }
                    frame[0] = {.can_id = 0x1ff, .len = 8};
                    frame[1] = {.can_id = 0x200, .len = 8};
                    frame[2] = {.can_id = 0x2ff, .len = 8};
                    valid[0] = valid[1] = valid[2] = false;
                    for (const auto motor: can_block.motors_) {
                        valid[static_cast<int>(motor->can_info.can_id_)] = true;
                        auto data = frame[static_cast<int>(motor->can_info.can_id_)].data;
                        data[motor->can_info.data_bias] = static_cast<uint16_t>(motor->give_current) >> 8;
                        data[motor->can_info.data_bias | 1] = motor->give_current & 0xff;
                    }
                    if (valid[0]) {
                        can_block.can_->send(frame[0]);
                    }
                    if (valid[1]) {
                        can_block.can_->send(frame[1]);
                    }
                    if (valid[2]) {
                        can_block.can_->send(frame[2]);
                    }
                }
                data_lock.unlock();
                std::this_thread::sleep_until(now + std::chrono::milliseconds(1));
            }
        }

        void start() {
            task_handle = std::thread(task);
        }
    }
}