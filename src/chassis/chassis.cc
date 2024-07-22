#include "chassis.hpp"

namespace Chassis
{
    Chassis::Chassis() {
        // 电机初始int 
        motors.assign(4, Hardware::Motor{ Config::M3508_SPEED_PID_CONFIG });
    }

    void Chassis::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;
        for (size_t i = 0; i < motors.size(); i++) {
            auto &mot = motors[i];
            Robot::hardware->register_callback<CAN1>(0x201 + i, [&mot](const auto &frame) { mot.unpack(frame); });
        }
    }

    void Chassis::control_loop() {
        decomposition_speed();
        update_data();
        if (no_force) {
            for (auto &m : motors) {
                m.give_current = 0;
            }
        } else {
            fp32 max_speed = *std::max_element(wheel_speed, wheel_speed + 4);
            fp32 speed_rate = std::min(max_wheel_speed / max_speed, 1.f);
            for (auto &m : motors) {
                m.speed_set *= speed_rate;
                m.pid_ctrler.calc(m.speed, m.speed_set);
                m.give_current = (int16_t)(m.pid_ctrler.out);
            }
        }
        Robot::hardware->send<CAN1>(Hardware::get_frame(0x200, motors));
    }

    void Chassis::decomposition_speed() {
        wheel_speed[0] = -vx_set + vy_set + wz_set;
        wheel_speed[1] = vx_set + vy_set + wz_set;
        wheel_speed[2] = vx_set - vy_set + wz_set;
        wheel_speed[3] = -vx_set - vy_set + wz_set;
    }

    void Chassis::update_data() {
        for (auto &m : motors) {
            m.speed = Config::CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * (fp32)m.motor_measure.speed_rpm;
            m.accel = Config::CHASSIS_CONTROL_FREQUENCE * m.pid_ctrler.Dbuf;
        }
    }
}  // namespace Chassis
