#include "device/imu.hpp"

#include "io.hpp"
#include "serial_interface.hpp"
#include "user_lib.hpp"

namespace Device
{
    IMU::IMU(const std::string& serial_name) : serial_name(serial_name){}

    void IMU::enable() {
        auto serial_interface = IO::io<SERIAL>[serial_name];
        if(serial_interface == nullptr) {
            LOG_ERR("IMU Error: no serial named %s\n", serial_name.c_str());
            return;
        }
        serial_interface->register_callback([&](const Types::ReceivePacket &rp) {
            unpack(rp);
        });
    }

    void IMU::unpack(const Types::ReceivePacket &pkg) {
        yaw = UserLib::rad_format(pkg.yaw * (M_PIf / 180));
        pitch = -UserLib::rad_format(pkg.pitch * (M_PIf / 180));
        roll = UserLib::rad_format(pkg.roll * (M_PIf / 180));
        yaw_rate = pkg.yaw_v * (M_PIf / 180) / 1000;
        pitch_rate = pkg.pitch_v * (M_PIf / 180) / 1000;
        roll_rate = pkg.roll_v * (M_PIf / 180) / 1000;
        update_time();
    }

    void IMU::unpack1(const Types::ReceivePacket &pkg) {
        robot_set->gyro1_ins_yaw = UserLib::rad_format(pkg.yaw * (M_PIf / 180));
        robot_set->gyro1_ins_pitch = -UserLib::rad_format(pkg.pitch * (M_PIf / 180));
        robot_set->gyro1_ins_roll = UserLib::rad_format(pkg.roll * (M_PIf / 180));
        robot_set->gyro1_ins_yaw_v = (pkg.yaw_v * (M_PIf / 180)) / 1000;
        robot_set->gyro1_ins_pitch_v = (pkg.pitch_v * (M_PIf / 180)) / 1000;
        robot_set->gyro1_ins_roll_v = (pkg.roll_v * (M_PIf / 180)) / 1000;
        update_time();
    }

    void IMU::unpack2(const Types::ReceivePacket &pkg) {
        robot_set->gyro2_ins_yaw = UserLib::rad_format(pkg.yaw * (M_PIf / 180));
        robot_set->gyro2_ins_pitch = UserLib::rad_format(pkg.pitch * (M_PIf / 180));
        robot_set->gyro2_ins_roll = UserLib::rad_format(pkg.roll * (M_PIf / 180));
        robot_set->gyro2_ins_yaw_v = (pkg.yaw_v * (M_PIf / 180)) / 1000;
        robot_set->gyro2_ins_pitch_v = (pkg.pitch_v * (M_PIf / 180)) / 1000;
        robot_set->gyro2_ins_roll_v = (pkg.roll_v * (M_PIf / 180)) / 1000;
        update_time();
    }

    void IMU::unpack3(const Types::ReceivePacket &pkg) {
        robot_set->gyro3_ins_yaw = UserLib::rad_format(pkg.yaw * (M_PIf / 180));
        robot_set->gyro3_ins_pitch = UserLib::rad_format(pkg.pitch * (M_PIf / 180));
        robot_set->gyro3_ins_roll = UserLib::rad_format(pkg.roll * (M_PIf / 180));
        robot_set->gyro3_ins_yaw_v = (pkg.yaw_v * (M_PIf / 180)) / 1000;
        robot_set->gyro3_ins_pitch_v = (pkg.pitch_v * (M_PIf / 180)) / 1000;
        robot_set->gyro3_ins_roll_v = (pkg.roll_v * (M_PIf / 180)) / 1000;
        update_time();
    }
}  // namespace Device
