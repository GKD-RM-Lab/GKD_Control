#include "device/imu.hpp"

#include "io.hpp"
#include "serial_interface.hpp"
#include "user_lib.hpp"

namespace Device
{
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

    void IMU::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;

        IO::io<SERIAL>["/dev/IMU_RIGHT"]->register_callback([&](const Types::ReceivePacket &rp) {
            unpack1(rp);
        });

        // Robot::hardware->register_callback<SER2>([&](const Types::ReceivePacket &rp) {
        //     // LOG_INFO("gyro 2 %f %f %f\n", rp.yaw, rp.pitch, rp.roll);
        //     unpack2(rp);
        // });

        // Robot::hardware->register_callback<SER3>([&](const Types::ReceivePacket &rp) {
        //     // LOG_INFO("gyro 3 %f %f %f\n", rp.yaw, rp.pitch, rp.roll);
        //     unpack3(rp);
        // });
    }
}  // namespace Device
