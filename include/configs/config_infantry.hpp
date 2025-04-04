#pragma once
#include <cmath>
#include <vector>

#include "chassis/chassis_config.hpp"
#include "gimbal/gimbal_config.hpp"
#include "gimbal/gimbal_temp.hpp"
#include "pid_controller.hpp"
#include "shoot_config.hpp"
#include "types.hpp"

namespace Config
{
    using GimbalType = Gimbal::GimbalT;

    const std::vector<std::string> CanInitList = { "Hero_Chassis", "Hero_Gimbal" };

    const std::vector<std::string> SocketInitList = { "AUTO_AIM_CONTROL" };

    const std::vector<std::tuple<std::string, int, int>> SerialInitList = {
        { "/dev/IMU_HERO", 115200, 2000 }
    };

    const std::string rc_controller_serial = "/dev/IMU_HERO";

    const std::string super_cap_can_interface = "Hero_Chassis";

    const Chassis::ChassisConfig chassis_config = {
        .wheels_config = {
            Hardware::DJIMotorConfig{3508, "Hero_Chassis", 1, 0.075},
            Hardware::DJIMotorConfig{3508, "Hero_Chassis", 2, 0.075},
            Hardware::DJIMotorConfig{3508, "Hero_Chassis", 3, 0.075},
            Hardware::DJIMotorConfig{3508, "Hero_Chassis", 4, 0.075}
        },
        .chassis_follow_gimbal_pid_config = {
            .kp =           2.0f,
            .ki =           0.0f,
            .kd =           10.0f,
            .max_out =      6.0f,
            .max_iout =     0.2f,
        },
        .wheel_speed_pid_config = {
            .kp =           15000.0f,
            .ki =           10.0f,
            .kd =           0.0f,
            .max_out =      14000.0f,
            .max_iout =     2000.0f,
        },
        .ControlTime = 2,
				.follow_dir = -1,
    };

    const Gimbal::GimbalConfig gimbal_config = {
        .imu_serial_port = "/dev/IMU_HERO",
        .yaw_motor_config = Hardware::DJIMotorConfig(6020, "Hero_Gimbal", 1),
        .pitch_motor_config = Hardware::DJIMotorConfig(6020, "Hero_Gimbal", 2),
        .yaw_rate_pid_config = {
            .kp =           16668.f,
            .ki =           234.f,
            .kd =           200.f,
            .max_out =      30000.0f,
            .max_iout =     15000.0f,
        },
                //PITCH SPEED PID 科学调参OK 3.17
        .pitch_rate_pid_config = {
            .kp =           9073.0f,
            .ki =           283.0f,
            .kd =           0.0f,
            .max_out =      30000.0f,
            .max_iout =     15000.0f,
        },
				// PID POSITION RELATIVE
        .yaw_relative_pid_config ={
            .kp =           10.0f,
            .ki =           0.0f,
            .kd =           0.3f,
            .max_out =      10.0f,
            .max_iout =     0.0f,
        },
				// PID POSITION ABSOLUTE 科学调参ok 3.17
        .yaw_absolute_pid_config = {
            .kp =           8.0f,
            .ki =           0.062f,
            .kd =           0.0f,
            .max_out =      60.0f,
            .max_iout =     0.0f,
        },
                //PID PITCH POSITION
        .pitch_absolute_pid_config = {
            .kp =           13.45f,
            .ki =           0.0f,
            .kd =           0.0f,
            .max_out =      10.0f,
            .max_iout =     0.0f,
        },
        .gimbal_motor_dir = 1.0,
        .gimbal_id = 1,
        .ControlTime = 1,
        .YawOffSet = 2114,
        .shoot_config = {
            .left_friction_motor_config = Hardware::DJIMotorConfig{3508, "Hero_Gimbal", 1, 0.075},
            .right_friction_motor_config = Hardware::DJIMotorConfig{3508, "Hero_Gimbal", 2, 0.075},
            .trigger_motor_config = Hardware::DJIMotorConfig{2006, "Hero_Gimbal", 3, 0.075},
            .friction_speed_pid_config = Pid::PidConfig{
                2000.f,       // KP
                0.05f,     // KI
                10.0f,     // KD
                16000.0f,  // MAX_OUT
                2000.0f,   // MAX_IOUT
            },
            .trigger_speed_pid_config = Pid::PidConfig{
                800.0f,    // KP
                0.5f,      // KI
                0.0f,      // KD
                10000.0f,  // MAX_OUT
                9000.0f,   // MAX_IOUT
            },
            .gimbal_id = 1
        },
        .header = 0x6A,
        .auto_aim_ip = "127.0.0.1",
        .auto_aim_port = 11453
    };

    // NOTE: PID CONFIG

    /** CHASSIS **/
    const typename Pid::PidConfig M3508_SPEED_PID_CONFIG{
        15000.0f,  // KP
        10.0f,     // KI
        0.0f,      // KD
        14000.0f,  // MAX_OUT
        2000.0f,   // MAX_IOUT
    };

    const typename Pid::PidConfig CHASSIS_FOLLOW_GIMBAL_PID_CONFIG{
        4.0f,   // KP
        0.0f,   // KI
        10.0f,  // KD
        6.0f,   // MAX_OUT
        0.2f,   // MAX_IOUT
    };

    // TODO Adjust PID parameters
    /** GIMBAL **/
    const typename Pid::PidConfig GIMBAL_YAW_ABSOLUTE_PID_CONFIG{
        12.0f,  // KP
        0.0f,   // KI
        0.3f,   // KD
        10.0f,  // MAX_OUT
        0.0f,   // MAX_IOUT
    };

    const typename Pid::PidConfig GIMBAL_9025_YAW_ABSOLUTE_PID_CONFIG{
        8.0f, 0.0f, 10.0f, 15.0f, 5.0f,
    };

    const typename Pid::PidConfig GIMBAL_PITCH_ABSOLUTE_PID_CONFIG{
        15.0f,  // KP
        0.0f,   // KI
        10.0f,  // KD
        10.0f,  // MAX_OUT
        0.0f,   // MAX_IOUT
    };

    const typename Pid::PidConfig GIMBAL_YAW_RELATIVE_PID_CONFIG{
        8.0f, 0.0f, 0.3f, 10.0f, 0.0f,
    };

    const typename Pid::PidConfig GIMBAL_9025_YAW_RELATIVE_PID_CONFIG{
        3.6f, 0.0f, 8.0f, 15.0f, 0.0f,
    };

    const typename Pid::PidConfig GIMBAL_PITCH_RELATIVE_PID_CONFIG{
        12.0f, 0.0f, 0.0f, 10.0f, 0.0f,
    };

    const typename Pid::PidConfig YAW_SPEED_PID_CONFIG{
        5000.f, 0.0f, 0.f, 20000.0f, 5000.0f,
    };

    const typename Pid::PidConfig YAW_9025_SPEED_PID_CONFIG{
        450.f, 5.0f, 50.f, 850.0f, 250.0f,
    };

    const typename Pid::PidConfig PITCH_SPEED_PID_CONFIG{
        5500.0f, 100.0f, 0.0f, 30000.0f, 5000.0f,
    };

    constexpr fp32 GIMBAL_INIT_YAW_SPEED = 0.005f;
    constexpr fp32 GIMBAL_INIT_PITCH_SPEED = 0.004f;

    // m3508 rmp change to chassis speed,
    // m3508转化成底盘速度(m/s)的比例，
    constexpr fp32 CHASSIS_MOTOR_RPM_TO_VECTOR_SEN = 0.000415809748903494517209f;
    constexpr fp32 SHOOT_MOTOR_RPM_TO_SPEED = 0.00290888208665721596153948461415f;
    constexpr fp32 M6020_ECD_TO_RAD = 2.f * M_PIf / 8192.f;
    constexpr fp32 M9025_ECD_TO_RAD = 2.f * M_PIf / 65535.f;
    constexpr fp32 RPM_TO_RAD_S = 2.f * M_PIf / 60.f;
    constexpr fp32 temp = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN / (RPM_TO_RAD_S / 19.f);
    constexpr fp32 CHASSIS_CONTROL_FREQUENCE = 500.0f;
#define STAND
#ifdef STAND
    constexpr fp32 GIMBAL1_YAW_OFFSET_ECD = 3459;
    constexpr fp32 GIMBAL1_PITCH_OFFSET_ECD = 2194;

    constexpr fp32 GIMBAL2_YAW_OFFSET_ECD = 3366;
    constexpr fp32 GIMBAL2_PITCH_OFFSET_ECD = 3985;

    constexpr fp32 GIMBAL3_YAW_OFFSET_ECD = 48041;
    constexpr fp32 GIMBAL3_PITCH_OFFSET_ECD = 3985;
#else
    constexpr fp32 GIMBAL_YAW_OFFSET_ECD = 5424;
    constexpr fp32 GIMBAL_PITCH_OFFSET_ECD = 618;
#endif

    constexpr uint32_t GIMBAL_INIT_STOP_TIME = 2000;
    constexpr fp32 GIMBAL_INIT_EXP = 0.1f;

    constexpr fp32 FRICTION_MAX_SPEED = 2.5f;
    constexpr fp32 FRICTION_ADD_SPEED = 1.0f;
    constexpr fp32 CONTINUE_TRIGGER_SPEED = 6.f;

    constexpr uint32_t CHASSIS_CONTROL_TIME = 2;
    constexpr uint32_t GIMBAL_CONTROL_TIME = 1;
    constexpr uint32_t SHOOT_CONTROL_TIME = 1;

    constexpr uint32_t DEFAULT_OFFLINE_TIME = 100;

}  // namespace Config
