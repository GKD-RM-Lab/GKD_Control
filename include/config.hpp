#pragma once
#include "pid_controller.hpp"
#include "types.hpp"

namespace Config
{
    const char CAN_CHANNEL[] = "can1";

    // NOTE: PID CONFIG

    /** CHASSIS **/
    const typename Pid::Pid_config M3508_SPEED_PID_CONFIG{
        15000.0f,  // KP
        10.0f,     // KI
        0.0f,      // KD
        14000.0f,  // MAX_OUT
        2000.0f,   // MAX_IOUT
    };

    const typename Pid::Pid_config CHASSIS_FOLLOW_GIMBAL_PID_CONFIG{
        4.0f,   // KP
        0.0f,    // KI
        10.0f,  // KD
        6.0f,    // MAX_OUT
        0.2f,    // MAX_IOUT
    };

    //TODO Adjust PID parameters
    /** GIMBAL **/
    const typename Pid::Pid_config GIMBAL_YAW_ABSOLUTE_PID_CONFIG{
        12.0f,  // KP
        0.0f,   // KI
        0.0f,   // KD
        10.0f,  // MAX_OUT
        0.0f,   // MAX_IOUT
    };

    const typename Pid::Pid_config GIMBAL_9025_YAW_ABSOLUTE_PID_CONFIG{
        8.0f,
        0.0f,
        10.0f,
        15.0f,
        5.0f,
    };

    const typename Pid::Pid_config GIMBAL_PITCH_ABSOLUTE_PID_CONFIG{
        15.0f,  // KP
        0.0f,   // KI
        10.0f,   // KD
        10.0f,  // MAX_OUT
        0.0f,   // MAX_IOUT
    };

    const typename Pid::Pid_config GIMBAL_YAW_RELATIVE_PID_CONFIG{
        15.0f,
        0.0f,
        0.3f,
        10.0f,
        0.0f,
    };

    const typename Pid::Pid_config GIMBAL_9025_YAW_RELATIVE_PID_CONFIG{
        3.6f,
        0.0f,
        8.0f,
        15.0f,
        0.0f,
    };

    const typename Pid::Pid_config GIMBAL_PITCH_RELATIVE_PID_CONFIG{
        12.0f,
        0.0f,
        0.0f,
        10.0f,
        0.0f,
    };

    const typename Pid::Pid_config YAW_SPEED_PID_CONFIG{
        4500.f,
        0.0f,
        0.f,
        20000.0f,
        5000.0f,
    };

    const typename Pid::Pid_config YAW_9025_SPEED_PID_CONFIG{
        450.f,
        0.0f,
        50.f,
        850.0f,
        0.0f,
    };

    const typename Pid::Pid_config PITCH_SPEED_PID_CONFIG{
        4500.0f,
        100.0f,
        0.0f,
        30000.0f,
        5000.0f,
    };

    /** SHOOT **/
    const typename Pid::Pid_config FRIC_SPEED_PID_CONFIG{
        5.f,  // KP
        0.05f,     // KI
        10.0f,      // KD
        16000.0f,  // MAX_OUT
        2000.0f,   // MAX_IOUT
    };

    const typename Pid::Pid_config TRIGGER_SPEED_PID_CONFIG{
        800.0f,  // KP
        0.5f,     // KI
        0.0f,      // KD
        10000.0f,  // MAX_OUT
        9000.0f,   // MAX_IOUT
    };
    /** shoot config **/

    constexpr uint32_t FRICTION_NUM = 4;
    constexpr uint32_t TRIGGER_NUM = 2;

    constexpr fp32 GIMBAL_INIT_YAW_SPEED = 0.005f;
    constexpr fp32 GIMBAL_INIT_PITCH_SPEED = 0.004f;

    // m3508 rmp change to chassis speed,
    // m3508转化成底盘速度(m/s)的比例，
    constexpr fp32 CHASSIS_MOTOR_RPM_TO_VECTOR_SEN = 0.000415809748903494517209f;
    constexpr fp32 SHOOT_MOTOR_RPM_TO_SPEED = 0.00290888208665721596153948461415f;
    constexpr fp32 M6020_ECD_TO_RAD = 2.f * M_PIf / 8192.f;
    constexpr fp32 M9025_ECD_TO_RAD = 2.f * M_PIf / 65535.f;
    constexpr fp32 RPM_TO_RAD_S = 2.f * M_PIf / 60.f;
    constexpr fp32 CHASSIS_CONTROL_FREQUENCE = 500.0f;
#define STAND
#ifdef STAND
    constexpr fp32 GIMBAL1_YAW_OFFSET_ECD = 6064;
    constexpr fp32 GIMBAL1_PITCH_OFFSET_ECD = 6162;

    constexpr fp32 GIMBAL2_YAW_OFFSET_ECD = 6121;
    constexpr fp32 GIMBAL2_PITCH_OFFSET_ECD = 3985;

    constexpr fp32 GIMBAL3_YAW_OFFSET_ECD = 47563;
    constexpr fp32 GIMBAL3_PITCH_OFFSET_ECD = 3985;
#else
    constexpr fp32 GIMBAL_YAW_OFFSET_ECD = 5424;
    constexpr fp32 GIMBAL_PITCH_OFFSET_ECD = 618;
#endif

    constexpr uint32_t GIMBAL_INIT_STOP_TIME = 1500;
    constexpr fp32 GIMBAL_INIT_EXP = 0.1f;

    constexpr fp32 FRICTION_MAX_SPEED = 4850.f;
    constexpr fp32 FRICTION_ADD_SPEED = 2500.f;
    constexpr fp32 CONTINUE_TRIGGER_SPEED = 9.f;

    constexpr uint32_t CHASSIS_CONTROL_TIME = 2;
    constexpr uint32_t GIMBAL_CONTROL_TIME = 1;
    constexpr uint32_t SHOOT_CONTROL_TIME = 1;

    constexpr uint32_t DEFAULT_OFFLINE_TIME = 100;

}  // namespace BulletSolverConfig
