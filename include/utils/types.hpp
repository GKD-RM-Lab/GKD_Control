#ifndef __TYPES__
#define __TYPES__
#include <linux/can.h>

#include <cstdint>

#include "macro_helpers.hpp"
#include "protocol.hpp"
#include "string"

// NOTE: defines and type defines
typedef signed char int8_t;
typedef signed short int int16_t;
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

namespace Status
{
    const bool OK = true;
    const bool ERROR = false;
}  // namespace Status

struct Vec3d
{
    double x, y, z;
};

namespace Types
{
    typedef struct ReceivePacket_IMU
    {
        float yaw;
        float pitch;
        float roll;
        float yaw_v;
        float pitch_v;
        float roll_v;
    } __attribute__((packed)) ReceivePacket_IMU;

    typedef struct ReceivePacket_RC_CTRL
    {
        int ch0;
        int ch1;
        int ch2;
        int ch3;
        int ch4;
        int s1;
        int s2;
        int mouse_x;
        int mouse_y;
        int mouse_z;
        int mouse_l;
        int mouse_r;
        int key;
    } __attribute__((packed)) ReceivePacket_RC_CTRL;

    struct ReceivePacket_Super_Cap
    {
        uint8_t errorCode;
        float chassisPower;
        uint16_t chassisPowerlimit;
        uint8_t capEnergy;
    } __attribute__((packed));

    struct Referee_info
    {
        Referee::GameStatus game_status_data;
        Referee::GameResult game_result_ref;
        Referee::RefereeWarning referee_warning_ref;
        Referee::GameRobotStatus game_robot_status_data;
        Referee::BulletAllowance bullet_allowance_data;
        Referee::PowerHeatData power_heat_data;
    };

    typedef struct
    {
        fp32 input;         // 输入数据
        fp32 out;           // 滤波输出的数据
        fp32 num[1];        // 滤波参数
        fp32 frame_period;  // 滤波的时间间隔 单位 s
    } first_order_filter_type_t;

    enum Kb_event
    {
        UP = 0,
        DOWN,
        LEFT,
        RIGHT,
        SPIN_R,
        SPIN_L,
        STOP_X,
        STOP_Y,
    };

    enum Init_status
    {
        INIT_FINISH = MUXDEF(CONFIG_SENTRY, 0x7, 0x1),
    };

    typedef struct
    {
        fp32 vx;
        fp32 vy;
        fp32 wz;
        fp32 wheel_speed[4];
        can_frame can_f;
        uint64_t pkg;
        bool err;
        std::string debuginfo1;
        std::string debuginfo2;
        std::string debuginfo3;

    } debug_info_t;

    typedef struct
    {
        struct
        {
            int16_t ch[5];
            char s[2];
        } rc;
        struct
        {
            int16_t x;
            int16_t y;
            int16_t z;
            uint8_t press_l;
            uint8_t press_r;
        } mouse;
        struct
        {
            bool q;  //  spin clock wise
            bool f;  //  spin counter clock wise
            fp32 speed_x;
            fp32 speed_y;
            int16_t v;
        } key;

    } RC_ctrl_t;
    enum ROBOT_MODE
    {
        ROBOT_NO_FORCE,
        ROBOT_FINISH_INIT,
        ROBOT_FOLLOW_GIMBAL,
        ROBOT_SEARCH,
        ROBOT_IDLE,
        ROBOT_NOT_FOLLOW
    };

    typedef struct
    {
        fp32 input;         // 输入数据
        fp32 out;           // 滤波输出的数据
        fp32 num;           // 滤波参数
        fp32 frame_period;  // 滤波的时间间隔 单位 s
    } fof_t;

    typedef struct
    {
    } ramp_t;
}  // namespace Types

#endif
