#ifndef __ROBOT__
#define __ROBOT__
#include "types.hpp"

namespace Robot
{
    // robot set header = 0xEA;
    struct Robot_set
    {
        uint8_t header;
        /** chassis_control **/
        fp32 vx_set = 0.f;
        fp32 vy_set = 0.f;
        fp32 wz_set = 0.f;
        bool spin_state = false;

        /** gimbal_control **/
        fp32 gimbal1_yaw_set = 0.f;
        fp32 gimbal1_yaw_offset = 0.f;
        fp32 gimbal1_pitch_set = 0.f;

        fp32 gimbal2_yaw_set = 0.f;
        fp32 gimbal2_yaw_offset = 0.f;
        fp32 gimbal2_pitch_set = 0.f;

        fp32 gimbal3_yaw_set = 0.f;
        fp32 gimbal3_yaw_offset = 0.f;
        fp32 gimbal3_pitch_set = 0.f;

        /** shoot_control **/
        bool friction_open = false;
        bool friction_real_state =
            false;  // friction's real state (motor linear speed < 0.5 ? false : true)
        bool cv_fire = false;
        int shoot_open = 0;

        /** other **/
        fp32 gimbalT_1_yaw_set = 0.f;
        fp32 gimbalT_1_pitch_set = 0.f;
        fp32 gimbalT_1_yaw_reletive = 0.f;


        fp32 gimbal_sentry_yaw_set = 0.f;
        fp32 gimbal_sentry_yaw = 0.f;
        fp32 gimbal_sentry_yaw_reletive = 0.f;

        fp32 aimx;
        fp32 aimy;
        fp32 aimz;
        bool is_aiming = false;
        uint8_t inited = 0;

        uint8_t sentry_follow_gimbal = 0;

        bool auto_aim_status = false;

        Types::ROBOT_MODE mode = Types::ROBOT_MODE::ROBOT_NO_FORCE;
        Types::ROBOT_MODE last_mode = Types::ROBOT_MODE::ROBOT_NO_FORCE;

        Types::ReceivePacket_Super_Cap super_cap_info;
        Types::Referee_info referee_info;

        void set_mode(Types::ROBOT_MODE set_mode) {
            this->last_mode = this->mode;
            this->mode = set_mode;
        }

        bool mode_changed() {
            if (this->last_mode != this->mode) {
                this->last_mode = this->mode;
                return true;
            }
            return false;
        }

        void sync_head() {
        }
    };

    // send gimbal package header = 0x5A;
    struct ReceiveGimbalPacket
    {
        /*      自瞄部分     */
        uint8_t header;
        bool tracking : 1;
        uint8_t id : 3;
        uint8_t armors_num : 3;
        uint8_t reserved : 1;
        int16_t sd_yaw;
        int16_t sd_pitch;
        int16_t sd_vx;
        int16_t sd_vy;
        uint8_t sd_a;
        uint8_t sd_b;
        int8_t sd_mx;
        int8_t sd_my;
        uint8_t sd_x;
        uint8_t sd_y;
        uint8_t sd_ltb;
        uint8_t sd_rtb;
        float x;
        float y;
        float z;
        float yaw;
        float vx;
        float vy;
        float vz;
        float v_yaw;
        float r1;
        float r2;
        float dz;
        /*  决策部分   */

    } __attribute__((packed));

    struct SendGimbalPacket
    {
        uint8_t header = 0x5A;
        uint8_t detect_color : 1;  // 0-red 1-blue
        bool reset_tracker : 1;
        uint8_t reserved : 6;
        float roll;
        float pitch;
        float yaw;
        float aim_x;
        float aim_y;
        float aim_z;
        uint16_t checksum = 0;
    } __attribute__((packed));

    // send twist package heaedr = 0x6A;
    struct Auto_aim_control
    {
        uint8_t header;
        float yaw_set;
        float pitch_set;
        bool fire;

        Types::ROBOT_MODE mode = Types::ROBOT_MODE::ROBOT_NO_FORCE;
    } __attribute__((packed));

    struct SendAutoAimInfo
    {
        uint8_t header;
        float yaw;
        float pitch;
        bool red;
    } __attribute__((packed));

    struct SendVisionControl
    {
        uint8_t header = 0xA6;
        float roll;
        float pitch;
        float yaw;
    } __attribute__((packed));

    struct ReceiveNavigationInfo
    {
        uint8_t header;
        float vx;
        float vy;
    } __attribute__((packed));

    struct SendNavigationInfo
    {
        uint8_t header;
        float yaw;
        float pitch;
        float hp;
        bool start;
    } __attribute__((packed));

}  // namespace Robot

#endif
