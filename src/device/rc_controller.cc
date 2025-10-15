#include "device/rc_controller.hpp"
#include "io.hpp"
#include "serial_interface.hpp"
#include "types.hpp"

namespace Device
{

    Rc_Controller::Rc_Controller(const std::string &serial_name) : serial_name(serial_name) {
    }

    void Rc_Controller::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;
        auto serial_interface = IO::io<SERIAL>[serial_name];
        if (serial_interface == nullptr) {
            LOG_ERR("RC_CONTROLLER Error: no serial named %s\n", serial_name.c_str());
            return;
        }
        serial_interface->register_callback<Types::ReceivePacket_RC_CTRL>(
            [&](const Types::ReceivePacket_RC_CTRL &rp) { unpack(rp); });
        delta = 0;
    }

    void Rc_Controller::unpack(const Types::ReceivePacket_RC_CTRL &pkg) {
        if (pkg.s1 == S1_DOWN && pkg.s2 == S2_DOWN && pkg.ch4 == ROLL_UP_MAX) {
            inited = true;
        }

    if(delta == 0) {       
        logger.push_value("rc.ch0",  pkg.ch0);
        logger.push_value("rc.ch1",  pkg.ch1);
        logger.push_value("rc.ch2",  pkg.ch2);
        logger.push_value("rc.ch3",  pkg.ch3);
        logger.push_value("rc.ch4",  pkg.ch4);
        logger.push_value("rc.s1",  pkg.s1);
        logger.push_value("rc.s2",  pkg.s2);
        logger.push_value("rc.mouse_x",  pkg.mouse_x);
        logger.push_value("rc.mouse_y",  pkg.mouse_y);
        logger.push_value("rc.mouse_z",  pkg.mouse_z);
        logger.push_value("rc.mouse_l",  pkg.mouse_l);
        logger.push_value("rc.mouse_r",  pkg.mouse_r);
        logger.push_value("rc.key",  pkg.key);
    } else if(delta == 10) {
        delta = 0;
    }
    delta++;

#ifndef CONFIG_SENTRY 
        float vx = 0, vy = 0;
        float speed = 1;

        if (pkg.key & KEY_D) {
            vx++;
        }
        if (pkg.key & KEY_A) {
            vx--;
        }
        if (pkg.key & KEY_S) {
            vy--;
        }
        if (pkg.key & KEY_W) {
            vy++;
        }

        robot_set->vx_set = vx * speed;
        robot_set->vy_set = vy * speed;

        if (pkg.key) {
            LOG_INFO("key : %d\n", pkg.key);
        }

        static bool wz_key_pressed_last = false;
        static bool friction_key_pressed_last = false;

        // 切换自旋状态
        if (pkg.key & KEY_R) {
            if (!wz_key_pressed_last) {
                robot_set->wz_set = 1 - robot_set->wz_set;
            }
            wz_key_pressed_last = true;
        } else {
            wz_key_pressed_last = false;
        }

        // 切换摩擦轮状态
        if (pkg.key & KEY_F) {
            if (!friction_key_pressed_last) {
                robot_set->friction_open = !robot_set->friction_open;
            }
            friction_key_pressed_last = true;
        } else {
            friction_key_pressed_last = false;
        }


        if (pkg.mouse_r || (pkg.s1 == S1_DOWN && pkg.s2 == S2_UP)) {
            robot_set->auto_aim_status = true;
            // LOG_INFO("auto aim status : %d\n", pkg.s1);
        } else {
            robot_set->auto_aim_status = false;
        }

        if (pkg.mouse_l || pkg.ch4 == ROLL_DOWN_MAX) {
            robot_set->shoot_open = SHOOT_PERMISSION_GIMBAL1;
        } else {
            robot_set->shoot_open = SHOOT_PERMISSION_NONE;
        }

        if (robot_set->auto_aim_status)
            return;

        robot_set->gimbalT_1_yaw_set += pkg.mouse_x / 10000.;

        robot_set->gimbalT_1_pitch_set += pkg.mouse_y / 10000.;

        robot_set->gimbalT_1_pitch_set =
            std::clamp(robot_set->gimbalT_1_pitch_set, -0.3f, 0.3f); 

#endif

        static bool use_key = false;
        if (pkg.key & KEY_PRESS) {
            use_key = true;
        }

        if (use_key) {
            return; 
        }
            

        // auto-aim, disable control
        // if (pkg.s1 == S1_DOWN) {
        //     return; 
        // }
        if (inited) {
            // LOG_INFO("rc controller ch1 %d %d %d %d\n", pkg.s1, pkg.s2, pkg.ch1, pkg.ch3);
            robot_set->vx_set = ((float)pkg.ch3 / RC_SCALE) * CHASSIS_SPEED_SCALE;
            robot_set->vy_set = ((float)pkg.ch2 / RC_SCALE) * CHASSIS_SPEED_SCALE;

            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_SEARCH) {
                robot_set->gimbal_sentry_yaw_set += ((float)pkg.ch0 / RC_SCALE) * GIMBAL_YAW_SENSITIVITY;
            } else {
                robot_set->gimbalT_1_yaw_set += ((float)pkg.ch0 / RC_SCALE) * GIMBAL_YAW_SENSITIVITY;
                robot_set->gimbalT_1_pitch_set = ((float)pkg.ch1 / RC_SCALE) * GIMBAL_PITCH_SENSITIVITY;
                
                IFDEF(CONFIG_SENTRY, robot_set->gimbalT_2_yaw_set = robot_set->gimbalT_1_yaw_set;
                    robot_set->gimbalT_2_pitch_set = robot_set->gimbalT_1_pitch_set;)
            }

            if (pkg.s1 == S1_UP)
                robot_set->wz_set = 1.0;
            else
                robot_set->wz_set = 0;

            if (pkg.s2 == S2_UP)
                robot_set->friction_open = true;
            else
                robot_set->friction_open = false;

            IFDEF(
                CONFIG_SENTRY,
                if (pkg.s2 == S2_DOWN) {
                    robot_set->sentry_follow_gimbal = true;
                    robot_set->friction_open = true;
                    if (pkg.ch4 == ROLL_DOWN_MAX)
                        robot_set->shoot_open = SHOOT_PERMISSION_BOTH;
                    else
                        robot_set->shoot_open = SHOOT_PERMISSION_NONE;
                } else {
                    robot_set->sentry_follow_gimbal = false;
                    robot_set->friction_open = false;
                })
        }
        update_time();
    }
}  // namespace Device
