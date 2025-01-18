#include "device/cv_controller.hpp"
#include "hardware.hpp"
#include "utils.hpp"

namespace Device
{
    void Cv_controller::init(const std::shared_ptr<Robot::Robot_set>& robot) {
        robot_set = robot;
        Robot::hardware->register_callback<SOCKET, Robot::ReceiveGimbalPacket>(
            [&](const Robot::ReceiveGimbalPacket& pkg) { unpack(pkg); });
    }

    [[noreturn]] void Cv_controller::task() {
        while (true) {
            Robot::SendGimbalPacket sgp;
            sgp.detect_color = 0;
            sgp.reserved = 0;
            sgp.reset_tracker = false;
            sgp.header = 0x5A;
            sgp.yaw = -robot_set->gimbal2_yaw_abs;
            sgp.pitch = -robot_set->gyro2_ins_pitch;
            sgp.roll = robot_set->gyro2_ins_roll;
            sgp.aim_x = robot_set->aimx;
            sgp.aim_y = robot_set->aimy;
            sgp.aim_z = robot_set->aimz;
            Robot::hardware->send<SOCKET>(sgp);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void Cv_controller::unpack(const Robot::ReceiveGimbalPacket& pkg) {
				 //LOG_INFO("cv unpacking %f %f\n", pkg.x, pkg.y);
				//float y = (float)pkg.sd_yaw / 32767, p = -(float)pkg.sd_pitch / 62767;
				//if(pkg.sd_left_trigger != 0){
				//  robot_set->friction_open = true;
				//}else{
				//  robot_set->friction_open = false;
				//}
				//if(pkg.sd_up != 0){
				//  robot_set->shoot_open = true;
				//}else{
				//  robot_set->shoot_open = false;
				//}

				//if (pkg.sd_right == 1) {
				//  LOG_INFO("steam deck right\n");
				//    robot_set->wz_set = 1.5;
				//} else {
				//    robot_set->wz_set = 0.f;
				//}

				//if (robot_set->mode == Types::ROBOT_MODE::ROBOT_FOLLOW_GIMBAL) {
				//    robot_set->gimbal1_yaw_offset += ((float)pkg.sd_mx / 18) * 0.02;
				//    robot_set->gimbal2_yaw_offset += ((float)pkg.sd_mx / 18) * 0.02;
				//    control_ramp.update(y);
				//    control_ramp2.update(p);
				//    if (y < 1.5 && y > -1.5) {
				//        robot_set->gimbal1_yaw_set = control_ramp.out + robot_set->gimbal1_yaw_offset;
				//        robot_set->gimbal2_yaw_set = control_ramp.out + robot_set->gimbal2_yaw_offset;
				//    }
				//    if (p < 0.15 && p > -0.47) {
				//        robot_set->gimbal1_pitch_set = control_ramp2.out;
				//        robot_set->gimbal2_pitch_set = control_ramp2.out;
				//    }
				//} else if (robot_set->mode == Types::ROBOT_MODE::ROBOT_SEARCH) {
				//    robot_set->gimbal3_yaw_set += ((float)pkg.sd_mx / 18) * 0.02;
				//}

				//if (pkg.sd_a == 1) {
				//    robot_set->set_mode(Types::ROBOT_MODE::ROBOT_FOLLOW_GIMBAL);
				//} else if (pkg.sd_b == 1) {
				//    robot_set->set_mode(Types::ROBOT_MODE::ROBOT_SEARCH);
				//} else if (pkg.sd_x == 1) {
				//    exit(0);
				//}

				//robot_set->vx_set = -(float)pkg.sd_vy / 32767;
				//robot_set->vy_set = (float)pkg.sd_vx / 32767;

				 if (pkg.x != 0) {
						 auto solver_successful = bullet_solver_.solve(
								 { pkg.x, pkg.y, pkg.z },
								 { pkg.vx, pkg.vy, pkg.vz },
								 17,
								 pkg.yaw,
								 pkg.v_yaw,
								 pkg.r1,
								 pkg.r2,
								 pkg.dz,
								 pkg.armors_num);

						 if (solver_successful) {
								 robot_set->gimbal2_yaw_set = -(fp32)bullet_solver_.getYaw() + 0.25;
								 robot_set->gimbal2_pitch_set = -(fp32)bullet_solver_.getPitch() - 0.25;
								 LOG_INFO("%f yaw set %f pitch set %f\n",robot_set->gimbal2_yaw_set, -(fp32)bullet_solver_.getYaw(),(fp32)bullet_solver_.getPitch());
								 robot_set->aimx = (fp32)bullet_solver_.target_pos_.x;
								 robot_set->aimy = (fp32)bullet_solver_.target_pos_.y;
								 robot_set->aimz = (fp32)bullet_solver_.target_pos_.z;
						 }
				 }

        // if (pkg.tracking) {
        //     robot_set->is_aiming = true;
        // } else {
        //     robot_set->is_aiming = false;
        // }
        update_time();
    }

}  // namespace Device
