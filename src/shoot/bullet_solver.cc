#include "bullet_solver.hpp"

#include "cmath"

namespace Control
{
    BulletSolver::BulletSolver() {
        config_ = { .resistance_coff_qd_10 = 0.45,
                    .resistance_coff_qd_15 = 1.0,
                    .resistance_coff_qd_16 = 0.7,
                    .resistance_coff_qd_18 = 0.55,
                    .resistance_coff_qd_30 = 5.0,
                    .g = 9.81 };
        max_track_target_vel_ = 5.0;
    }

    double BulletSolver::getResistanceCoefficient(double bullet_speed) const {
        // bullet_speed have 5 value:10,15,16,18,30
        double resistance_coff;
        if (bullet_speed < 12.5)
            resistance_coff = config_.resistance_coff_qd_10;
        else if (bullet_speed < 15.5)
            resistance_coff = config_.resistance_coff_qd_15;
        else if (bullet_speed < 17)
            resistance_coff = config_.resistance_coff_qd_16;
        else if (bullet_speed < 24)
            resistance_coff = config_.resistance_coff_qd_18;
        else
            resistance_coff = config_.resistance_coff_qd_30;
        return resistance_coff;
    }

    bool BulletSolver::solve(
        Vec3d pos,
        Vec3d vel,
        double bullet_speed,
        double yaw,
        double v_yaw,
        double r1,
        double r2,
        double dz,
        int armors_num) {
        bullet_speed_ = bullet_speed;
        resistance_coff_ =
            getResistanceCoefficient(bullet_speed_) != 0 ? getResistanceCoefficient(bullet_speed_) : 0.001;

        double temp_z = pos.z;
        double target_rho = std::sqrt(std::pow(pos.x, 2) + std::pow(pos.y, 2));
        output_yaw_ = std::atan2(pos.y, pos.x);
        output_pitch_ = std::atan2(temp_z, std::sqrt(std::pow(pos.x, 2) + std::pow(pos.y, 2)));
        double rough_fly_time =
            (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) /
            resistance_coff_;
        selected_armor_ = 0;
        double r = r1;
        double z = pos.z;
        track_target_ = std::abs(v_yaw) < max_track_target_vel_;
        double switch_armor_angle =
            track_target_ ? acos(r / target_rho) - M_PI / 12 +
                                (-acos(r / target_rho) + M_PI / 6) * std::abs(v_yaw) / max_track_target_vel_
                          : M_PI / 12;
        if ((((yaw + v_yaw * rough_fly_time) > output_yaw_ + switch_armor_angle) && v_yaw > 0.) ||
            (((yaw + v_yaw * rough_fly_time) < output_yaw_ - switch_armor_angle) && v_yaw < 0.)) {
            selected_armor_ = v_yaw > 0. ? -1 : 1;
            r = armors_num == 4 ? r2 : r1;
            z = armors_num == 4 ? pos.z + dz : pos.z;
        }
        int count{};
        double error = 999;
        if (track_target_) {
            target_pos_.x = pos.x - r * cos(yaw + selected_armor_ * 2 * M_PI / armors_num);
            target_pos_.y = pos.y - r * sin(yaw + selected_armor_ * 2 * M_PI / armors_num);
        } else {
            target_pos_.x = pos.x - r * cos(atan2(pos.y, pos.x));
            target_pos_.y = pos.y - r * sin(atan2(pos.y, pos.x));
        }
        target_pos_.z = z;
        while (error >= 0.001) {
            output_yaw_ = std::atan2(target_pos_.y, target_pos_.x);
            output_pitch_ = std::atan2(temp_z, std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2)));
            target_rho = std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2));
            fly_time_ = (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) /
                        resistance_coff_;
            double real_z = (bullet_speed_ * std::sin(output_pitch_) + (config_.g / resistance_coff_)) *
                                (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_ -
                            config_.g * fly_time_ / resistance_coff_;

            if (track_target_) {
                target_pos_.x = pos.x + vel.x * fly_time_ -
                                r * cos(yaw + v_yaw * fly_time_ + selected_armor_ * 2 * M_PI / armors_num);
                target_pos_.y = pos.y + vel.y * fly_time_ -
                                r * sin(yaw + v_yaw * fly_time_ + selected_armor_ * 2 * M_PI / armors_num);
            } else {
                double target_pos_after_fly_time[2];
                target_pos_after_fly_time[0] = pos.x + vel.x * fly_time_;
                target_pos_after_fly_time[1] = pos.y + vel.y * fly_time_;
                target_pos_.x = target_pos_after_fly_time[0] -
                                r * cos(atan2(target_pos_after_fly_time[1], target_pos_after_fly_time[0]));
                target_pos_.y = target_pos_after_fly_time[1] -
                                r * sin(atan2(target_pos_after_fly_time[1], target_pos_after_fly_time[0]));
            }
            target_pos_.z = z + vel.z * fly_time_;

            double target_yaw = std::atan2(target_pos_.y, target_pos_.x);
            double error_theta = target_yaw - output_yaw_;
            double error_z = target_pos_.z - real_z;
            temp_z += error_z;
            error = std::sqrt(std::pow(error_theta * target_rho, 2) + std::pow(error_z, 2));
            count++;

            if (count >= 20 || std::isnan(error))
                return false;
        }
        return true;
    }
}  // namespace Control
