#include "gimbal/gimbal_temp.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sys/stat.h>

#include "UI.hpp"
#include "gimbal/gimbal_config.hpp"
#include "macro_helpers.hpp"
#include "robot_controller.hpp"
#include "robot_type_config.hpp"
#include "serial/serial.h"
#include "types.hpp"
#include "user_lib.hpp"
#include "utils.hpp"
#include <cmath>

namespace {

std::mutex g_imu_log_mtx;
std::once_flag g_imu_log_once;
std::ofstream g_imu_log_ofs;
std::chrono::steady_clock::time_point g_imu_log_t0;
std::chrono::steady_clock::time_point g_imu_log_last;

void imu_log_init() {
    // Resolve relative to current working directory (normally project root when running).
    ::mkdir("../../../../log", 0755);  // ignore EEXIST and other non-fatal errors
    g_imu_log_ofs.open("../../../../log/imu.txt", std::ios::out | std::ios::trunc);
    if (g_imu_log_ofs.is_open()) {
        g_imu_log_ofs << "# t_s,id,pitch_set_rad,yaw_set_rad,pitch_rad,yaw_rad,"
                         "pitch_rate_rad_s,yaw_rate_rad_s\n";
        g_imu_log_ofs << std::fixed << std::setprecision(6);
    }
    g_imu_log_t0 = std::chrono::steady_clock::now();
    g_imu_log_last = g_imu_log_t0;
}

void imu_log_write(
    int gimbal_id,
    float pitch_set_rad,
    float yaw_set_rad,
    float pitch_rad,
    float yaw_rad,
    float pitch_rate_rad_s,
    float yaw_rate_rad_s) {
    std::call_once(g_imu_log_once, imu_log_init);
    if (!g_imu_log_ofs.is_open()) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lk(g_imu_log_mtx);
    // Default: ~100Hz logging (adjust if needed).
    if (now - g_imu_log_last < std::chrono::milliseconds(10)) {
        return;
    }
    g_imu_log_last = now;

    const double t_s =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - g_imu_log_t0).count();
    g_imu_log_ofs << t_s << "," << gimbal_id << "," << pitch_set_rad << ","
                  << yaw_set_rad << "," << pitch_rad << "," << yaw_rad << ","
                  << pitch_rate_rad_s << "," << yaw_rate_rad_s << "\n";
    // Keep data visible even if the process exits unexpectedly during debug.
    g_imu_log_ofs.flush();
}
}  // namespace

namespace Gimbal
{
    GimbalT::GimbalT(const GimbalConfig &config)
        : config(config),
          imu_yaw(config.imu_serial_port),
          imu_pitch(config.imu_serial_port_pitch.empty() ? config.imu_serial_port
                                                        : config.imu_serial_port_pitch),
#ifdef CONFIG_INFANTRY
          yaw_motor(config.yaw_motor_config.can_name_,
                    config.yaw_motor_config.id_,
                    config.yaw_command_header),
#else
          yaw_motor(config.yaw_motor_config),
#endif
          pitch_motor(config.pitch_motor_config),
          yaw_set(nullptr),
          another_yaw_set(nullptr),
          another_pitch_set(nullptr),
          pitch_set(nullptr),
          yaw_rela(nullptr),
          shoot(config.shoot_config) {
        receive_auto_aim = std::chrono::steady_clock::now();
    }

    void GimbalT::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;
        shoot.init(robot);
        if (config.gimbal_id == 1) {
            yaw_set = &robot_set->gimbalT_1_yaw_set;
            another_yaw_set = &robot_set->gimbalT_2_yaw_set;
            another_pitch_set = &robot_set->gimbalT_2_pitch_set;
            pitch_set = &robot_set->gimbalT_1_pitch_set;
            yaw_rela = &robot_set->gimbalT_1_yaw_reletive;
        } else {
            yaw_set = &robot_set->gimbalT_2_yaw_set;
            another_yaw_set = &robot_set->gimbalT_1_yaw_set;
            another_pitch_set = &robot_set->gimbalT_1_pitch_set;
            pitch_set = &robot_set->gimbalT_2_pitch_set;
            yaw_rela = &robot_set->gimbalT_2_yaw_reletive;
        }

        yaw_motor.setCtrl(Pid::PidPosition(config.yaw_rate_pid_config, yaw_gyro)); //速度环     
        pitch_motor.setCtrl(
            Pid::PidPosition(config.pitch_rate_pid_config, pitch_gyro) >>
            Pid::Invert(config.gimbal_motor_dir));

        yaw_relative_pid = Pid::PidRad(config.yaw_relative_pid_config, yaw_relative);
        MUXDEF(
            CONFIG_SENTRY,
            yaw_absolute_pid =
                Pid::PidRad(config.yaw_absolute_pid_config, fake_yaw_abs) >> Pid::Invert(-1),
            yaw_absolute_pid =
                Pid::PidRad(config.yaw_absolute_pid_config, imu_yaw.yaw) ) >> Pid::Invert(-1);   //位置环

        pitch_absolute_pid = Pid::PidRad(config.pitch_absolute_pid_config, imu_pitch.pitch);

        imu_yaw.enable();
        imu_pitch.enable();
        yaw_motor.enable();
        pitch_motor.enable();

        IO::io<SOCKET>["AUTO_AIM_CONTROL"]->add_client(
            config.header, config.auto_aim_ip, config.auto_aim_port);

        IO::io<SOCKET>["AUTO_AIM_CONTROL"]->register_callback_key(
            config.header, [this](const Robot::Auto_aim_control &vc) {
                // LOG_INFO("socket recive %f %f %d %d\n",vc.yaw_set,vc.pitch_set,vc.fire,config.gimbal_id);
                receive_auto_aim = std::chrono::steady_clock::now();
                if (robot_set->auto_aim_status) {
                    robot_set->set_mode(Types::ROBOT_MODE::ROBOT_FOLLOW_GIMBAL);
                    robot_set->cv_fire = vc.fire;
                    if (vc.fire && ISDEF(CONFIG_SENTRY)) {
                        robot_set->shoot_open |= config.gimbal_id;
                    }

                    if ((robot_set->shoot_open & (3 - config.gimbal_id)) == 0) {
                        *another_yaw_set = vc.yaw_set;
                        *another_pitch_set = vc.pitch_set;
                    }
                    // LOG_INFO("status:%d\n",robot_set->auto_aim_status);
                    // LOG_INFO("yaw:%f,pitch:%f\n",vc.yaw_set,vc.pitch_set);
                    // if (!ISDEF(CONFIG_SENTRY) && !robot_set->auto_aim_status)
                    //     return;
                    *yaw_set = vc.yaw_set;
                    *pitch_set = vc.pitch_set;
                }
            });

        std::thread check_auto_aim([this] {
            while (true) {
                if (robot_set->sentry_follow_gimbal) {
                    IFDEF(
                        CONFIG_SENTRY, robot_set->set_mode(Types::ROBOT_MODE::ROBOT_FOLLOW_GIMBAL));
                    continue;
                }
                if (std::chrono::steady_clock::now() - receive_auto_aim >
                    std::chrono::milliseconds(300)) {
                    if (robot_set->auto_aim_status || robot_set->cv_fire) {
                        robot_set->shoot_open &= ~config.gimbal_id;
                    }
                    robot_set->cv_fire = false;
                }
                // LOG_INFO("shoot open %d\n", robot_set->shoot_open);
                if (robot_set->shoot_open == 0) {
                    IFDEF(CONFIG_SENTRY, robot_set->set_mode(Types::ROBOT_MODE::ROBOT_SEARCH));
                }
                UserLib::sleep_ms(10);
            }
        });

        check_auto_aim.detach();
    }

    void GimbalT::init_task() {
        static int delta = 0;
        while ((imu_yaw.offline() && imu_pitch.offline()) || yaw_motor.offline() || pitch_motor.offline()) {
            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
            LOG_INFO(
                "status: imu_yaw:%s | imu_pitch:%s | yaw:%s | pitch:%s\n",
                (imu_yaw.offline() == 1) ? "off" : "on",
                (imu_pitch.offline() == 1) ? "off" : "on",
                (yaw_motor.offline() == 1) ? "off" : "on",
                (pitch_motor.offline() == 1) ? "off" : "on");
            delta++;
            if (delta > 1000)
                exit(-1);
        }
        static const auto runtime_begin = std::chrono::steady_clock::now();
       
       
        while (robot_set->inited != Types::Init_status::INIT_FINISH) {
        // while(1) {
             const auto elapsed_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::steady_clock::now() - runtime_begin) .count();
            update_data();
            if (config.gimbal_id == 2) {
                robot_set->inited |= 1 << 1;
            }
            // delta++;
            //  if (delta > 10000)
                // exit(-1);

            // if ( elapsed_milliseconds % 4000 < 2000 ){
            //     -1.f >> yaw_absolute_pid >> yaw_motor;
            // } else if(  elapsed_milliseconds % 4000 > 2000){
            //     -1.8f >> yaw_absolute_pid >> yaw_motor;
            // }

            // if ( elapsed_milliseconds % 4000 < 2000 ){
            //     -0.2f >> yaw_motor;
            // } else if(  elapsed_milliseconds % 4000 > 2000){
            //     0.2f >> yaw_motor;
            // }
            // 0.0f >> yaw_motor;


            // -1.f >> yaw_relative_pid >> yaw_motor;

            // 1.f >> yaw_motor;
            0.f >> yaw_relative_pid >> yaw_motor;
            0.f >> pitch_absolute_pid >> pitch_motor;
            
            // 速度环 阶跃
            // int max_current = 5000;
            // if (elapsed_milliseconds % 3600 < 200) {
            //     yaw_motor.set(0);
            // } else if (elapsed_milliseconds % 3600 > 200 && elapsed_milliseconds % 3600 < 1200) {
            //     yaw_motor.set(max_current);
            // } else if (elapsed_milliseconds % 3600 > 1200 && elapsed_milliseconds % 3600 < 1800) {
            //     yaw_motor.set(-max_current);
            // } else if (elapsed_milliseconds % 3600 > 1800 && elapsed_milliseconds % 3600 < 2000) {
            //     yaw_motor.set(0);
            // } else if (elapsed_milliseconds % 3600 > 2000 && elapsed_milliseconds % 3600 < 2800) {
            //     yaw_motor.set(max_current*0.5);
            // } else if (elapsed_milliseconds % 3600 > 2800 && elapsed_milliseconds % 3600 < 3600) {
            //     yaw_motor.set(-max_current*0.5);
            // }

            // 速度环 正弦
            // yaw_motor.set(std::sin(2.0 * std::acos(-1.0) * elapsed_milliseconds / 1500.0) * max_current);

            // float max_speed = 4.4;
            // float speed_target = 0;

            // if (elapsed_milliseconds % 3600*0.5 < 200*0.5) {
            //     yaw_motor.set(0);
            //     speed_target = 0;
            // } else if (elapsed_milliseconds % 3600*0.5 > 200*0.5 && elapsed_milliseconds % 3600*0.5 < 1200*0.5) {
            //     yaw_motor.set(max_speed);
            //     speed_target = max_speed;
            // } else if (elapsed_milliseconds % 3600*0.5 > 1200*0.5 && elapsed_milliseconds % 3600*0.5 < 1800*0.5) {
            //     yaw_motor.set(-max_speed);
            //     speed_target = -max_speed;
            // } else if (elapsed_milliseconds % 3600*0.5 > 1800*0.5 && elapsed_milliseconds % 3600*0.5 < 2000*0.5) {
            //     yaw_motor.set(0);
            //     speed_target = 0;
            // } else if (elapsed_milliseconds % 3600*0.5 > 2000*0.5 && elapsed_milliseconds % 3600*0.5 < 2800*0.5) {
            //     yaw_motor.set(max_speed*0.1);
            //     speed_target = max_speed*0.1;
            // } else if (elapsed_milliseconds % 3600*0.5 > 2800*0.5 && elapsed_milliseconds % 3600*0.5 < 3600*0.5) {
            //     yaw_motor.set(-max_speed*0.1);
            //     speed_target = -max_speed*0.1;
            // }

            // if (elapsed_milliseconds % 600 < 300) {
            //     yaw_motor.set(max_speed);
            //     speed_target = max_speed;
            // } else{
            //     yaw_motor.set(-max_speed);
            //     speed_target = -max_speed;
            // }

            // YAW 速度环 科学调参采集
            // printf("%ld,%d,%f\n", elapsed_milliseconds, yaw_motor.give_current, yaw_gyro);

            // YAW 位置环
            // printf("%ld,%f,%f\n", elapsed_milliseconds, speed_target, imu_yaw.yaw);
            // printf("%d\n",yaw_motor.motor_measure_.ecd);
            // LOG_INFO(
            //    "imu : %6f %6f %6f %6d\n",
            //    imu.yaw,
            //    imu.pitch,
            //    imu.roll,
            //    yaw_motor.motor_measure_.ecd);

            if (fabs(yaw_relative) < Config::GIMBAL_INIT_EXP &&
                fabs(imu_pitch.pitch) < Config::GIMBAL_INIT_EXP) {
                init_stop_times += 1;
            } else {
                init_stop_times = 0;
            }

            MUXDEF(CONFIG_SENTRY, *yaw_set = robot_set->gimbal_sentry_yaw, *yaw_set = imu_yaw.yaw);
            *pitch_set = 0;

            if (init_stop_times >= Config::GIMBAL_INIT_STOP_TIME) {
                if (config.gimbal_id == 1)
                    robot_set->inited |= 1;
                else
                    robot_set->inited |= 1 << 1;
            }
            UserLib::sleep_ms(config.ControlTime);
        }
    }

    [[noreturn]] void GimbalT::task() {
        std::jthread shoot_thread(&Shoot::Shoot::task, &shoot);
        while (true) {
            update_data();
            // LOG_INFO("%d: yaw set %f, imu yaw %f\n", config.header, *yaw_set, imu.yaw);
            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                yaw_motor.give_current = 0;
                pitch_motor.give_current = 0;
            } else if (robot_set->mode == Types::ROBOT_MODE::ROBOT_SEARCH) {
                static float delta = 0;
                static float delta_1 = 0;

                float yaw = (sin(delta) - 1) * (M_PIf / 2);
                float pitch = sin(delta_1) * 0.30 + 0.165;
                delta += 0.001;
                delta_1 += 0.003;

                if (config.gimbal_id == 1) {
                    yaw >> yaw_relative_pid >> yaw_motor;
                } else {
                    -yaw >> yaw_relative_pid >> yaw_motor;
                }
                *pitch_set = std::clamp((double)pitch, -0.18, 0.51);
                *pitch_set >> pitch_absolute_pid >> pitch_motor;
            } else {
                // NOTE: 抽象双头限位
                MUXDEF(
                    CONFIG_SENTRY, static float yr; static flz
                    if (config.gimbal_id == 1 && (yr < -2.6 || yr > 0.5)) {
                        if (yr > 0)
                            ty = robot_set->gimbal_sentry_yaw - (0.5);
                        else
                            ty = robot_set->gimbal_sentry_yaw - (-2.6);
                    } else if (config.gimbal_id == 2 && (yr < -0.5 || yr > 2.6)) {
                        if (yr > 0)
                            ty = robot_set->gimbal_sentry_yaw - 2.6;
                        else
                            ty = robot_set->gimbal_sentry_yaw - (-0.5);
                    } else { ty = *yaw_set; }

                    ty >>
                    yaw_absolute_pid >> yaw_motor;
                    , *yaw_set >> yaw_absolute_pid >> yaw_motor;)
                // LOG_INFO("yaw set %f, imu yaw %f\n", *yaw_set, imu_yaw.yaw);
                //LOG_INFO("mode:%d\n", robot_set->mode);
                // LOG_INFO("%f\n", *pitch_set);
#ifdef CONFIG_INFANTRY
                *pitch_set = std::clamp(*pitch_set, -0.2f, 0.275f);
#endif
                *pitch_set >> pitch_absolute_pid >> pitch_motor;
                //LOG_INFO("status::%d\n", robot_set->auto_aim_status);
            }
            // if (config.gimbal_id == 1)
            // LOG_INFO("%dpitch set %f\n", config.gimbal_id, *pitch_set);
            // LOG_INFO("robot id % d\n", robot_set->referee_info.game_robot_status_data.robot_id);
            Robot::SendAutoAimInfo pkg;
            pkg.header = config.header;
            MUXDEF(CONFIG_SENTRY, pkg.yaw = fake_yaw_abs, pkg.yaw = imu_yaw.yaw);
            pkg.pitch = imu_pitch.pitch;
            pkg.roll = imu_yaw.roll;
            pkg.red = robot_set->referee_info.game_robot_status_data.robot_id < 100;
            // LOG_INFO("%s\n", (pkg.red == 1) ? "red" : "blue" );
            // LOG_INFO("pkg.roll:%f\n", pkg.roll);
            IO::io<SOCKET>["AUTO_AIM_CONTROL"]->send(pkg);



            UserLib::sleep_ms(config.ControlTime);
        }
    }

    void GimbalT::update_data() {
        if (imu_yaw.offline() && !imu_pitch.offline()) {
            imu_yaw.yaw = imu_pitch.yaw;
            imu_yaw.pitch = imu_pitch.pitch;
            imu_yaw.roll = imu_pitch.roll;
            imu_yaw.yaw_rate = imu_pitch.yaw_rate;
            imu_yaw.pitch_rate = imu_pitch.pitch_rate;
            imu_yaw.roll_rate = imu_pitch.roll_rate;
        }
#ifdef CONFIG_INFANTRY
        constexpr float kYawEcdToRad = Device::M9025::ECD_65535_TO_RAD;
#else
        constexpr float kYawEcdToRad = Hardware::DJIMotor::ECD_8192_TO_RAD;
#endif
        yaw_relative = UserLib::rad_format(
            yaw_motor.data_.rotor_angle - kYawEcdToRad * config.YawOffSet);
        yaw_gyro = (std::cos(imu_pitch.pitch) * imu_yaw.yaw_rate -
                    std::sin(imu_pitch.pitch) * imu_yaw.roll_rate);
        pitch_gyro = imu_pitch.pitch_rate;
        // auto newYawOffSet = yaw_motor.data_.rotor_angle / Hardware::DJIMotor::ECD_8192_TO_RAD;
        // LOG_INFO("Yawoffset:%f\n", newYawOffSet);
        // gimbal sentry follow needs
        // LOG_INFO("imu.pitch:%f | pitch_set:%f\n", imu_pitch.pitch, robot_set->gimbalT_1_pitch_set);        //  LOG_INFO("imu.yaw:%f\n", imu_yaw.yaw);
        // LOG_INFO("imu.yaw:%f | yaw_set:%f\n", imu_yaw.yaw, robot_set->gimbalT_1_yaw_set);
        // LOG_INFO("imu.pitch_rate:%f\n", imu.pitch_rate);
        // LOG_INFO("imu.yaw_rate:%f\n", imu_yaw.yaw_rate);
        imu_log_write(
            config.gimbal_id,
            *pitch_set,
            *yaw_set,
            imu_pitch.pitch,
            imu_yaw.yaw,
            imu_pitch.pitch_rate,
            imu_yaw.yaw_rate);
        *yaw_rela = yaw_relative;
        fake_yaw_abs = robot_set->gimbal_sentry_yaw - yaw_relative;
    }

}  // namespace Gimbal
