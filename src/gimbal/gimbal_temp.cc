#include "gimbal/gimbal_temp.hpp"

#include <algorithm>
#include <cstddef>

#include "UI.hpp"
#include "gimbal/gimbal_config.hpp"
#include "macro_helpers.hpp"
#include "robot_controller.hpp"
#include "robot_type_config.hpp"
#include "serial/serial.h"
#include "types.hpp"
#include "user_lib.hpp"
#include "utils.hpp"
namespace Gimbal
{
    GimbalT::GimbalT(const GimbalConfig &config)
        : config(config),
          imu(config.imu_serial_port),
          yaw_motor(config.yaw_motor_config),
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

        // yaw_motor.setCtrl(Pid::PidPosition(config.yaw_rate_pid_config, yaw_gyro));
        // pitch_motor.setCtrl(
        //     Pid::PidPosition(config.pitch_rate_pid_config, pitch_gyro) >>
        //     Pid::Invert(config.gimbal_motor_dir));

        yaw_relative_pid = Pid::PidRad(config.yaw_relative_pid_config, yaw_relative);
        MUXDEF(
            CONFIG_SENTRY,
            yaw_absolute_pid =
                Pid::PidRad(config.yaw_absolute_pid_config, fake_yaw_abs) >> Pid::Invert(-1),
            yaw_absolute_pid =
                Pid::PidRad(config.yaw_absolute_pid_config, imu.yaw) >> Pid::Invert(-1));

        pitch_absolute_pid = Pid::PidRad(config.pitch_absolute_pid_config, imu.pitch);

        imu.enable();
        yaw_motor.enable();
        pitch_motor.enable();

        IO::io<SOCKET>["AUTO_AIM_CONTROL"]->add_client(
            config.header, config.auto_aim_ip, config.auto_aim_port);

        IO::io<SOCKET>["AUTO_AIM_CONTROL"]->register_callback_key(
            config.header, [this](const Robot::Auto_aim_control &vc) {
                LOG_INFO("socket recive %f %f %d %d\n",vc.yaw_set,vc.pitch_set,vc.fire,config.gimbal_id);
                receive_auto_aim = std::chrono::steady_clock::now();
                if (robot_set->auto_aim_status) {
                    if (vc.fire == false)
                    return;
                robot_set->set_mode(Types::ROBOT_MODE::ROBOT_FOLLOW_GIMBAL);
                robot_set->cv_fire = true;
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
                    robot_set->shoot_open &= ~config.gimbal_id;
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
       test_yaw_speed_pid();
       // test_yaw_position_pid();
    }

    void GimbalT::test_pitch_speed_pid(){}
    void GimbalT::test_pitch_position_pid(){}
    void GimbalT::test_yaw_speed_pid(){
        FILE *fp = fopen("/home/gkd/test_sentry/log/yaw_speed_data.csv", "w");
    
        if (fp == NULL) {
            LOG_INFO("Error: Cannot open file! Check path.\n");
        } 

        static uint32_t tick_ms = 0; 
        
        float output_current = 0.0f;
        const float BASE_CURRENT = 0.0f; 
        float STEP_AMP = 0.0f; 
        
        const int CYCLE_PERIOD = 4000; 

        while (1) {
            update_data(); 
            int phase_time = tick_ms % CYCLE_PERIOD;

            if (phase_time < 1000) {
                output_current = BASE_CURRENT;
            } 
            else if (phase_time < 2000) {
                output_current = BASE_CURRENT + STEP_AMP; 
            } 
            else if (phase_time < 2500) {
                output_current = BASE_CURRENT;
            } 
            else if (phase_time < 3500){
                output_current = BASE_CURRENT - STEP_AMP;
            }else {
                output_current = BASE_CURRENT;
            }

            if (output_current > 10000.0f) output_current = 10000.0f;
            if (output_current < -10000.0f) output_current = -10000.0f;

            output_current >> yaw_motor;

            if(fp != NULL) {
                fprintf(fp, "%.4f,%.4f,%.4f\n", 
                (float)tick_ms , 
                output_current,           
                imu.pitch_rate          
                );
                if (tick_ms % 100 == 0) fflush(fp);
            }

            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
            tick_ms += Config::GIMBAL_CONTROL_TIME;
        }
    }
    void GimbalT::test_yaw_position_pid(){
        // remember yaw_motor.setCtrl
        FILE *fp = fopen("/home/gkd/test_sentry/log/yaw_position_data.csv", "w");
    
        if (fp == NULL) {
            LOG_INFO("Error: Cannot open file! Check path.\n");
        } 

        static uint32_t tick_ms = 0; 
        // current here means imu_rate
        float output_current = 0.0f;
        const float BASE_CURRENT = 0.0f; 
        float STEP_AMP = 0.0f; 
        
        const int CYCLE_PERIOD = 4000; 

        while (1) {
            update_data(); 
            int phase_time = tick_ms % CYCLE_PERIOD;

            if (phase_time < 1000) {
                output_current = BASE_CURRENT;
            } 
            else if (phase_time < 2000) {
                output_current = BASE_CURRENT + STEP_AMP; 
            } 
            else if (phase_time < 3000) {
                output_current = BASE_CURRENT;
            } 
            else if (phase_time < 3500){
                output_current = BASE_CURRENT - STEP_AMP;
            }else {
                output_current = BASE_CURRENT;
            }

            output_current >> yaw_relative_pid >> yaw_motor;

            if(fp != NULL) {
                fprintf(fp, "%.4f,%.4f,%.4f\n", 
                (float)tick_ms , 
                imu.pitch_rate,           
                imu.pitch          
                );
                if (tick_ms % 100 == 0) fflush(fp);
            }

            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
            tick_ms += Config::GIMBAL_CONTROL_TIME;
        }
    }


    [[noreturn]] void GimbalT::task() {
        std::jthread shoot_thread(&Shoot::Shoot::task, &shoot);
        while (true) {
            update_data();
            // LOG_INFO("%d: yaw set %f, imu yaw %f\n", config.header, *yaw_set, imu.yaw);
            // logger.push_value("gimbal.yaw.set", (double)*yaw_set);
            // logger.push_value("gimbal.yaw.imu", (double)imu.yaw);
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
                //LOG_INFO("mode:%d\n", robot_set->mode);
                // LOG_INFO("%f\n", *pitch_set);
                *pitch_set >> pitch_absolute_pid >> pitch_motor;
                //LOG_INFO("status::%d\n", robot_set->auto_aim_status);
            }
            // if (config.gimbal_id == 1)
            // LOG_INFO("%dpitch set %f\n", config.gimbal_id, *pitch_set);
            // LOG_INFO("robot id % d\n", robot_set->referee_info.game_robot_status_data.robot_id);
            Robot::SendAutoAimInfo pkg;
            pkg.header = config.header;
            MUXDEF(CONFIG_SENTRY, pkg.yaw = fake_yaw_abs, pkg.yaw = imu.yaw);
            pkg.pitch = imu.pitch;
            pkg.red = robot_set->referee_info.game_robot_status_data.robot_id < 100;
            IO::io<SOCKET>["AUTO_AIM_CONTROL"]->send(pkg);

            UserLib::sleep_ms(config.ControlTime);
        }
    }

    void GimbalT::update_data() {
        yaw_relative = UserLib::rad_format(
            yaw_motor.data_.rotor_angle - Hardware::DJIMotor::ECD_8192_TO_RAD * config.YawOffSet);
        yaw_gyro = (std::cos(imu.pitch) * imu.yaw_rate - std::sin(imu.pitch) * imu.roll_rate);
        pitch_gyro = imu.pitch_rate;
        // gimbal sentry follow needs
        *yaw_rela = yaw_relative;
        fake_yaw_abs = robot_set->gimbal_sentry_yaw - yaw_relative;
    }

}  // namespace Gimbal
