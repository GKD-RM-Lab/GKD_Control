#include "gimbal/gimbal_sentry.hpp"

#include <cmath>
#include <cstdio>
#include <future>

#include "logger.hpp"
#include "robot_type_config.hpp"
#include "socket_interface.hpp"
#include "types.hpp"
#include "user_lib.hpp"
#include "utils.hpp"

namespace Gimbal
{
    GimbalSentry::GimbalSentry(const GimbalConfig& config)
        : config(config),
          imu(config.imu_serial_port),
          yaw_motor("CAN_GIMBAL", 1)

    {
    }

    void GimbalSentry::init(const std::shared_ptr<Robot::Robot_set>& robot) {
        robot_set = robot;

        yaw_absolute_pid = Pid::PidRad(config.yaw_absolute_pid_config, imu.yaw) >> Pid::Invert(-1);
        yaw_relative_pid = Pid::PidRad(config.yaw_relative_pid_config, yaw_relative);


        yaw_motor.setCtrl(Pid::PidPosition(config.yaw_rate_pid_config, imu.yaw_rate));
        yaw_set = &robot_set->gimbal_sentry_yaw_set;

        imu.enable();
        yaw_motor.enable();

        IO::io<SOCKET>["AUTO_AIM_CONTROL"] -> register_callback([this](const
        Robot::ReceiveNavigationInfo& vc) {
            LOG_INFO("navigation receive %f %f\n", vc.vx, vc.vy);
            robot_set->vx_set = vc.vx;
            robot_set->vy_set = vc.vy;
            std::cout << "robot_set_vx" << robot_set->vx_set << std::endl;
        });

        IO::io<SOCKET>["AUTO_AIM_CONTROL"]->add_client(0x37, "127.0.0.1", 11456);
    }

    void GimbalSentry::init_task() {
        test_yaw_speed_pid();
        // test_yaw_position_pid();
        while (robot_set->inited != Types::Init_status::INIT_FINISH) {
            // LOG_INFO("robot_set->inited:%d\n", robot_set->inited);
            update_data();
            // 0.f >> yaw_absolute_pid >> yaw_motor;
            0.f >> yaw_relative_pid >> yaw_motor;
            *yaw_set = imu.yaw;
            if (fabs(yaw_relative) < Config::GIMBAL_INIT_EXP) {
                init_stop_times += 1;
            } else {
                init_stop_times = 0;
            }

            // if (init_stop_times >= static_cast<int>(Config::GIMBAL_INIT_STOP_TIME))
            robot_set->inited |= 1 << 1;



            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
    }

    [[noreturn]] void GimbalSentry::task() {
        last_mode = robot_set->mode;
        while (true) {
            // LOG_INFO("set:%d\n", robot_set->mode);
            update_data();

            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                yaw_motor.set(0);
            } else if (robot_set->mode == Types::ROBOT_MODE::ROBOT_SEARCH) {
                if(last_mode != Types::ROBOT_MODE::ROBOT_SEARCH) {
                    search_start_time = std::chrono::steady_clock::now();
                    search_start_yaw = imu.yaw;
                }

                auto duration = std::chrono::duration<double>(std::chrono::steady_clock::now() - search_start_time);

                fp32 yaw = search_start_yaw + duration.count() * config.search_yaw_speed;
                yaw >> yaw_absolute_pid >> yaw_motor;

            } else {
                robot_set->gimbal_sentry_yaw_set >> yaw_absolute_pid >> yaw_motor;
                // LOG_INFO("yaw_set:%f\n", robot_set->gimbal_sentry_yaw_set);
            }

            last_mode = robot_set->mode;

            Robot::SendNavigationInfo gimbal_info;
            gimbal_info.header = 0x37;
            gimbal_info.yaw = imu.yaw;
            gimbal_info.hp = robot_set->referee_info.game_robot_status_data.remain_hp * 1. /
                             robot_set->referee_info.game_robot_status_data.max_hp;
            gimbal_info.start =
                (robot_set->referee_info.game_status_data.game_progress & 0x0f) == 4;

            // FIXME: random robot_set used
            if (gimbal_info.start) {
                // robot_set->wz_set = 0.3;
                // robot_set->friction_open = true;
            }
            // LOG_INFO("game progress %d\n", robot_set->referee_info.game_status_data.game_progress
            // & 0x0f); 
            IO::io<SOCKET>["AUTO_AIM_CONTROL"]->send(gimbal_info);

            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
        }
    }

    void GimbalSentry::test_yaw_speed_pid(){
        FILE *fp = fopen("/home/gkd/GKD_Control/speed.csv", "w");
    
        if (fp == NULL) {
            LOG_INFO("Error: Cannot open file! Check path.\n");
        } 

        static uint32_t tick_ms = 0; 
        //output_current is current
        float output_current = 0.0f;
        const float BASE_CURRENT = 500.0f; 
        float STEP_AMP = 1100.0f; 
        
        const int CYCLE_PERIOD = 4000; 

        while (1) {
            update_data(); 
            int phase_time = tick_ms % CYCLE_PERIOD;

            if (phase_time < 1000) {
                output_current = BASE_CURRENT;  //500
            } 
            else if (phase_time < 2000) {
                output_current = BASE_CURRENT + STEP_AMP;  //1500
            } 
            else if (phase_time < 2500) {
                output_current = BASE_CURRENT;  //500
            } 
            else if (phase_time < 3500){
                output_current = BASE_CURRENT - STEP_AMP;  //-500
            }else {
                output_current = BASE_CURRENT;    //500
            }

            if (output_current > 10000.0f) output_current = 10000.0f;
            if (output_current < -10000.0f) output_current = -10000.0f;

            // output_current >> yaw_motor;

            // LOG_INFO("output : %f",output_current);
            LOG_INFO("output : %f",imu.yaw_rate);
            1 >> yaw_motor;


            if(fp != NULL) {
                fprintf(fp, "%.4f,%.4f,%.4f\n", 
                (float)tick_ms , 
                output_current,           
                imu.yaw_rate          
                );
                if (tick_ms % 100 == 0) fflush(fp);
            }

            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
            tick_ms += Config::GIMBAL_CONTROL_TIME;
        }
    }
    
    void GimbalSentry::test_yaw_position_pid(){
        // remember yaw_motor.setCtrl
        FILE *fp = fopen("/home/gkd/GKD_Control/pos.csv", "w");
    
        if (fp == NULL) {
            LOG_INFO("Error: Cannot open file! Check path.\n");
        } 

        static uint32_t tick_ms = 0; 
        // current here means imu_rate
        float output_current = 0.0f;
        const float BASE_CURRENT = 4.0f; 
        float STEP_AMP = 2.0f; 
        
        const int CYCLE_PERIOD = 4500; 

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
            else if (phase_time < 4000){
                output_current = BASE_CURRENT - STEP_AMP;
            }else {
                output_current = BASE_CURRENT;
            }

            // 1 >> yaw_absolute_pid >> yaw_motor;
            output_current >> yaw_motor;

            if(fp != NULL) {
                fprintf(fp, "%.4f,%.4f,%.4f\n", 
                (float)tick_ms , 
                output_current,           
                imu.yaw          
                );
                if (tick_ms % 100 == 0) fflush(fp);
            }

            LOG_INFO("speed : %f",imu.yaw_rate);


            UserLib::sleep_ms(Config::GIMBAL_CONTROL_TIME);
            tick_ms += Config::GIMBAL_CONTROL_TIME;
        }
    }

    void GimbalSentry::update_data() {
        yaw_motor_speed = Config::RPM_TO_RAD_S * (fp32)yaw_motor.motor_measure.speed_rpm;
        yaw_relative = UserLib::rad_format(
            Config::M9025_ECD_TO_RAD *
            ((fp32)yaw_motor.motor_measure.ecd - Config::GIMBAL3_YAW_OFFSET_ECD));
    
        LOG_INFO("yaw relative:%d",(int)yaw_motor.motor_measure.ecd);
        
        // LOG_INFO("gimbal:%f\n", robot_set->gimbal_sentry_yaw_reletive);
        // LOG_INFO("imu:%f\n", imu.yaw);



        //LOG_INFO("yaw_motor.motor_measure.ecd:%d", yaw_motor.motor_measure.ecd);
        yaw_relative_with_head = robot_set->gimbalT_1_yaw_reletive;
        robot_set->gimbal_sentry_yaw_reletive = yaw_relative;
        robot_set->gimbal_sentry_yaw = imu.yaw;
        // LOG_INFO("mode:%d\n", robot_set->mode);
    }
}  // namespace Gimbal
