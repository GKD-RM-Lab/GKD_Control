#include "robot_controller.hpp"

#include "io.hpp"
#include "logger.hpp"
#include "macro_helpers.hpp"
#include "referee.hpp"
#include "robot_type_config.hpp"

namespace Robot
{

    Robot_ctrl::Robot_ctrl()
        : rc_controller(Config::rc_controller_serial),
          gimbal(Config::gimbal_config),
          chassis(Config::chassis_config) IFDEF(
            CONFIG_SENTRY, 
            ,
            gimbal_sentry(Config::gimbal_config)) {
        robot_set = std::make_shared<Robot_set>();
    }

    Robot_ctrl::~Robot_ctrl() = default;

    void Robot_ctrl::start_init() {
        // NOTE: register motors here

        rc_controller.init(robot_set);
        referee.init(robot_set);
        // IFNDEF(CONFIG_SENTRY,
        super_cap.init(Config::super_cap_can_interface, robot_set);
        super_cap.set(true, 30);
        //);
        
        chassis.init(robot_set);
        gimbal.init(robot_set);
        IFDEF(CONFIG_SENTRY, gimbal_sentry.init(robot_set));

        // start DJIMotorManager thread
        Hardware::DJIMotorManager::start();

        threads.emplace_back(&Config::GimbalType::init_task, &gimbal);
        IFDEF(CONFIG_SENTRY, threads.emplace_back(&Gimbal::GimbalT::init_task, &gimbal_sentry));
    }

    void Robot_ctrl::init_join() {
        threads.clear();
    }

    void Robot_ctrl::start() {
        threads.emplace_back(&Config::GimbalType::task, &gimbal);
        threads.emplace_back(&Chassis::Chassis::task, &chassis);
        threads.emplace_back(&Device::Dji_referee::task, &referee);
        threads.emplace_back(&Device::Dji_referee::task_ui, &referee);
        IFDEF(CONFIG_SENTRY, threads.emplace_back(&Gimbal::GimbalT::task, &gimbal_sentry));
        IFDEF(__DEBUG__, threads.emplace_back(&Logger::task, &logger));
    }

    void Robot_ctrl::join() {
        threads.clear();
        std::this_thread::sleep_for(std::chrono::seconds(1000));
    }

    void Robot_ctrl::load_hardware() {
        for (auto& name : Config::CanInitList) {
            IO::io<CAN>.insert(name);
        }
        for (auto& [name, baud_rate, simple_timeout] : Config::SerialInitList) {
            IO::io<SERIAL>.insert(name, baud_rate, simple_timeout);
        }
        for (auto& name : Config::SocketInitList) {
            IO::io<SOCKET>.insert(name);
        }
    }
};  // namespace Robot
