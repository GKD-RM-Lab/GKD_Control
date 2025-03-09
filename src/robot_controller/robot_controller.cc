#include "robot_controller.hpp"
#include CONFIGHPP
#include "io.hpp"

namespace Robot
{

    Robot_ctrl::Robot_ctrl()
        : rc_controller("/dev/IMU_BIG_YAW"),
          gimbal(Config::gimbal_config),
          chassis(Config::chassis_config) {
        robot_set = std::make_shared<Robot_set>();
    }

    Robot_ctrl::~Robot_ctrl() = default;

    void Robot_ctrl::start_init() {
        // NOTE: register motors here

        // cv_controller_.init(robot_set);
        rc_controller.enable();
        // super_cap.init("Hero_Chassis");

        // chassis.init(robot_set);
        gimbal.init(robot_set);

        // start DJIMotorManager thread
        Hardware::DJIMotorManager::start();

        threads.emplace_back(&Config::GimbalType::init_task, &gimbal);
    }

    void Robot_ctrl::init_join() {
        threads.clear();
    }

    void Robot_ctrl::start() {
        threads.emplace_back(&Config::GimbalType::task, &gimbal);
        // threads.emplace_back(&Chassis::Chassis::task, &chassis);
        // vision_thread = std::make_unique<std::thread>(&Device::Cv_controller::task, &cv_controller_);
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
        for(auto & name : Config :: SocketInitList) {
            IO::io<SOCKET>.insert(name);
            IO::io<SOCKET>[name]->register_callback<Vison_control>([this](const Vison_control& vc){
                LOG_INFO("%f %f\n", vc.yaw_set, vc.pitch_set);
                robot_set->gimbalT_1_yaw_set += vc.yaw_set;
            });
        }

    }
};  // namespace Robot
