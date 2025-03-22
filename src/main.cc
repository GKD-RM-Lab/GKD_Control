#include <csignal>
#include <user_lib.hpp>

#include "robot_controller.hpp"
#include "utils.hpp"

const auto shoot_config = Shoot::ShootConfig{
    .left_friction_motor_config = Hardware::DJIMotorConfig{3508, "can0", 1, 0.075},
    .right_friction_motor_config = Hardware::DJIMotorConfig{3508, "can0", 2, 0.075},
    .trigger_motor_config = Hardware::DJIMotorConfig{2006, "can0", 3, 0.075},
    .friction_speed_pid_config = Pid::PidConfig{
        2000.f,       // KP
        0.05f,     // KI
        10.0f,     // KD
        16000.0f,  // MAX_OUT
        2000.0f,   // MAX_IOUT
    },
    .trigger_speed_pid_config = Pid::PidConfig{
        1000.0f,    // KP
        2.5f,      // KI
        0.0f,      // KD
        10000.0f,  // MAX_OUT
        9000.0f,   // MAX_IOUT
    }
};

int main(int argc, char **argv) {
    IO::io<CAN>.insert("can0");
    Hardware::DJIMotorManager::start();
    Shoot::Shoot shoot(shoot_config);
    auto robot_set = std::make_shared<Robot::Robot_set>();
    shoot.init(robot_set);
    robot_set->mode = Types::ROBOT_FOLLOW_GIMBAL;
    std::thread shoot_thread([&](){shoot.task();});
    while(1) {
        int cmd = 0;
        std::cin >> cmd;
        if(cmd == 1) {
            robot_set->shoot_open ^= 1;
            std::cout << "shoot " << (robot_set->shoot_open ? "open" : "closed") << std::endl;
        } else if(cmd == 2) {
            robot_set->friction_open ^= 1;
            std::cout << "friction " << (robot_set->friction_open ? "open" : "closed") << std::endl;
        }
    }
    shoot_thread.join();
    return 0;
}
