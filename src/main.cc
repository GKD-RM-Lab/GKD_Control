#include <user_lib.hpp>

#include "robot_controller.hpp"
#include "utils.hpp"

using namespace std::chrono;

int main(int argc, char **argv) {

    Robot::Robot_ctrl robot;

    robot.load_hardware();
    robot.start_init();
    std::thread out_thread([&]() {
        while (true) {
            std::cout << robot.gimbal.imu.yaw << ' ' << robot.gimbal.imu.pitch << std::endl;
            UserLib::sleep_ms(1);
        }
    });
    robot.init_join();
    std::cout << "finish" << std::endl;
    // LOG_INFO("init finished!\n");

    robot.robot_set->set_mode(Types::ROBOT_MODE::ROBOT_FINISH_INIT);
    robot.robot_set->vx_set = 0.1;
    //
    robot.start();
    robot.join();
    out_thread.join();
    return 0;
}
