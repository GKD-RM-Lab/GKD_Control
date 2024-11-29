#include <user_lib.hpp>

#include "robot_controller.hpp"
#include "utils.hpp"

using namespace std::chrono;

int main(int argc, char **argv) {

    Robot::Robot_ctrl robot;

    robot.load_hardware();
    robot.start_init();
    robot.init_join();
    // LOG_INFO("init finished!\n");
    std::thread out_thread([&]() {
        while (true) {
            std::cout << robot.robot_set->gyro1_ins_yaw << ' ' << robot.robot_set->gyro1_ins_pitch << ' '
            << ' ' << robot.robot_set->gyro1_ins_roll << std::endl;
            UserLib::sleep_ms(1);
        }
    });

    robot.robot_set->set_mode(Types::ROBOT_MODE::ROBOT_FINISH_INIT);
    robot.robot_set->vx_set = 0.1;

    robot.start();
    robot.join();
    return 0;
}
