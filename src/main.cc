#include <csignal>
#include <user_lib.hpp>
#include "robot_controller.hpp"
#include "utils.hpp"

int main(int argc, char **argv) {
    Robot::Robot_ctrl robot;

    robot.load_hardware();

    robot.start_init();
    robot.init_join();
    LOG_INFO("init finished!\n");

    robot.robot_set->set_mode(Types::ROBOT_MODE::ROBOT_FOLLOW_GIMBAL);

    robot.start();
    robot.join();

    return 0;
}
