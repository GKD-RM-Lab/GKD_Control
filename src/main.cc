#include <csignal>
#include <user_lib.hpp>
#include "logger.hpp"
#include "robot_controller.hpp"
#include "types.hpp"
#include "utils.hpp"

int main(int argc, char **argv) {
    Robot::Robot_ctrl robot;

    logger.set_level(log_level::Debug);

    robot.load_hardware();

    robot.start_init();
    // robot.robot_set->inited = Types::Init_status::INIT_FINISH;
    robot.init_join();
    LOG_INFO("init finished!\n");
    robot.robot_set->set_mode(Types::ROBOT_FOLLOW_GIMBAL);

    robot.start();
    robot.join();

    return 0;
}
