#include "robot_controller.hpp"
#include "utils.hpp"

using namespace std::chrono;


int main(int argc, char **argv) {
    Robot::Robot_ctrl robot;

    robot.load_hardware();
    robot.start_init();
    robot.init_join();
    LOG_INFO("init finished!\n");

    robot.robot_set->set_mode(Types::ROBOT_MODE::ROBOT_FOLLOW_GIMBAL);
    // robot.robot_set->vx_set = 1;
    // robot.robot_set->vy_set = 1;
    robot.robot_set->shoot_open = 1;
    // robot.robot_set->friction_open = 1;
    robot.robot_set->gimbal1_pitch_set = 0;
    robot.robot_set->gimbal1_yaw_set = 0;
    robot.robot_set->wz_set = 1;

    robot.start();
    robot.join();

    return 0;
}
