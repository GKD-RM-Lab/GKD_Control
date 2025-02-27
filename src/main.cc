#include <user_lib.hpp>
#include <csignal>

#include "device/M9025.hpp"
#include "robot_controller.hpp"
#include "utils.hpp"

using namespace std::chrono;
Device::M9025 motor("CAN_CHASSIS", 1);

void test(int sig) {
    std::cout << "Interrupt signal (" << sig << ") received.\n";
    UserLib::sleep_ms(1);
    motor.set(0);
    UserLib::sleep_ms(1);
    motor.set(0);
    exit(0);
}

int main(int argc, char **argv) {
    signal(SIGINT, test);
    signal(SIGTERM, test);
    signal(SIGSEGV, test);
    signal(SIGABRT, test);
    signal(SIGHUP, test);

    Robot::Robot_ctrl robot;

    robot.load_hardware();

		robot.start_init();
		robot.init_join();
		LOG_INFO("init finished!\n");

    // robot.robot_set->set_mode(Types::ROBOT_MODE::ROBOT_FINISH_INIT);
    // robot.robot_set->vx_set = 0.1;
    //
    // robot.start();
    // robot.join();

    return 0;
}
