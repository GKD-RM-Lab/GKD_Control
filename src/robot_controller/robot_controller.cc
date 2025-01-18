#include "robot_controller.hpp"

namespace Robot
{
    Robot_ctrl::Robot_ctrl() {
        robot_set = std::make_shared<Robot_set>();
    }

    Robot_ctrl::~Robot_ctrl() {
        delete ser1;
    }

    void Robot_ctrl::start_init() {
        imu.init(robot_set);
        cv_controller_.init(robot_set);
        // chassis.init(robot_set);
        // gimbal.init(robot_set);
        shoot.init(robot_set);
        while (imu.offline()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        gimbal_init_thread = std::make_unique<std::thread>(&Gimbal::Gimbal::init_task, &gimbal);
    }

    void Robot_ctrl::init_join() const {
        // if (gimbal_init_thread != nullptr) {
        //     gimbal_init_thread->join();
        // }
    }

    void Robot_ctrl::start() {
        // chassis_thread = std::make_unique<std::thread>(&Chassis::Chassis::task, &chassis);
        // gimbal_thread = std::make_unique<std::thread>(&Gimbal::Gimbal::task, &gimbal);
        shoot_thread = std::make_unique<std::thread>(&Shoot::Shoot::task, &shoot);

        // vision_thread = std::make_unique<std::thread>(&Device::Cv_controller::task, &cv_controller_);
    }

    void Robot_ctrl::join() const {
        // if (hardware != nullptr) {
        //     hardware->join();
        // }
        // if (chassis_thread != nullptr) {
        //     chassis_thread->join();
        // }
        // if (gimbal_thread != nullptr) {
        //     gimbal_thread->join();
        // }
    }

    void Robot_ctrl::load_hardware() {
        can0.init("can0");
        can1.init("can1");
        socket_intrf = new Io::Server_socket_interface();
        try {
            ser1 = new Hardware::Serial_interface<Types::ReceivePacket>("/dev/ttyACM0", 115200, 1000);
        } catch (serial::IOException &ex) {
            LOG_ERR("there's no such serial device\n");
        }

        hardware = std::make_shared<RobotHardware>(can0, can1, ser1, socket_intrf, rc_ctrl);

        Robot::hardware->register_callback<SOCKET, Robot::Vison_control>([&](const Robot::Vison_control &vc) {
            robot_set->vx_set = vc.linear_vx;
            robot_set->vy_set = vc.linear_vy;
            robot_set->wz_set = 0;
            robot_set->yaw_set = vc.yaw_set;
        });
    }
};  // namespace Robot
