#include "robot_controller.hpp"

namespace Robot
{
    Robot_ctrl::Robot_ctrl() {
        robot_set = std::make_shared<Robot_set>();
    }

    Robot_ctrl::~Robot_ctrl() {
        delete ser1;
        delete ser2;
        delete ser3;
        delete socket_intrf;
    }

    void Robot_ctrl::start_init() {
        imu.init(robot_set);
        cv_controller_.init(robot_set);
		chassis.init(robot_set);
        gimbal.init(robot_set);
		shoot.init(robot_set);
        while (imu.offline()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        gimbal_init_thread = std::make_unique<std::thread>(&Gimbal::Gimbal::init_task, &gimbal);
    }

    void Robot_ctrl::init_join() const {
        if (gimbal_init_thread != nullptr) {
            gimbal_init_thread->join();
        }
    }

    void Robot_ctrl::start() {
		chassis_thread = std::make_unique<std::thread>(&Chassis::Chassis::task, &chassis);
        gimbal_thread = std::make_unique<std::thread>(&Gimbal::Gimbal::task, &gimbal);
	    shoot_thread = std::make_unique<std::thread>(&Shoot::Shoot::task, &shoot);

        // vision_thread = std::make_unique<std::thread>(&Device::Cv_controller::task, &cv_controller_);
    }

    void Robot_ctrl::join() const {
        if (hardware != nullptr) {
            hardware->join();
        }
        if (chassis_thread != nullptr) {
           chassis_thread->join();
        }
        if (gimbal_thread != nullptr) {
            gimbal_thread->join();
        }
    }

    void Robot_ctrl::load_hardware() {
        can0.init("can0");
        can1.init("can1");
        // can2.init("CAN_CHASSIS");
        // can3.init("CAN_BULLET");
        socket_intrf = new Io::Server_socket_interface();
        try {
            ser1 = new Hardware::Serial_interface<Types::ReceivePacket>("/dev/ttyACM0", 115200, 2000);
        } catch (serial::IOException &ex) {
            LOG_ERR("there's no such serial device called ACM0 \n");
        }

        try {
            ser2 = new Hardware::Serial_interface<Types::ReceivePacket>("/dev/tty10", 115200, 2000);
        } catch (serial::IOException &ex) {
            LOG_ERR("there's no such serial device called ACM1 \n");
        }

        try {
            ser3 = new Hardware::Serial_interface<Types::ReceivePacket>("/dev/tty10", 115200, 2000);
        } catch (serial::IOException &ex) {
            LOG_ERR("there's no such serial device called ACM1 \n");
        }

        hardware = std::make_shared<RobotHardware>(can0, can1, can2, can3, ser1, ser2, ser3, socket_intrf, rc_ctrl);

        Robot::hardware->register_callback<SOCKET, Robot::Vison_control>([&](const Robot::Vison_control &vc) {
            robot_set->vx_set = vc.linear_vx;
            robot_set->vy_set = vc.linear_vy;
            robot_set->wz_set = 0;
            robot_set->gimbal1_yaw_set = vc.yaw_set;
        });
    }
};  // namespace Robot
