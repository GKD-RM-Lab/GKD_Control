#pragma once

#include "can.hpp"
#include "hardware_manager.hpp"
#include "rc_ctrl.hpp"
#include "serial_interface.hpp"
#include "socket_interface.hpp"

namespace Robot
{
    using RobotHardware = Hardware::Hardware_manager<
        Hardware::Can_interface,
        Hardware::Can_interface,
        Hardware::Can_interface,
        Hardware::Can_interface,
        Hardware::Serial_interface<Types::ReceivePacket>,
        Hardware::Serial_interface<Types::ReceivePacket>,
        Hardware::Serial_interface<Types::ReceivePacket>,
        Io::Server_socket_interface,
        Io::Rc_ctrl>;

    extern std::shared_ptr<RobotHardware> hardware;
}  // namespace Robot
