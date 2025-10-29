#include "serial_interface.hpp"

#include "user_lib.hpp"
#include "utils.hpp"

namespace IO
{
    Serial_interface::Serial_interface(std::string port_name, int baudrate, int simple_timeout)
        : serial::Serial(port_name, baudrate, serial::Timeout::simpleTimeout(simple_timeout)),
          name(port_name) {
    }

    Serial_interface::~Serial_interface() = default;

    inline void Serial_interface::enumerate_ports() {
        std::vector<serial::PortInfo> devices_found = serial::list_ports();
        auto iter = devices_found.begin();

        while (iter != devices_found.end()) {
            serial::PortInfo device = *iter++;
            LOG_INFO("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str());
        }
    }

    inline int Serial_interface::unpack(uint8_t pkg_id) {
        if (pkg_id == 1) {
            memcpy(buffer, read(sizeof(Types::ReceivePacket_IMU)).c_str(), sizeof(Types::ReceivePacket_IMU));
            UserLib::fromVector(buffer, &imu_pkg);
            callback(imu_pkg);
        } else if (pkg_id == 2) {
            memcpy(buffer, read(sizeof(Types::ReceivePacket_RC_CTRL)).c_str(), sizeof(Types::ReceivePacket_RC_CTRL));
            UserLib::fromVector(buffer, &rc_pkg);
            callback(rc_pkg);
        }
        return 0;
    }

    void Serial_interface::task() {
        while (true) {
            try {
                if (isOpen()) {
                    read((uint8_t *)&header, 2);
                    if (header == 0xAA55) {
                        read((uint8_t *)&header, 1);
                        unpack(header);
                    }
                } else {
                    enumerate_ports();
                    return;
                }
            } catch (serial::IOException &e) {
                LOG_ERR("serail offline! end program now\n");
                exit(-1);
            }
        }
    }
}  // namespace IO
