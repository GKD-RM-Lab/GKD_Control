#ifndef __SERIAL_INTERFACE__
#define __SERIAL_INTERFACE__
#include <functional>

#include "io_callback.hpp"
#include "serial/serial.h"
#include "types.hpp"
#include "utils.hpp"

namespace IO
{

    class Serial_interface : serial::Serial, public Callback<Types::ReceivePacket_IMU, Types::ReceivePacket_RC_CTRL>
    {
       public:
        Serial_interface(std::string port_name, int baudrate, int simple_timeout);
        Serial_interface() = delete;
        ~Serial_interface();
        void task();
        template<typename T>
        void send(T val) {
           write(&val, sizeof(T));
        }

       private:
        inline void enumerate_ports();
        inline int unpack(uint8_t pkg_id);

       public:
        Types::ReceivePacket_IMU imu_pkg;
        Types::ReceivePacket_RC_CTRL rc_pkg;
        std::string name;

       private:
        uint8_t buffer[256];
        uint16_t header;
    };
}  // namespace IO
#endif

using SERIAL = IO::Serial_interface;
