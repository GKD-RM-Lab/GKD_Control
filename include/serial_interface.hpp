#ifndef __SERIAL_INTERFACE__
#define __SERIAL_INTERFACE__
#include <functional>

#include "hardware_callback.hpp"
#include "serial/serial.h"
#include "utils.hpp"

namespace Hardware
{

    template<class T>
    class Serial_interface : serial::Serial, public Callback<T>
    {
       public:
        Serial_interface(std::string port_name, int baudrate, int simple_timeout);
        Serial_interface() = delete;
        ~Serial_interface();
        void task();

       private:
        inline void enumerate_ports();
        inline int unpack();
        inline void fromVector(const uint8_t *data, T *pkg);

       public:
        T rp;

       private:
        uint8_t buffer[256];
        uint8_t header;
    };

    template<class T>
    Serial_interface<T>::Serial_interface(std::string port_name, int baudrate, int simple_timeout)
        : serial::Serial(port_name, baudrate, serial::Timeout::simpleTimeout(simple_timeout)) {
    }

    template<class T>
    Serial_interface<T>::~Serial_interface() = default;

    template<class T>
    inline void Serial_interface<T>::enumerate_ports() {
        std::vector<serial::PortInfo> devices_found = serial::list_ports();
        auto iter = devices_found.begin();

        while (iter != devices_found.end()) {
            serial::PortInfo device = *iter++;
            //LOG_INFO("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str());
        }
    }

    template<class T>
    inline void Serial_interface<T>::fromVector(const uint8_t *data, T *pkg) {
        for (size_t i = 0; i < sizeof(T); ++i) {
            ((uint8_t *)pkg)[i] = data[i];
        }
    }

    template<class T>
    inline int Serial_interface<T>::unpack() {
        memcpy(buffer, read(sizeof(T)).c_str(), sizeof(T));
        fromVector(buffer, &rp);
				//LOG_INFO("serial data : (angle y/p/r) %f %f %f\n", rp.yaw, rp.pitch, rp.roll);
        this->callback(rp);
        return 0;
    }

    template<class T>
    void Serial_interface<T>::task() {
        while (true) {
            if (isOpen()) {
                read(&header, 1);
                if (header == 0xAB)
                    unpack();
            } else {
                enumerate_ports();
                return;
            }
        }
    }
}  // namespace Hardware
#endif
