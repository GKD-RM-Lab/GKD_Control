// #pragma
#ifndef _DEVIECE_BASE_H
#define _DEVIECE_BASE_H

#include "chrono"

namespace Device
{
    class DeviceBase
    {
       public:
        explicit DeviceBase(uint32_t offline_time_t);
        explicit DeviceBase();
        bool offline() const;

       protected:
        void update_time();

       private:
        using time_point = typename std::chrono::system_clock::time_point;

        uint32_t offline_time;
        time_point last_time;
    };
}  // namespace Device

#endif