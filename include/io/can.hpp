#ifndef __CAN_INTERFACE__
#define __CAN_INTERFACE__

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdint>
#include <cstdlib>

#include "io_callback.hpp"
#include "types.hpp"

namespace IO
{
    class Can_interface : public Callback_key<uint32_t, can_frame>
    {
       public:
        Can_interface(const std::string &name);
        ~Can_interface();
        bool send(const can_frame &frame);
        bool task();
        void init(const char *can_channel);

       private:
        sockaddr_can *addr;
        can_frame frame_r;
        ifreq *ifr;
        Types::debug_info_t *debug;
        int soket_id;
        bool init_flag;

       public:
        std::string name;
    };

}  // namespace IO

#endif
