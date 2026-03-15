#ifndef __CAN_INTERFACE__
#define __CAN_INTERFACE__

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <shared_mutex>

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
        bool init(const char *can_channel);

       private:
        bool init_locked(const char *can_channel);
        void close_locked();
        bool try_reconnect(const char *source, int error_code);
        static bool should_reconnect(int error_code);

        sockaddr_can *addr;
        can_frame frame_r;
        ifreq *ifr;
        Types::debug_info_t *debug;
        int soket_id;
        std::atomic<bool> init_flag;
        std::string can_channel_;
        std::atomic<uint32_t> send_fail_count_;
        std::atomic<bool> reconnect_requested_;
        std::shared_mutex socket_mutex_;
        std::chrono::steady_clock::time_point last_reconnect_attempt_;
        static constexpr uint32_t kSendRecoverThreshold = 2000;
        static constexpr auto kReconnectInterval = std::chrono::milliseconds(200);
        static constexpr auto kReadTimeout = std::chrono::milliseconds(100);

       public:
        std::string name;
    };

}  // namespace IO

#endif
