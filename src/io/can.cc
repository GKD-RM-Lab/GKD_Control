#include "can.hpp"

#include <cerrno>
#include <cstring>
#include <mutex>
#include <shared_mutex>
#include <thread>

#include "utils.hpp"

namespace IO
{
    Can_interface::Can_interface(const std::string &name) : name(name) {
        addr = new sockaddr_can;
        ifr = new ifreq;
        soket_id = -1;
        init_flag = false;
        can_channel_ = name;
        send_fail_count_ = 0;
        reconnect_requested_ = false;
        last_reconnect_attempt_ = std::chrono::steady_clock::now() - kReconnectInterval;
        init(name.c_str());
    }

    bool Can_interface::init(const char *can_channel) {
        std::unique_lock lock(socket_mutex_);
        return init_locked(can_channel);
    }

    bool Can_interface::init_locked(const char *can_channel) {
        can_channel_ = can_channel;
        close_locked();

        soket_id = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (soket_id < 0) {
            LOG_ERR("CAN[%s] create socket failed (errno=%d)\n", can_channel_.c_str(), errno);
            return false;
        }

        std::memset(ifr, 0, sizeof(*ifr));
        std::strncpy(ifr->ifr_name, can_channel, IFNAMSIZ - 1);
        if (ioctl(soket_id, SIOCGIFINDEX, ifr) < 0) {
            LOG_ERR("CAN[%s] ioctl(SIOCGIFINDEX) failed (errno=%d)\n", can_channel_.c_str(), errno);
            close_locked();
            return false;
        }

        addr->can_family = AF_CAN;
        addr->can_ifindex = ifr->ifr_ifindex;

        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = static_cast<suseconds_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(kReadTimeout).count());
        if (setsockopt(soket_id, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            LOG_ERR("CAN[%s] setsockopt(SO_RCVTIMEO) failed (errno=%d)\n", can_channel_.c_str(), errno);
            close_locked();
            return false;
        }

        if (bind(soket_id, reinterpret_cast<sockaddr *>(addr), sizeof(*addr)) < 0) {
            LOG_ERR("CAN[%s] bind failed (errno=%d)\n", can_channel_.c_str(), errno);
            close_locked();
            return false;
        }

        init_flag = true;
        reconnect_requested_ = false;
        send_fail_count_ = 0;
        return true;
    }

    void Can_interface::close_locked() {
        init_flag = false;
        if (soket_id >= 0) {
            close(soket_id);
            soket_id = -1;
        }
    }

    bool Can_interface::should_reconnect(int error_code) {
        switch (error_code) {
            case ENXIO:
            case ENODEV:
            case ENETDOWN:
            case ENETUNREACH:
            case EBADF:
            case ENOTCONN:
            case EPIPE:
                return true;
            default:
                return false;
        }
    }

    bool Can_interface::try_reconnect(const char *source, int error_code) {
        const auto now = std::chrono::steady_clock::now();
        std::unique_lock lock(socket_mutex_);
        if (init_flag && !reconnect_requested_) {
            return true;
        }
        if (now - last_reconnect_attempt_ < kReconnectInterval) {
            return init_flag;
        }
        last_reconnect_attempt_ = now;

        close_locked();
        if (!init_locked(can_channel_.c_str())) {
            LOG_ERR(
                "CAN[%s] reconnect failed from %s (errno=%d)\n",
                can_channel_.c_str(),
                source,
                error_code);
            return false;
        }

        LOG_INFO("CAN[%s] reconnected from %s\n", can_channel_.c_str(), source);
        return true;
    }

    Can_interface::~Can_interface() {
        std::unique_lock lock(socket_mutex_);
        close_locked();
        delete addr;
        delete ifr;
    }

    bool Can_interface::task() {
        for (;;) {
            if (!init_flag || reconnect_requested_) {
                if (!try_reconnect("task", 0)) {
                    std::this_thread::sleep_for(kReconnectInterval);
                }
                continue;
            }

            ssize_t read_size = -1;
            int read_errno = 0;
            {
                std::shared_lock lock(socket_mutex_);
                if (!init_flag) {
                    continue;
                }
                read_size = read(soket_id, &frame_r, sizeof(can_frame));
                if (read_size < 0) {
                    read_errno = errno;
                }
            }

            if (read_size == static_cast<ssize_t>(sizeof(can_frame))) {
                callback_key(frame_r.can_id, frame_r);
                continue;
            }

            if (read_size < 0 && (read_errno == EAGAIN || read_errno == EWOULDBLOCK)) {
                continue;
            }

            reconnect_requested_ = true;
            if (read_size < 0) {
                LOG_ERR("CAN[%s] read failed (errno=%d), rebuilding socket\n", can_channel_.c_str(), read_errno);
                try_reconnect("read", read_errno);
            } else {
                LOG_ERR("CAN[%s] read returned %zd, rebuilding socket\n", can_channel_.c_str(), read_size);
                try_reconnect("read", 0);
            }
        }
    }

    bool Can_interface::send(const can_frame &frame) {
        ssize_t sent = -1;
        int send_errno = 0;
        {
            std::shared_lock lock(socket_mutex_);
            if (!init_flag) {
                reconnect_requested_ = true;
                return false;
            }

            sent = write(soket_id, &frame, sizeof(can_frame));
            if (sent < 0) {
                send_errno = errno;
            }
        }

        if (sent == static_cast<ssize_t>(sizeof(can_frame))) {
            send_fail_count_ = 0;
            return true;
        }

        reconnect_requested_ = true;
        const auto fail_count = send_fail_count_.fetch_add(1) + 1;
        if (fail_count == 1 || fail_count % kSendRecoverThreshold == 0) {
            LOG_ERR(
                "CAN[%s] send failed %u times (errno=%d), rebuilding socket\n",
                can_channel_.c_str(),
                fail_count,
                send_errno);
        }

        if (should_reconnect(send_errno) || !init_flag) {
            try_reconnect("send", send_errno);
        }
        return false;
    }

}  // namespace IO
