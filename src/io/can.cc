#include "can.hpp"

#include <cerrno>
#include <cstring>

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
        init(name.c_str());
    }

    void Can_interface::init(const char *can_channel) {
        can_channel_ = can_channel;
        if (soket_id >= 0) {
            close(soket_id);
            soket_id = -1;
        }

        // create CAN socket
        if ((soket_id = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            LOG_ERR("Error while creating socket");
            exit(-1);
        }

        std::strcpy(ifr->ifr_name, can_channel);
        ioctl(soket_id, SIOCGIFINDEX, ifr);

        addr->can_family = AF_CAN;
        addr->can_ifindex = ifr->ifr_ifindex;

        // bind CAN socket to the given interface
        if (bind(soket_id, (sockaddr *)addr, sizeof(*addr)) < 0) {
            perror("Error in socket bind");
            exit(-1);
        }
        init_flag = true;
        send_fail_count_ = 0;
    }

    Can_interface::~Can_interface() {
        delete addr;
        delete ifr;
    }

    bool Can_interface::task() {
        for (;;) {
            if (init_flag) {
                // read CAN frame
                if (read(soket_id, &frame_r, sizeof(can_frame)) <= 0) {
                    LOG_ERR("Error reading CAN frame\n");
                    return Status::ERROR;
                } else {
                    // printf("not reading!\n");
                }
                callback_key(frame_r.can_id, frame_r);
            }
        }
    }

    bool Can_interface::send(const can_frame &frame) {
        if (!init_flag) {
            return false;
        }

        const ssize_t sent = write(soket_id, &frame, sizeof(can_frame));
        if (sent == static_cast<ssize_t>(sizeof(can_frame))) {
            send_fail_count_ = 0;
            return true;
        }

        send_fail_count_++;
        if (send_fail_count_ >= kSendRecoverThreshold) {
            // LOG_ERR(
            //     "CAN[%s] send failed %u times (errno=%d), rebuilding socket\n",
            //     can_channel_.c_str(),
            //     send_fail_count_,
            //     errno);
            // exit(-1);
        }
        return false;
    }

}  // namespace IO
