#include "can.hpp"

#include <cstring>

#include "utils.hpp"

namespace IO
{
    Can_interface::Can_interface(const std::string &name) : name(name) {
        addr = new sockaddr_can;
        ifr = new ifreq;
        soket_id = -1;
        init_flag = false;
        init(name.c_str());
    }

    void Can_interface::init(const char *can_channel) {
        const std::string open_cmd = "echo 1 | sudo -S ip link set " + name + " up type can bitrate 1000000";
        system(open_cmd.c_str());
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
                    LOG_ERR("Error reading CAN frame");
                    return Status::ERROR;
                } else {
                    // printf("not reading!\n");
                }
                callback_key(frame_r.can_id, frame_r);
            }
        }
    }

    bool Can_interface::send(const can_frame &frame) {
        /* send CAN frame */
        write(soket_id, &frame, sizeof(can_frame));
        return true;
    }

}  // namespace IO
