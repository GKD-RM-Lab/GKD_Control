#include "socket_interface.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>

#include "robot.hpp"
#include "user_lib.hpp"

namespace IO
{
    void Server_socket_interface::task() {
        while (true) {
            memset(buffer, 0, sizeof(buffer));
            sockaddr cli_addr;
            socklen_t cli_addr_len = sizeof(cli_addr);
            auto n = recvfrom(sockfd, buffer, 1, MSG_WAITALL, &cli_addr, &cli_addr_len);

            if (n > 0) {
                uint8_t header = buffer[0];
                // if (clients.count(header) == 0) {
                //     LOG_OK("register clients %d\n", header);
                //     clients.insert(std::pair<uint8_t, sockaddr_in>(header, cli_addr));
                // }
                callback_key(header, bit_reader_, cli_addr);
                // switch (header) {
                //     case 0xA5: {
                //         Robot::ReceiveGimbalPacket pkg{};
                //         UserLib::unpack(pkg, buffer);
                //         callback(pkg);
                //         break;
                //     }
                //     case 0x6A: {
                //         Robot::Auto_aim_control vc;
                //         UserLib::unpack(vc, buffer);
                //         callback(vc);
                //         break;
                //     }
                //     default: {
                //         LOG_ERR("get error flag: %02x\n", header);
                //         Robot::ReceiveGimbalPacket pkg{};
                //         UserLib::unpack(pkg, buffer);
                //         break;
                //     }
                // }
            }
        }
    }

    Server_socket_interface::Server_socket_interface(std::string name) : port_num(11451), name(name) {
        // NOTE: read this https://www.linuxhowtos.org/C_C++/socket.htm
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            LOG_ERR("can't open socket\n");
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(port_num);

        connections.insert(std::pair<uint8_t, uint8_t>(0xA6, 0x6A));
        connections.insert(std::pair<uint8_t, uint8_t>(0x5A, 0xA5));

        sockaddr_in client;
        client.sin_family = AF_INET;
        client.sin_addr.s_addr = inet_addr("127.0.0.1");
        client.sin_port = htons(11453);

        clients.insert(std::pair<uint8_t, sockaddr_in>(0x6A, client));

        if (bind(sockfd, (sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            LOG_ERR("can't bind socket fd with port number");
        }
        bit_reader_ = [&](uint8_t *data, size_t len) {
            sockaddr cli_addr;
            socklen_t cli_addr_len = sizeof(cli_addr);
            recvfrom(sockfd, data, len, MSG_WAITALL, &cli_addr, &cli_addr_len);
        };
    }

    Server_socket_interface::~Server_socket_interface() {
        close(sockfd);
    }
}  // namespace IO
