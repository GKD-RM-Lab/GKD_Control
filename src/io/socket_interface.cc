#include "socket_interface.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>

#include "robot.hpp"
#include "user_lib.hpp"
#include "utils.hpp"

namespace IO
{
    void Server_socket_interface::task() {
        while (true) {
            memset(buffer, 0, sizeof(buffer));
            sockaddr_in cli_addr;
            socklen_t cli_addr_len = sizeof(cli_addr);
            auto n =
                recvfrom(sockfd, buffer, 256, MSG_WAITALL, (sockaddr *)&cli_addr, &cli_addr_len);
            if (n > 0) {
                uint8_t header = buffer[0];
                if (clients.count(header) == 0) {
                    LOG_INFO("register clients %d\n", header);
                    clients.insert(std::pair<uint8_t, sockaddr_in>(header, cli_addr));
                }
                switch (header) {
                    case 0x37: {
                        Robot::ReceiveNavigationInfo pkg{};
                        UserLib::unpack(pkg, buffer);
                        callback(pkg);
                        break;
                    }
                    default: {
                        Robot::Auto_aim_control vc;
                        UserLib::unpack(vc, buffer);
                        callback_key(vc.header, vc);
                        break;
                    }
                }
            }
        }
    }

    Server_socket_interface::Server_socket_interface(std::string name)
        : port_num(11451),
          name(name) {
        // NOTE: read this https://www.linuxhowtos.org/C_C++/socket.htm
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            LOG_ERR("can't open socket\n");
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(port_num);

        if (bind(sockfd, (sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            LOG_ERR("can't bind socket fd with port number");
        }
        else{
            LOG_INFO("bind socket %d",port_num);
        }
    }

    void Server_socket_interface::add_client(uint8_t header, std::string ip, int port) {
        sockaddr_in client;
        client.sin_family = AF_INET;
        client.sin_addr.s_addr = inet_addr(ip.c_str());
        client.sin_port = htons(port);
        clients.insert(std::pair<uint8_t, sockaddr_in>(header, client));
        LOG_INFO("ip %s, port %d\n", ip.c_str(), port);
    }

    Server_socket_interface::~Server_socket_interface() {
        close(sockfd);
    }
}  // namespace IO
