#ifndef __SOCKET_INTERFACE__
#define __SOCKET_INTERFACE__

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <map>
#include <memory>

#include "arpa/inet.h"
#include "hardware_callback.hpp"
#include "robot.hpp"
#include "utils.hpp"
#include "iostream"

namespace Io
{
    class Server_socket_interface : public Hardware::Callback<Robot::Vison_control, Robot::ReceiveGimbalPacket>

    {
       public:
        Server_socket_interface();
        ~Server_socket_interface();
        void task();

        template<typename T>
        void send(const T &pkg) {
            auto it = connections.find(*(uint8_t *)&pkg);
            if (it == connections.end()) {
                LOG_ERR("error connections %x to %x\n", *(uint8_t *)&pkg, it->second);
            } else {
                auto n = sendto(
                    sockfd,
                    (const char *)(&pkg),
                    sizeof(pkg),
                    MSG_CONFIRM,
                    (const struct sockaddr *)&clients.find(it->second)->second,
                    sizeof(clients.find(it->second)->second));
                //LOG_OK("send to client %d bytes\n", n);
            }
        }

        // void send(const std::string &str) {
        //     auto n =
        //         sendto(sockfd, str.c_str(), str.size(), MSG_CONFIRM, (const struct sockaddr *)&cli_addr,
        //         cli_addr_len);
        //     //            LOG_OK("%lu bytes send to client\n", n);
        // }

        // void send(const char *str) {
        //     auto n = sendto(sockfd, str, strlen(str), MSG_CONFIRM, (const struct sockaddr *)&cli_addr, cli_addr_len);
        //     //            LOG_OK("%lu bytes send to client\n", n);
        // }

       private:
        int64_t port_num;
        int sockfd;

        sockaddr_in serv_addr;
        std::map<uint8_t, sockaddr_in> clients;
        std::map<uint8_t, uint8_t> connections;

        char buffer[256];

       public:
       private:
    };
}  // namespace Io

#endif
