#ifndef __SOCKET_INTERFACE__
#define __SOCKET_INTERFACE__

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <map>

#include "hardware_callback.hpp"
#include "robot.hpp"
#include "utils.hpp"

namespace IO
{
    class Server_socket_interface : public Callback<Robot::Vison_control, Robot::ReceiveGimbalPacket>

    {
       public:
        Server_socket_interface(std::string name);
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
            }
        }

       private:
        int64_t port_num;
        int sockfd;

        sockaddr_in serv_addr;
        std::map<uint8_t, sockaddr_in> clients;
        std::map<uint8_t, uint8_t> connections;

        char buffer[256];

       public:
        std::string name;

       private:
    };
}  // namespace IO
#endif

using SOCKET = IO::Server_socket_interface;
