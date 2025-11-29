#ifndef __SOCKET_INTERFACE__
#define __SOCKET_INTERFACE__

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <map>

#include "io_callback.hpp"
#include "robot.hpp"
#include "utils.hpp"

namespace IO
{
    class Server_socket_interface : public Callback<Robot::ReceiveNavigationInfo>, public Callback_key<uint8_t, Robot::Auto_aim_control>

    {
       public:
        Server_socket_interface(std::string name);
        ~Server_socket_interface();
        void task();
        void add_client(uint8_t header, std::string ip, int port);

        template<typename T>
        void send(const T &pkg) {
            uint8_t header = *(uint8_t *)(&pkg);
            // LOG_INFO("header %d client: %d\n", header, clients.find(header)->second.sin_addr.s_addr);
            auto n = sendto(
                sockfd,
                (const char *)(&pkg),
                sizeof(pkg),
                MSG_CONFIRM,
                (const struct sockaddr *)&clients.find(header)->second,
                sizeof(clients.find(header)->second));
            if(n == -1) {
                LOG_ERR("error socket send");
            }  
        }

       private:
        int64_t port_num;
        int sockfd;

        sockaddr_in serv_addr;
        std::map<uint8_t, sockaddr_in> clients;

        char buffer[256];

       public:
        std::string name;

       private:
    };
}  // namespace IO
#endif

using SOCKET = IO::Server_socket_interface;
