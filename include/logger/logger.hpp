#pragma once

#include <fstream>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <cstdint>
#include <ostream>
#include <string>
#include <map>
#include <functional> 
#include <iostream>
#include <thread>
#include <unordered_map>
#include <sstream>
#include <cstring>
#include <unordered_set>
#include <utility>
#include "singleton.hpp"


#define LOG_ERR(s, ...)                                              \
    do {                                                             \
        printf(ANSI_FMT(s, ANSI_FG_RED) __VA_OPT__(, ) __VA_ARGS__); \
    } while (0)


enum class MessageType : uint8_t {
    RegisterName = 0x00,
    UpdateValue  = 0x01,
    Console      = 0x02,
    MessageBox   = 0x03   
};

struct LogMessage {
    static std::string build(const std::string& data, MessageType type) {
        uint16_t package_size = static_cast<uint16_t>(data.size() + sizeof(uint16_t) + sizeof(uint8_t));

        std::string res;
        res.resize(package_size);

        memcpy(res.data(), &package_size, sizeof(package_size));
        res[2] = static_cast<char>(type);

        memcpy(res.data() + 3, data.data(), data.size());

        return res;
    }
};

// RegisterName
struct LogRegisterNameMessage : public LogMessage {
    static std::string build(uint32_t id, const std::string& name) {
        uint8_t name_length = static_cast<uint8_t>(name.size());

        std::string payload;
        payload.resize(sizeof(uint32_t) + sizeof(uint8_t) + name_length);
        memcpy(payload.data(), &id, sizeof(id));
        payload[sizeof(id)] = static_cast<char>(name_length);
        memcpy(payload.data() + sizeof(id) + sizeof(uint8_t), name.data(), name_length);

        return LogMessage::build(payload, MessageType::RegisterName);
    }
};


// UpdateValue
struct LogUpdateValueMessage : public LogMessage {
    static std::string build(uint32_t id, double value) {
        std::string payload;
        payload.resize(sizeof(uint32_t) + sizeof(double));

        memcpy(payload.data(), &id, sizeof(id));
        memcpy(payload.data() + sizeof(id), &value, sizeof(value));

        return LogMessage::build(payload, MessageType::UpdateValue);
    }
};

// Console Message
struct LogConsoleMessage : public LogMessage {
    static std::string build(const std::string& msg) {
        uint16_t msg_len = static_cast<uint16_t>(msg.size());

        std::string payload;
        payload.resize(sizeof(uint16_t) + msg_len);

        memcpy(payload.data(), &msg_len, sizeof(msg_len));
        memcpy(payload.data() + sizeof(uint16_t), msg.data(), msg.size());

        return LogMessage::build(payload, MessageType::Console);
    }
};

// MessageBox Message
struct LogMessageBoxMessage : public LogMessage {
    static std::string build(const std::string& msg) {
        uint16_t msg_len = static_cast<uint16_t>(msg.size());

        std::string payload;
        payload.resize(sizeof(uint16_t) + msg_len);

        memcpy(payload.data(), &msg_len, sizeof(msg_len));
        memcpy(payload.data() + sizeof(uint16_t), msg.data(), msg.size());

        return LogMessage::build(payload, MessageType::MessageBox);
    }
};



inline uint32_t string_hash(const std::string& str) {
    uint32_t hash = 2166136261u;  // FNV offset basis
    for (unsigned char c : str) {
        hash ^= c;
        hash *= 16777619u;        // FNV prime
    }
    return hash;
}

class Logger:public Singleton<Logger>{
private:
    std::unordered_set<std::string> _registered_names;
    std::queue<std::string> _q;
    std::mutex _mtx;
    std::condition_variable _cv;
    int client_socket;
    std::mutex _mutex;

public:
    template<typename T,typename... Args>
    inline void push_message(Args&&... args){
        auto message = T::build(std::forward<Args>(args)...);
        {
            std::lock_guard<std::mutex> lock(_mtx);
            _q.push(message);
        }

        _cv.notify_one();
    }

    std::map<std::string,int> cnt;


    void push_value(const std::string& name,double value){
        uint32_t hash = string_hash(name);

        if(!_registered_names.contains(name)){
            push_message<LogRegisterNameMessage>(hash,name);
            _registered_names.insert(name);
        }

        push_message<LogUpdateValueMessage>(hash,value);
    }

    // TODO
    void push_console_message(const std::string& msg) {

        push_message<LogConsoleMessage>(msg);
    }

    //TODO
    void push_message_box(const std::string& msg) {
        push_message<LogMessageBoxMessage>(msg);
    }

    [[noreturn]] void task() {

        client_socket = socket(AF_INET, SOCK_DGRAM, 0); 
        if (client_socket < 0) {
            std::cout << "socket创建失败" << std::endl;
        }

        sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(8080);

        inet_pton(AF_INET, "192.168.1.53", &server_addr.sin_addr);


        while (true) {
            std::string buffer;
            
            {
                std::unique_lock<std::mutex> lock(_mtx);
                _cv.wait(lock, [this]{ return !_q.empty(); });

                size_t count = 0;
                while(!_q.empty() && count < 16) {
                    buffer += _q.front();
                    _q.pop();
                    count++;
                }
            } 

            if (buffer.empty()) {
                continue;
            }

            ssize_t sent_bytes = sendto(
                client_socket, 
                buffer.data(), 
                buffer.size(), 
                0, 
                (struct sockaddr*)&server_addr, 
                sizeof(server_addr)
            );
            
            if (sent_bytes < 0) {
                
            } else if ((size_t)sent_bytes != buffer.size()) {
                
            }

        }
    }

    void into_txt(std::string file_path, std::string log) {
        std::ofstream ofs(file_path, std::ios::app);
        if (!ofs.is_open()) {
            std::cout << "error open: " << file_path << std::endl;
        }
        ofs << log << "\n"; 
        ofs.close(); 
    }    

};

#define logger (Logger::instance())

