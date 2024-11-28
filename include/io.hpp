#pragma once

#include <string>
#include <thread>
#include <unordered_map>
#include <utils.hpp>
#include "can.hpp"

using CAN = IO::Can_interface;

namespace IO {
    template<typename T>
    class IO {
    private:
        std::unordered_map<std::string, T*> data;
        std::vector<std::thread> io_handles;
    public:

        ~IO() {
            for(auto &handle : io_handles) {
                handle.detach();
            }
            io_handles.clear();
        }

        T* operator[](const std::string& key) const {
            auto p = data.find(key);
            if(p == data.end()) {
                LOG_ERR("IO error: no device named %s\n", key.c_str());
                return nullptr;
            }
            return p->second;
        }

        template<typename ...Args>
        void insert(Args && ...args) {
            T *data_p = new T(std::forward<Args>(args)...);
            insert(*data_p);
        }

        void insert(T *device) {
            insert(*device);
        }

        void insert(T &device) {
            auto &p = data[device.name];
            if(p != nullptr) {
                LOG_ERR("IO error: double register device named %s\n", device.name.c_str());
                throw std::runtime_error("IO error: double register device named " + device.name);
            }
            p = &device;
            io_handles.emplace_back(std::thread([&]() {device.task();}));
        }
    };

    template<typename T>
    IO<T> io;
}
