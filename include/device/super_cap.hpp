#pragma once

#include <cstring>
#include <memory>

#include "can.hpp"
#include "deviece_base.hpp"
#include "io.hpp"
#include "robot.hpp"
#include "super_cap.hpp"
#include "types.hpp"

namespace Device
{
    /**
     * @brief 超级电容通信模块
     *
     * 职责:
     * 1) 接收电容状态帧并更新 robot_set->super_cap_info
     * 2) 周期性向电容发送“使能+功率限制”配置帧
     *
     * 协议约定(当前项目):
     * - 接收ID: 0x51  （电容 -> 主控）
     * - 发送ID: 0x061 （主控 -> 电容）
     */
    class Super_Cap : Device::DeviceBase
    {
       private:
        std::string can_name_;                        // can 名称，用于异常时重取接口
        IO::Can_interface* can;                      // 对应底盘 CAN 总线
        std::shared_ptr<Robot::Robot_set> robot_set; // 共享状态（功率/裁判信息）

       public:
        // 初始化 CAN 回调，绑定状态接收入口
        void init(const std::string& can_name, const std::shared_ptr<Robot::Robot_set>& robot);

        // 解析电容状态帧（0x51）
        void unpack(const can_frame& frame);

        // 下发电容控制帧（0x061）
        // enable: 电容使能开关
        // power_limit: 目标底盘功率限制
        void set(bool enable, uint16_t power_limit);
    };
}  // namespace Device
