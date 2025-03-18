#include "device/super_cap.hpp"

namespace Device
{
    void Super_Cap::init(const std::string& can_name, const std::shared_ptr<Robot::Robot_set>& robot) {
        robot_set = robot;
        can = IO::io<CAN>[can_name];
        can->register_callback_key(0x51, std::bind(&Super_Cap::unpack, this, std::placeholders::_1));
    }

    void Super_Cap::unpack(const can_frame& frame) {
        std::memcpy(&robot_set->super_cap_info, frame.data, 8);
        set(true, 300);
        // LOG_INFO(
        //     "errorCode %d\tchassisPower %f\tchassisPowerlimit %d\tcapEnergy %d\n",
        //     robot_set->super_cap_info.errorCode,
        //     robot_set->super_cap_info.chassisPower,
        //     (int)robot_set->super_cap_info.chassisPowerlimit,
        //     (int)robot_set->super_cap_info.capEnergy);
    }

    void Super_Cap::set(bool enable, uint16_t power_limit) {
        can_frame send{};
        send.can_id = 0x61;
        if (enable)
            send.data[0] = 0x80;
        send.data[3] = power_limit >> 8;
        send.data[4] = power_limit & 0xff;

        can->send(send);
    }
}  // namespace Device
