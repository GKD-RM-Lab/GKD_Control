#ifndef __PID_CONTROLLER__
#define __PID_CONTROLLER__

#include <climits>
#include <cmath>

#include "types.hpp"

namespace Pid
{

    class Pid_ctrl
    {
       public:
        Pid_ctrl() = delete;
        Pid_ctrl(uint8_t mode, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout);
        ~Pid_ctrl();
        void calc(fp32 ref, fp32 set);

       private:
       public:
        uint8_t mode;
        // PID 三参数
        fp32 kp;
        fp32 ki;
        fp32 kd;

        fp32 max_out;   // 最大输出
        fp32 max_iout;  // 最大积分输出

        fp32 out;
        fp32 Pout;
        fp32 Iout;
        fp32 Dout;
        fp32 Dbuf[3];   // 微分项 0最新 1上一次 2上上次
        fp32 error[3];  // 误差项 0最新 1上一次 2上上次
       private:
    };

}  // namespace Pid

#endif
