#include "pid_controller.hpp"

namespace Pid
{
    /**
     * @brief          pid config data init
     * @param[out]     pid: PID结构数据指针
     * @param[in]      mode: PID_POSITION:普通PID
     *                 PID_DELTA: 差分PID
     * @param[in]      PID: 0: kp, 1: ki, 2:kd
     * @param[in]      max_out: pid最大输出
     * @param[in]      max_iout: pid最大积分输出
     * @retval         none
     */

    Pid_config::Pid_config(fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout)
        : kp(kp),
          ki(ki),
          kd(kd),
          max_out(max_out),
          max_iout(max_iout) {
    }
}  // namespace Pid

namespace Pid
{

    Pid_position::Pid_position(const Pid_config &config) : Pid_config(config) {
    }

    void Pid_position::calc(fp32 ref, fp32 set) {
        error[1] = error[0];
        error[0] = (set - ref);

        Pout = kp * error[0];
        Iout += ki * error[0];
        Dbuf = (error[0] - error[1]);
        Dout = kd * Dbuf;

        Iout = std::clamp(Iout, -max_iout, max_iout);
        out = Pout + Iout + Dout;
        out = std::clamp(out, -max_out, max_out);
    }

    void Pid_position::clean() {
        error[0] = error[1] = 0;
        Dbuf = 0.f;
        Pout = Iout = Dout = 0;
        out = 0;
    }

    Pid_rad::Pid_rad(const Pid_config &config) : Pid_config(config), ramp(25, 1.f / 1000) {
    }

    void Pid_rad::calc(fp32 get, fp32 set) {
        last_err = err;
        ramp.update(UserLib::rad_format(set - get));

        err = ramp.out;
        Pout = kp * err;
        Iout += ki * err;
        Dout = kd * (err - last_err);

        Iout = std::clamp(Iout, -max_iout, max_iout);
        out = Pout + Iout + Dout;
        out = std::clamp(out, -max_out, max_out);
    }

}  // namespace Pid
