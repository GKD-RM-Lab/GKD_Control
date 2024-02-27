#include "pid_controller.hpp"

namespace Pid
{

    Pid_ctrl::Pid_ctrl(
        uint8_t mode,
        const fp32 kp,
        const fp32 ki,
        const fp32 kd,
        const fp32 max_out,
        const fp32 max_iout)
        : mode(mode),
          kp(kp),
          ki(ki),
          kd(kd),
          max_out(max_out),
          max_iout(max_iout) {
        this->Dbuf[0] = this->Dbuf[1] = this->Dbuf[2] = 0.0f;
        this->error[0] = this->error[1] = this->error[2] = this->Pout = this->Iout = this->Dout = this->out = 0.0f;
    }

    Pid_ctrl::~Pid_ctrl() {
    }
    void Pid_ctrl::calc(fp32 ref, fp32 set) {
        error[2] = error[1];
        error[1] = error[0];
        error[0] = (set - ref);
        Dbuf[2] = Dbuf[1];
        Dbuf[1] = Dbuf[0];
        if (mode == PID_POSITION) {
            Pout = kp * error[0];
            Iout += ki * error[0];
            Dbuf[0] = (error[0] - error[1]);
            Dout = kd * Dbuf[0];
            Iout = std::min(std::max(Iout, -max_iout), max_iout);
            out = Pout + Iout + Dout;
            out = std::min(std::max(out, -max_out), max_out);
        } else if (mode == PID_DELTA) {
            Pout = kp * (error[0] - error[1]);
            Iout = ki * error[0];
            Dbuf[0] = (error[0] - 2.f * error[1] + error[2]);
            Dout = kd * Dbuf[0];
            out = Pout + Iout + Dout;
            out = std::min(std::max(out, -max_out), max_out);
        }
    }

    /** TODO :pub all these into constructor
     * @brief          pid struct data init
     * @param[out]     pid: PID struct data point
     * @param[in]      mode: PID_POSITION: normal pid
     *                 PID_DELTA: delta pid
     * @param[in]      PID: 0: kp, 1: ki, 2:kd
     * @param[in]      max_out: pid max out
     * @param[in]      max_iout: pid max iout
     * @retval         none
     */
    /**
     * @brief          pid struct data init
     * @param[out]     pid: PID结构数据指针
     * @param[in]      mode: PID_POSITION:普通PID
     *                 PID_DELTA: 差分PID
     * @param[in]      PID: 0: kp, 1: ki, 2:kd
     * @param[in]      max_out: pid最大输出
     * @param[in]      max_iout: pid最大积分输出
     * @retval         none
     */

}  // namespace Pid