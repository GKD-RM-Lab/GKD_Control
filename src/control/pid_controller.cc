#include "pid_controller.hpp"

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

    Pid_rad::Pid_rad(const Pid_config &config) : Pid_config(config), ramp(10, 1.f / 1000) {
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

//    void Pid_rad::calc(fp32 get, fp32 set, fp32 error_delta) {
//
//        err = UserLib::rad_format(set - get);
//        Pout = kp * err;
//        Iout += ki * err;
//        Dout = kd * error_delta;
//
//        Iout = std::clamp(Iout, -max_iout, max_iout);
//        out = Pout + Iout + Dout;
//        out = std::clamp(out, -max_out, max_out);
//    }
}  // namespace Pid
