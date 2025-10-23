#pragma once
#include <controller.hpp>
#include <bits/stl_algo.h>

#include "types.hpp"

namespace Pid
{
    /**
     * @brief          pid config data
     * @param[in]      PID: 0: kp, 1: ki, 2:kd
     * @param[in]      max_out: pid最大输出
     * @param[in]      max_iout: pid最大积分输出
     * @retval         none
     */

    struct PidConfig
    {

        // PID 三参数
        fp32 kp;
        fp32 ki;
        fp32 kd;

        fp32 max_out;   // 最大输出
        fp32 max_iout;  // 最大积分输出
    };

    class PidPosition final : public PidConfig, public Controller
    {
    public:
        fp32 &ref;
        ~PidPosition() override = default;
        explicit PidPosition(const PidConfig &config, fp32 &ref) : PidConfig(config), ref(ref) {}
        void set(fp32 set_v) override;
        void clean();

    public:
        fp32 Pout = 0.f;
        fp32 Iout = 0.f;
        fp32 Dout = 0.f;
        fp32 Dbuf = 0.f;   // 微分项 0最新 1上一次 2上上次
        fp32 error[2] = {};  // 误差项 0最新 1上一次 2上上次
    };



    class PidRad final : public PidConfig, public Controller
    {
    public:
        fp32 &ref;
        explicit PidRad(const PidConfig &config, fp32 &ref) : PidConfig(config), ref(ref) {};
        ~PidRad() override = default;
        void set(fp32 set) override;

    public:
        fp32 Pout = 0.f;
        fp32 Iout = 0.f;
        fp32 Dout = 0.f;
        fp32 err = 0.f;
        fp32 last_err = 0.f;
    };

    class Invert final : public PidConfig, public Controller
    {
    public:
        explicit Invert(const fp32 dir) : dir(dir) {};
        ~Invert() override = default;
        void set(fp32 set) override;
				fp32 dir;
    };
}  // namespace Pid
