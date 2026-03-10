#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>

#include "logger.hpp"
#include "macro_helpers.hpp"
#include "pid_controller.hpp"
#include "power_controller.hpp"
#include "power_controller_utils.hpp"
#include "robot_type_config.hpp"
#include "utils.hpp"

namespace Power
{

    PowerStatus powerStatus;
    // 用户可配置功率下限（运行期会按模式/规则动态更新）
    static float MIN_MAXPOWER_CONFIGURED = 40.0f;
    static uint8_t LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = 1U;
    static uint16_t motorDisconnectCounter[4] = { 0U, 0U, 0U, 0U };
    static constexpr uint16_t MOTOR_DISCONNECT_HOLD_CYCLES = 1000U;
    static constexpr uint64_t CAP_OFFLINE_TIMEOUT_MS = 300U;
    static constexpr uint64_t REFEREE_OFFLINE_TIMEOUT_MS = 300U;
    static constexpr float RLS_UPDATE_POWER_ON_THRESHOLD = 6.0f;
    static constexpr float RLS_UPDATE_POWER_OFF_THRESHOLD = 4.0f;
    static constexpr uint16_t RLS_ENABLE_DEBOUNCE_CYCLES = 30U;
    static constexpr uint16_t RLS_DISABLE_DEBOUNCE_CYCLES = 5U;
    static constexpr uint16_t RLS_CAP_OK_ON_DEBOUNCE_CYCLES = 20U;
    static constexpr uint16_t RLS_CAP_OK_OFF_DEBOUNCE_CYCLES = 5U;
    static constexpr uint8_t RLS_REASON_USER_DISABLED = 1U << 0;
    static constexpr uint8_t RLS_REASON_CAP_INVALID = 1U << 1;
    static constexpr uint8_t RLS_REASON_LOW_MEASURED_POWER = 1U << 2;
    static constexpr uint8_t RLS_REASON_NONFINITE_SIGNAL = 1U << 3;

    static constexpr std::array<Utils::Log::BitDesc, 3> kPowerErrorBitDesc = {
        Utils::Log::BitDesc{static_cast<uint8_t>(Manager::ErrorFlags::MotorDisconnect), "motor_disc"},
        Utils::Log::BitDesc{static_cast<uint8_t>(Manager::ErrorFlags::RefereeDisConnect), "ref_disc"},
        Utils::Log::BitDesc{static_cast<uint8_t>(Manager::ErrorFlags::CAPDisConnect), "cap_disc"}};

    static constexpr std::array<Utils::Log::BitDesc, 4> kRlsReasonBitDesc = {
        Utils::Log::BitDesc{RLS_REASON_USER_DISABLED, "user_disabled"},
        Utils::Log::BitDesc{RLS_REASON_CAP_INVALID, "cap_invalid"},
        Utils::Log::BitDesc{RLS_REASON_LOW_MEASURED_POWER, "low_power"},
        Utils::Log::BitDesc{RLS_REASON_NONFINITE_SIGNAL, "nonfinite"}};

    Manager::Manager(
        std::deque<Hardware::DJIMotor> &motors_,
        const Division division_,
        RLSEnabled rlsEnabled_,
        const float k1_,
        const float k2_,
        const float k3_,
        const float lambda_)

        : rlsEnabled(rlsEnabled_),
          error(0UL),
          motors(motors_),
          division(division_),
          powerBuff(0.0f),
          fullBuffSet(0.0f),
          baseBuffSet(0.0f),
          fullMaxPower(0.0f),
          baseMaxPower(0.0f),
          powerUpperLimit(0.0f),
          refereeMaxPower(0.0f),
          userConfiguredMaxPower(0.0f),
          callback(nullptr),
          k1(k1_),
          k2(k2_),
          k3(k3_),
          lastUpdateTick(0),
          rls(1e-5f, 0.99999f) {
        (void)lambda_;
        // k1/k2/k3 物理上均应为非负，否则模型失真
        configASSERT(k1_ >= 0);
        configASSERT(k2_ >= 0);
        configASSERT(k3_ >= 0);

        // RLS 初值：用传入的 k1/k2 作为起始参数
        float initParams[2] = { k1_, k2_ };
        rls.setParamVector(Math::Matrixf<2, 1>(initParams));
    }

    static bool isInitialized;
    uint8_t xPowerTaskStack[1024];

    /**
     * @implements
     */
    void Manager::setMaxPowerConfigured(float maxPower) {
        // 最终可用上限会被运行时上下限再次夹紧
        userConfiguredMaxPower = std::clamp(maxPower, MIN_MAXPOWER_CONFIGURED, powerUpperLimit);
    }

    void Manager::setMode(uint8_t mode) {
        // mode=1: 放开到上限; 其他: 收敛到下限
        setMaxPowerConfigured(mode == 1 ? powerUpperLimit : powerLowerLimit);
    }

    /**
     * @implements
     */


std::array<float, 4> Manager::getControlledOutput(PowerObj *objs[4]) {
    std::array<float, 4> newTorqueCurrent{};
    static uint8_t lastOnlineMask = 0xFFU;
    static bool lastDegenerateRealloc = false;

    // 3508 近似扭矩常数(单位化后)，用于把“电流指令值”映射到“估算扭矩”
    // 0.3*(187/3591) 来自项目历史标定常数
    float torqueConst = 0.3 * ((float)187 / 3591);
    // k0: 电流指令 -> 扭矩(Nm)比例
    // 20/16384 来自 DJI 电流量程映射（20A 对应 16384 原始刻度）
    float k0 =
        torqueConst * 20 / 16384;  // torque current rate of the motor, defined as Nm/Output

    // sumCmdPower: 四轮在“未限功率”下的总估算功率
    float sumCmdPower = 0.0f;
    // cmdPower[i]: 第 i 轮的估算功率
    std::array<float, 4> cmdPower{};

    // sumError: 全轮速度误差和（只统计正功率轮）
    float sumError = 0.0f;
    std::array<float, 4> error{};

    // 当前循环可用最大功率，来自用户设置与能量环上下界综合约束
    float maxPower = std::clamp(userConfiguredMaxPower, fullMaxPower, baseMaxPower);

    // 可分配功率：先从 maxPower 起步，负功率轮会“返还预算”
    float allocatablePower = maxPower;
    // 正功率轮的总需求（用于按权重切分）
    float sumPowerRequired = 0.0f;
    uint8_t onlineMask = 0U;
#if USE_DEBUG
    static float newCmdPower;
#endif

    for (int i = 0; i < 4; i++) {
        PowerObj *p = objs[i];
        const bool motorOnline = detail::is_motor_connected(motors[i]);
        if (motorOnline) {
            onlineMask |= static_cast<uint8_t>(1U << i);
        } else {
            // 离线电机不参与功率估算与分配
            cmdPower[i] = 0.0f;
            error[i] = 0.0f;
            continue;
        }

        // 单轮功率模型:
        // P = τω + k1|ω| + k2τ² + k3/4
        // 第一项: 有效机械功（驱动/制动）
        // 第二项: 与速度相关损耗（粘性/风阻/机械摩擦等）
        // 第三项: 与电流平方相关损耗（铜损）
        // 第四项: 常量损耗均分到四轮
        cmdPower[i] = p->pidOutput * k0 * p->curAv + fabs(p->curAv) * k1 +
                      p->pidOutput * k0 * p->pidOutput * k0 * k2 + k3 / static_cast<float>(4);
        sumCmdPower += cmdPower[i];
        error[i] = fabs(p->setAv - p->curAv);

        // 对于负功率轮（回收/制动），其“功率需求”可视作给其他轮释放预算
        if (detail::float_equal(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f) {
            allocatablePower += -cmdPower[i];
        } else {
            sumError += error[i];
            sumPowerRequired += cmdPower[i];
        }
    }

    if (onlineMask != lastOnlineMask) {
        LOG_ERR(
            "[PWR_ALLOC] online_mask=0x%02X | m0:%s m1:%s m2:%s m3:%s\n",
            onlineMask,
            (onlineMask & 0x01U) ? "on" : "off",
            (onlineMask & 0x02U) ? "on" : "off",
            (onlineMask & 0x04U) ? "on" : "off",
            (onlineMask & 0x08U) ? "on" : "off");
        lastOnlineMask = onlineMask;
    }

    LOG_INFO(
        "sum power: %f, Max power: %f, Measured: %f, CapEnergy: %d\n",
        sumCmdPower,
        maxPower,
        measuredPower,
        robot_set->super_cap_info.capEnergy);

    // LOG_INFO("k1 %f k2 %f k3 %f max %f\n", k1, k2, k3, maxPower);

    // LOG_INFO("referee level %d\n",
    // robot_set->referee_info.game_robot_status_data.robot_level);

    //      update power status
    powerStatus.maxPowerLimited = maxPower;
    powerStatus.sumPowerCmd_before_clamp = sumCmdPower;

    // 仅在总需求超过上限时触发“限功率重分配”
    bool degenerateRealloc = false;
    if (sumCmdPower > maxPower) {
        if (!std::isfinite(sumPowerRequired) || sumPowerRequired <= 1e-5f) {
            // 极端退化场景：不做权重重分配，保持在线电机原输出并限幅
            degenerateRealloc = true;
            for (int i = 0; i < 4; i++) {
                if (!detail::is_motor_connected(motors[i])) {
                    newTorqueCurrent[i] = 0.0f;
                    continue;
                }
                PowerObj *p = objs[i];
                newTorqueCurrent[i] =
                    std::clamp(p->pidOutput, -p->pidMaxOutput, p->pidMaxOutput);
            }
        } else {
            float errorConfidence;
            // 权重混合策略:
            // - 误差大时，优先保控制误差（error 权重大）
            // - 误差小时，按功率占比分配（prop 权重大）
            if (sumError > error_powerDistribution_set) {
                errorConfidence = 1.0f;
            } else if (sumError > prop_powerDistribution_set) {
                errorConfidence = std::clamp(
                    (sumError - prop_powerDistribution_set) /
                        (error_powerDistribution_set - prop_powerDistribution_set),
                    0.0f,
                    1.0f);
            } else {
                errorConfidence = 0.0f;
            }
            for (int i = 0; i < 4; i++) {
                PowerObj *p = objs[i];
                if (!detail::is_motor_connected(motors[i])) {
                    newTorqueCurrent[i] = 0.0f;
                    continue;
                }

                // 负功率轮不参与削峰，保持原输出（它本身就在降总功率）
                if (detail::float_equal(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f) {
                    newTorqueCurrent[i] =
                        std::clamp(p->pidOutput, -p->pidMaxOutput, p->pidMaxOutput);
                    continue;
                }

                // 综合权重 = 误差权重 与 功率权重 的线性插值
                float powerWeight_Error =
                    (sumError > 1e-5f) ? (fabs(p->setAv - p->curAv) / sumError) : 0.0f;
                float powerWeight_Prop = cmdPower[i] / sumPowerRequired;
                float powerWeight = errorConfidence * powerWeight_Error +
                                    (1.0f - errorConfidence) * powerWeight_Prop;
                powerWeight = std::clamp(powerWeight, 0.0f, 1.0f);

                // 将“目标功率 = powerWeight * allocatablePower”代回功率二次式求电流:
                // k2*(k0*u)^2 + ω*(k0*u) + (k1|ω| + k3/4 - P_target) = 0
                // 判别式写成 delta，后续按根求新的电流指令
                float delta =
                    p->curAv * p->curAv - 4.0f * k2 *
                                              (k1 * fabs(p->curAv) + k3 / static_cast<float>(4) -
                                               powerWeight * allocatablePower);
                if (!std::isfinite(delta)) {
                    newTorqueCurrent[i] =
                        std::clamp(p->pidOutput, -p->pidMaxOutput, p->pidMaxOutput);
                    continue;
                }

                // delta=0: 重根
                if (detail::float_equal(delta, 0.0f))
                {
                    newTorqueCurrent[i] = -p->curAv / (2.0f * k2) / k0;
                } else if (delta > 0.0f)
                {
                    // 有两个实根时，按原 pidOutput 的符号选同向根，避免控制方向翻转
                    newTorqueCurrent[i] = p->pidOutput > 0.0f
                                              ? (-p->curAv + sqrtf(delta)) / (2.0f * k2) / k0
                                              : (-p->curAv - sqrtf(delta)) / (2.0f * k2) / k0;
                } else
                {
                    // 无实根时退化到抛物线顶点，保证有界
                    newTorqueCurrent[i] = -p->curAv / (2.0f * k2) / k0;
                }

                // 最后再做一次电流硬限幅
                newTorqueCurrent[i] =
                    std::clamp(newTorqueCurrent[i], -p->pidMaxOutput, p->pidMaxOutput);
            }
        }
    } else {
        for (int i = 0; i < 4; i++) {
            if (!detail::is_motor_connected(motors[i])) {
                newTorqueCurrent[i] = 0.0f;
            } else {
                newTorqueCurrent[i] =
                    std::clamp(objs[i]->pidOutput, -objs[i]->pidMaxOutput, objs[i]->pidMaxOutput);
            }
        }
    }

    if (degenerateRealloc != lastDegenerateRealloc) {
        LOG_ERR(
            "[PWR_ALLOC] degenerate: %s | sum_cmd=%.2f | max=%.2f | sum_req=%.5f | sum_err=%.5f\n",
            degenerateRealloc ? "on" : "off",
            sumCmdPower,
            maxPower,
            sumPowerRequired,
            sumError);
        lastDegenerateRealloc = degenerateRealloc;
    }

#if USE_DEBUG
    // 调试用途：统计限幅后的总估算功率
    float newCmdPower = 0.0f;
    for (int i = 0; i < 4; i++) {
        PowerObj *p = objs[i];
        newCmdPower += newTorqueCurrent[i] * k0 * p->curAv + fabs(p->curAv) * k1 +
                       newTorqueCurrent[i] * k0 * newTorqueCurrent[i] * k0 * k2 + k3 / 4.0f;
    }
    // LOG_INFO(
    //     "sumPower: %f, NewCMDPower power: %f, measuredPower: %f, capEnergy: %d\n",
    //     sumPowerRequired,
    //     newCmdPower,
    //     robot_set->super_cap_info.chassisPower,
    //     robot_set->super_cap_info.capEnergy);
#endif

    return newTorqueCurrent; 
}

    [[noreturn]] void Manager::powerDaemon() {
        // RLS 的输入向量:
        // samples[0] = Σ|ω|, samples[1] = Στ²
        static Math::Matrixf<2, 1> samples;
        static Math::Matrixf<2, 1> params;
        // effectivePower = Σ(τω) 项，代表有用机械功
        static float effectivePower = 0.0f;
        static uint8_t lastErrorMask = 0xFF;
        static bool lastCapConnected = false;
        static bool lastRefereeConnected = false;
        static bool lastRlsActive = false;
        static uint8_t lastRlsReasonMask = 0xFF;
        static bool powerGoodLatched = false;
        static bool capFeedbackHealthyLatched = false;
        static uint16_t rlsEnableDebounce = 0U;
        static uint16_t rlsDisableDebounce = 0U;
        static uint16_t capOkOnDebounce = 0U;
        static uint16_t capOkOffDebounce = 0U;

        isInitialized = true;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        lastUpdateTick = static_cast<size_t>(detail::now_ms());

        while (true) {
            // 默认保持高功率模式，随后由离线状态机夹紧边界
            setMode(1);

            const float torqueConst = 0.3f * (187.0f / 3591.0f);
            const float k0 = torqueConst * 20.0f / 16384.0f;
            const uint64_t nowMs = detail::now_ms();

            const bool capConnected =
                detail::is_cap_connected(*this, nowMs, CAP_OFFLINE_TIMEOUT_MS);
            const bool refereeConnected =
                detail::is_referee_connected(*this, nowMs, REFEREE_OFFLINE_TIMEOUT_MS);

            if (capConnected) {
                detail::clear_error_flag(error, Manager::ErrorFlags::CAPDisConnect);
            } else {
                detail::set_error_flag(error, Manager::ErrorFlags::CAPDisConnect);
            }
            if (refereeConnected) {
                detail::clear_error_flag(error, Manager::ErrorFlags::RefereeDisConnect);
            } else {
                detail::set_error_flag(error, Manager::ErrorFlags::RefereeDisConnect);
            }
            if (detail::are_all_motors_connected(*this)) {
                detail::clear_error_flag(error, Manager::ErrorFlags::MotorDisconnect);
            } else {
                detail::set_error_flag(error, Manager::ErrorFlags::MotorDisconnect);
            }

            // 能量反馈优先取超电，其次取裁判缓冲能量
            if (capConnected) {
                estimatedCapEnergy = robot_set->super_cap_info.capEnergy / 255.0f * 2100.0f;
                powerBuff = sqrtf(static_cast<float>(robot_set->super_cap_info.capEnergy));
                fullBuffSet = capFullBuffSet;
                baseBuffSet = capBaseBuffSet;
            } else {
                estimatedCapEnergy = 0.0f;
                powerBuff = refereeConnected
                                ? sqrtf(static_cast<float>(
                                      robot_set->referee_info.power_heat_data.chassis_power_buffer))
                                : 0.0f;
                fullBuffSet = refereeFullBuffSet;
                baseBuffSet = refereeBaseBuffSet;
            }

            // 裁判在线时更新基础功率上限和机器人等级，离线则回退到上次等级对应上限
            if (refereeConnected) {
                float refereeFeedbackLimit =
                    static_cast<float>(robot_set->referee_info.game_robot_status_data.chassis_power_limit);
                if (refereeFeedbackLimit <= 0.0f && capConnected) {
                    refereeFeedbackLimit =
                        static_cast<float>(robot_set->super_cap_info.chassisPowerlimit);
                }
                refereeMaxPower = fmax(refereeFeedbackLimit, CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD);
                uint8_t level = robot_set->referee_info.game_robot_status_data.robot_level;
                LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL =
                    std::clamp<uint8_t>(level == 0U ? 1U : level, 1U, maxLevel);

                if (capConnected) {
                    powerUpperLimit = refereeMaxPower + MAX_CAP_POWER_OUT;
                } else {
                    powerUpperLimit = refereeMaxPower + powerPD_base_pid_config.kp *
                                                            (sqrtf(refereeFullBuffSet) -
                                                             sqrtf(refereeBaseBuffSet));
                }
            } else {
                refereeMaxPower =
                    fmax(
                        detail::fallback_referee_limit(*this, LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL),
                        CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD);
                if (capConnected) {
                    powerUpperLimit = refereeMaxPower + MAX_CAP_POWER_OUT;
                } else {
                    powerUpperLimit = refereeMaxPower * CAP_REFEREE_BOTH_GG_COE;
                }
            }

            // 下限跟随当前基础上限动态变化，避免离线时仍保持过激输出
            MIN_MAXPOWER_CONFIGURED = fmax(refereeMaxPower * 0.8f, 15.0f);
            powerLowerLimit = MIN_MAXPOWER_CONFIGURED;

            // 能量环：双离线时禁用并采取保守功率
            if (!capConnected && !refereeConnected) {
                baseMaxPower = fullMaxPower = refereeMaxPower * CAP_REFEREE_BOTH_GG_COE;
                powerPD_base.clean();
                powerPD_full.clean();
            } else {
                powerPD_base.set(sqrtf(baseBuffSet));
                powerPD_full.set(sqrtf(fullBuffSet));
                baseMaxPower = fmax(refereeMaxPower - powerPD_base.out, MIN_MAXPOWER_CONFIGURED);
                fullMaxPower = fmax(refereeMaxPower - powerPD_full.out, MIN_MAXPOWER_CONFIGURED);
            }

            if (callback != nullptr) {
                setMaxPowerConfigured(callback());
            }

            // 基于反馈电流与转速，计算模型输入
            effectivePower = 0.0f;
            samples[0][0] = 0.0f;
            samples[1][0] = 0.0f;
            for (int i = 0; i < 4; ++i) {
                if (detail::is_motor_connected(motors[i])) {
                    motorDisconnectCounter[i] = 0U;
                } else if (motorDisconnectCounter[i] < MOTOR_DISCONNECT_HOLD_CYCLES) {
                    motorDisconnectCounter[i]++;
                }

                // 电机刚离线的短时间内仍保留上一状态估计，防止功率突降误判
                if (motorDisconnectCounter[i] < MOTOR_DISCONNECT_HOLD_CYCLES) {
                    effectivePower += motors[i].motor_measure_.given_current * k0 *
                                      detail::rpm_to_angular_velocity(motors[i].motor_measure_.speed_rpm);
                    samples[0][0] += fabsf(
                        detail::rpm_to_angular_velocity(motors[i].motor_measure_.speed_rpm));
                    samples[1][0] += motors[i].motor_measure_.given_current * k0 *
                                     motors[i].motor_measure_.given_current * k0;
                }
            }

            // 总估计功率 = k1*Σ|ω| + k2*Στ² + Στω + k3
            estimatedPower = k1 * samples[0][0] + k2 * samples[1][0] + effectivePower + k3;

            // 实测功率优先取超电，超电离线则回退到估计值（当前协议未使用裁判实时功率）
            measuredPower = capConnected ? robot_set->super_cap_info.chassisPower : estimatedPower;

            // 刷新对外状态（供 UI/调试）
            powerStatus.userConfiguredMaxPower = userConfiguredMaxPower;
            powerStatus.effectivePower = effectivePower;
            powerStatus.powerLoss = measuredPower - effectivePower;
            float measuredPowerSafe = fabsf(measuredPower) > 1e-5f ? measuredPower : 1e-5f;
            powerStatus.efficiency =
                std::clamp(effectivePower / measuredPowerSafe, 0.0f, 1.0f);
            powerStatus.estimatedCapEnergy =
                static_cast<uint8_t>(estimatedCapEnergy / 2100.0f * 255.0f);
            powerStatus.error = static_cast<Manager::ErrorFlags>(error);

            // RLS 更新门控（可诊断原因位）
            const bool capFeedbackHealthyRaw =
                detail::is_cap_feedback_healthy(*this, capConnected);
            if (capFeedbackHealthyRaw) {
                capOkOffDebounce = 0U;
                if (capOkOnDebounce < RLS_CAP_OK_ON_DEBOUNCE_CYCLES) {
                    ++capOkOnDebounce;
                }
                if (capOkOnDebounce >= RLS_CAP_OK_ON_DEBOUNCE_CYCLES) {
                    capFeedbackHealthyLatched = true;
                }
            } else {
                capOkOnDebounce = 0U;
                if (capOkOffDebounce < RLS_CAP_OK_OFF_DEBOUNCE_CYCLES) {
                    ++capOkOffDebounce;
                }
                if (capOkOffDebounce >= RLS_CAP_OK_OFF_DEBOUNCE_CYCLES) {
                    capFeedbackHealthyLatched = false;
                }
            }

            const float measuredPowerAbs = fabsf(measuredPower);
            if (powerGoodLatched) {
                if (measuredPowerAbs < RLS_UPDATE_POWER_OFF_THRESHOLD) {
                    powerGoodLatched = false;
                }
            } else if (measuredPowerAbs > RLS_UPDATE_POWER_ON_THRESHOLD) {
                powerGoodLatched = true;
            }

            const bool finiteSignal = std::isfinite(measuredPower) &&
                                      std::isfinite(samples[0][0]) &&
                                      std::isfinite(samples[1][0]) &&
                                      std::isfinite(effectivePower);
            uint8_t rlsReasonMask = 0U;
            if (rlsEnabled != Manager::RLSEnabled::Enable) {
                rlsReasonMask |= RLS_REASON_USER_DISABLED;
            }
            if (!capFeedbackHealthyLatched) {
                rlsReasonMask |= RLS_REASON_CAP_INVALID;
            }
            if (!powerGoodLatched) {
                rlsReasonMask |= RLS_REASON_LOW_MEASURED_POWER;
            }
            if (!finiteSignal) {
                rlsReasonMask |= RLS_REASON_NONFINITE_SIGNAL;
            }

            const bool rlsRawActive = (rlsReasonMask == 0U);
            bool rlsActive = lastRlsActive;
            if (rlsRawActive) {
                rlsDisableDebounce = 0U;
                if (rlsEnableDebounce < RLS_ENABLE_DEBOUNCE_CYCLES) {
                    ++rlsEnableDebounce;
                }
                if (!rlsActive && rlsEnableDebounce >= RLS_ENABLE_DEBOUNCE_CYCLES) {
                    rlsActive = true;
                }
            } else {
                rlsEnableDebounce = 0U;
                if (rlsDisableDebounce < RLS_DISABLE_DEBOUNCE_CYCLES) {
                    ++rlsDisableDebounce;
                }
                if (rlsActive && rlsDisableDebounce >= RLS_DISABLE_DEBOUNCE_CYCLES) {
                    rlsActive = false;
                }
            }

            if (rlsActive) {
                params = rls.update(samples, measuredPower - effectivePower - k3);
                k1 = fmax(params[0][0], 1e-5f);
                k2 = fmax(params[1][0], 1e-5f);
            }

            if (lastRlsActive != rlsActive || lastRlsReasonMask != rlsReasonMask) {
                char rlsReasonText[96] = {};
                Utils::Log::bitmask_to_cstr(
                    rlsReasonMask, kRlsReasonBitDesc, rlsReasonText, sizeof(rlsReasonText));
                LOG_ERR(
                    "[PWR_RLS] active: %s(raw:%s) | reason=0x%02X(%s) | en: %s | cap_ok: %s(raw:%s) | pwr_ok: %s | finite: %s | db:%u/%u | k1=%.5f | k2=%.5f | meas=%.2f\n",
                    rlsActive ? "on" : "off",
                    rlsRawActive ? "on" : "off",
                    rlsReasonMask,
                    rlsReasonText,
                    (rlsEnabled == Manager::RLSEnabled::Enable) ? "on" : "off",
                    capFeedbackHealthyLatched ? "on" : "off",
                    capFeedbackHealthyRaw ? "on" : "off",
                    powerGoodLatched ? "on" : "off",
                    finiteSignal ? "on" : "off",
                    rlsEnableDebounce,
                    rlsDisableDebounce,
                    k1,
                    k2,
                    measuredPower);
                lastRlsActive = rlsActive;
                lastRlsReasonMask = rlsReasonMask;
            }

            if (lastErrorMask != error || lastCapConnected != capConnected ||
                lastRefereeConnected != refereeConnected) {
                char fsmErrorText[96] = {};
                Utils::Log::bitmask_to_cstr(
                    error, kPowerErrorBitDesc, fsmErrorText, sizeof(fsmErrorText));
                LOG_ERR(
                    "[PWR_FSM] err=0x%02X(%s) | cap: %s | ref: %s | motor_all: %s | refMax=%.1f | upper=%.1f | base=%.1f | full=%.1f\n",
                    error,
                    fsmErrorText,
                    capConnected ? "on" : "off",
                    refereeConnected ? "on" : "off",
                    detail::are_all_motors_connected(*this) ? "on" : "off",
                    refereeMaxPower,
                    powerUpperLimit,
                    baseMaxPower,
                    fullMaxPower);
                lastErrorMask = error;
                lastCapConnected = capConnected;
                lastRefereeConnected = refereeConnected;
            }

            lastUpdateTick = static_cast<size_t>(nowMs);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    /**
     * @implements
     */
    void Manager::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        if (isInitialized)
            return;

        robot_set = robot;
        LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL =
            (division == Division::SENTRY) ? maxLevel : 1U;
        // 初始化阶段先给一组保守默认值，避免上电初期无界
        powerUpperLimit = CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD;
        powerPD_base = Pid::PidPosition(powerPD_base_pid_config, powerBuff);
        powerPD_full = Pid::PidPosition(powerPD_full_pid_config, powerBuff);
    }
}  // namespace Power
