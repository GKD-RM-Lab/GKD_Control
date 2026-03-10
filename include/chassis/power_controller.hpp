/**
 * @file PowerController.hpp
 * @version 2.0
 *
 * @note
 * 功率模型（单轮）:
 *   P_wheel ≈ τ * ω + k1 * |ω| + k2 * τ^2 + k3 / 4
 * 其中:
 *   τ: 电机输出扭矩（由电流指令估算）
 *   ω: 电机角速度
 *   k1: 与粘性阻尼/机械损耗相关
 *   k2: 与铜损/电流平方相关
 *   k3: 与整车基础损耗相关（4 轮均分）
 *
 * 本模块做两件事：
 * 1) 用上式估计底盘功率并在线更新参数(k1,k2)
 * 2) 当总功率超限时，对四轮电流指令进行再分配，优先保证控制误差大的轮子
 */
#pragma once

#include <cstdint>
#include <deque>
#include <array>
#include "dji_motor.hpp"
#include "pid_controller.hpp"
#include "referee.hpp"
#include "robot.hpp"
#include "super_cap.hpp"

#define USE_POWER_CONTROLLER TRUE

// If the capacitor is plugged into the circuit, make sure you enable the super
// cap module successfully Otherwise, it will cause unexpected behavior of the
// RLS model
#define USE_SUPER_CAPACITOR TRUE

#define USE_REFEREE_SYSTEM_COMM TRUE

#include "utils/RLS.hpp"

#include "logger/logger.hpp"

namespace Power
{

    // 以下为能量环/功率分配阈值，单位与对应反馈一致
    constexpr static float refereeFullBuffSet = 60.0f;   // 裁判系统“满功率模式”缓冲目标
    constexpr static float refereeBaseBuffSet = 50.0f;   // 裁判系统“保守模式”缓冲目标
    constexpr static float capFullBuffSet = 250.0f;      // 超级电容“满功率模式”缓冲目标
    constexpr static float capBaseBuffSet = 50.0f;      // 超级电容“保守模式”缓冲目标
    constexpr static float error_powerDistribution_set = 20.0f; // 误差优先权重切换上阈值
    constexpr static float prop_powerDistribution_set = 15.0f;  // 比例优先权重切换下阈值

    // constexpr float MIN_MAXPOWER_CONFIGURED                   = 15.0f;
    constexpr float MAX_CAP_POWER_OUT = 300.0f;                    // 超级电容可额外释放的最大功率
    constexpr float CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD = 43.0f; // 电容离线时保底功率上限阈值
    constexpr float CAP_OFFLINE_ENERGY_TARGET_POWER = 60.0f;       // 电容离线时建议目标功率
    constexpr float MAX_POEWR_REFEREE_BUFF = 60.0f;                // 裁判系统缓冲上限
    constexpr float REFEREE_GG_COE = 0.95f;                        // 裁判离线时保守系数（主参数）
    constexpr float CAP_REFEREE_BOTH_REL_COE = 0.8947368f;         // 双离线相对 REFEREE_GG_COE 的比例
    constexpr float CAP_REFEREE_BOTH_GG_COE =
        REFEREE_GG_COE * CAP_REFEREE_BOTH_REL_COE;                 // 裁判+电容都离线时保守系数
    const typename Pid::PidConfig powerPD_base_pid_config{
        50.f, 0.0f, 0.2f, MAX_CAP_POWER_OUT, 0.0f,
    };
    const typename Pid::PidConfig powerPD_full_pid_config{
        20.f, 0.0f, 0.03f, MAX_CAP_POWER_OUT, 0.0f,
    };
    /**
     * @brief The Power Limit and max HP enumeration attributed by division, chassis
     * type and level
     * @note  Copy from RM2024 Official Rule Manual
     * @attention The infantry data list only suits for standard infantry, but not
     * balanced infantry
     * @attention if the pilot changes the chassis type before the game officially
     * start, and simultaneously the referee system is disconnected before chassis
     * type changed, there will be problem of distinguishing the chassis type, so we
     * choose HP_FIRST chassis type conservatively, except for sentry
     */
    constexpr static uint8_t maxLevel = 11U; //最高等级
    constexpr static uint8_t HeroChassisPowerLimit_HP_FIRST[maxLevel] = { 100U,   100U,  100U, 100U,
                                                                          100U, 100U,  100U, 100U,
                                                                          100U, 100U, 100U }; // 英雄各等级功率限制
    constexpr static uint8_t InfantryChassisPowerLimit_HP_FIRST[maxLevel] = { 75U,   75U, 75U, 75U,
                                                                              75U, 75U, 75U, 75U,
                                                                              75U, 75U, 75U };// 步兵各等级功率限制
    constexpr static uint8_t SentryChassisPowerLimit = 100U;

    enum class Division
    {
        INFANTRY, // 0
        HERO,     // 1
        SENTRY    // 2
    };

    struct PowerObj
    {
       public:
        // 来自底盘速度环的数据（每个轮子一份）
        float pidOutput;     // 速度环输出的电流指令（DJI原始量纲）
        float curAv;         // 当前角速度(rad/s)
        float setAv;         // 目标角速度(rad/s)
        float pidMaxOutput;  // 电流指令饱和上限（DJI原始量纲）
    };

    struct Manager
    {
        enum RLSEnabled : bool
        {
            Disable = 0,
            Enable = 1
        } rlsEnabled;

        enum ErrorFlags
        {
            MotorDisconnect = 1U,
            RefereeDisConnect = 2U,
            CAPDisConnect = 4U
        };

        uint8_t error;

        /**
         * @remark In case of initialization without explicit datas
         */
        Manager() = delete;

        Manager(
            std::deque<Hardware::DJIMotor> &motors_,
            const Division division_,
            RLSEnabled rlsEnabled_ = Enable,
            const float k1_ = 0.22f,
            const float k2_ = 1.2f,
            const float k3_ = 2.78f,
            const float lambda_ = 0.9999f);

        std::deque<Hardware::DJIMotor> &motors;
        Division division;

        // 能量环状态
        float powerBuff;      // 当前能量反馈（经过 sqrt 变换后的“缓冲状态”）
        float fullBuffSet;    // 满功率模式能量目标
        float baseBuffSet;    // 基础功率模式能量目标
        float fullMaxPower;   // 满功率模式计算得到的最大允许功率
        float baseMaxPower;   // 基础功率模式计算得到的最大允许功率

        // 功率上下限
        float powerUpperLimit; // 当前可配置上限
        float powerLowerLimit; // 当前可配置下限
        float refereeMaxPower; // 来自裁判/电容反馈的基础功率上限

        // 用户配置与外部回调
        float userConfiguredMaxPower; // 用户请求的功率上限（最终仍会被系统夹紧）
        float (*callback)(void);

        // 功率估计观测量
        float measuredPower;      // 实测功率（优先取电容反馈）
        float estimatedPower;     // 模型估计功率
        float estimatedCapEnergy; // 估计电容能量

        // 在线辨识参数（功率模型）
        float k1; // 速度损耗项系数
        float k2; // 电流平方损耗项系数
        float k3; // 常量损耗项

        size_t lastUpdateTick;

        Math::RLS<2> rls;

        // 能量环（本质是对 powerBuff -> buffSet 的 PD 控制）
        ControllerList powerPD_base;
        ControllerList powerPD_full;

        std::shared_ptr<Robot::Robot_set> robot_set;
        std::shared_ptr<Device::Super_Cap> supercap;
        std::shared_ptr<Device::Dji_referee> referee;

        void init(const std::shared_ptr<Robot::Robot_set> &robot);
        std::array<float, 4> getControlledOutput(PowerObj *objs[4]);
        void setMaxPowerConfigured(float maxPower);
        void setMode(uint8_t mode); //功率最大值设置
        [[noreturn]] void powerDaemon (); //电源守护进程
    };



    /**
     * @brief Storing the power status of the chassis
     */
    struct PowerStatus
    {
       public:
        float userConfiguredMaxPower;
        float maxPowerLimited;
        float sumPowerCmd_before_clamp;
        float effectivePower;
        float powerLoss;
        float efficiency;
        uint8_t estimatedCapEnergy;
        Manager::ErrorFlags error;
    };

    // return the latest feedback referee power limit(before referee disconnected),
    // according to the robot level
    //TODO 完成定义
    float getLatestFeedbackJudgePowerLimit();

    /**
     * @brief Get the controlled output torque current based on current model
     * @param objs The collections of power objects from four wheels, recording the
     * necessary data from the PID controller
     * @retval The controlled output torque current
     */
    std::array<float, 4> getControlledOutput(PowerObj *objs[4]);

    /**
     * @brief return the power status of the chassis
     * @retval The power status object
     */
    const volatile PowerStatus &getPowerStatus();

    /**
     * @brief set the user configured max power
     * @param maxPower The max power value
     * @note The max power configured by this function will compete with the basic
     * energy limitation, to ensure system does not die
     */
    void setMaxPowerConfigured(float maxPower);


    /**
     * @brief Enable for disable the automatically parameters update process
     * @param isUpdate disable with 0, enable with 1
     * @note  The system will automatically disable the update when both referee
     * system and cap is disconnect from the power module
     * @retval None
     */
    void setRLSEnabled(uint8_t isUpdate);

    void setMode(uint8_t mode);

    void registerPowerCallbackFunc(float (*callback)(void));

}  // namespace Power
