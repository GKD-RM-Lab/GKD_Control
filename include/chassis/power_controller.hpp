/**
 * @file PowerController.hpp
 * @version 2.0
 * @note The estimated power formula: P = τΩ + k1|Ω| + k2τ^2
 */
#pragma once

#include <cstdint>
#include <deque>

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

namespace Power
{

    constexpr static float refereeFullBuffSet = 60.0f;
    constexpr static float refereeBaseBuffSet = 50.0f;
    constexpr static float capFullBuffSet = 250.0f;
    constexpr static float capBaseBuffSet = 100.0f;
    constexpr static float error_powerDistribution_set = 20.0f;
    constexpr static float prop_powerDistribution_set = 15.0f;

    // constexpr float MIN_MAXPOWER_CONFIGURED                   = 15.0f;
    constexpr float MAX_CAP_POWER_OUT = 300.0f;
    constexpr float CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD = 43.0f;
    constexpr float CAP_OFFLINE_ENERGY_TARGET_POWER = 37.0f;
    constexpr float MAX_POEWR_REFEREE_BUFF = 60.0f;
    constexpr float REFEREE_GG_COE = 0.95f;
    constexpr float CAP_REFEREE_BOTH_GG_COE = 0.85f;

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
    constexpr static uint8_t maxLevel = 10U;
    constexpr static uint8_t HeroChassisPowerLimit_HP_FIRST[maxLevel] = {
        55U, 60U, 65U, 70U, 75U, 80U, 85U, 90U, 100U, 120U
    };
    constexpr static uint8_t InfantryChassisPowerLimit_HP_FIRST[maxLevel] = { 45U, 50U, 55U, 60U,
                                                                              65U, 70U, 75U, 80U,
                                                                              90U, 100U };
    constexpr static uint8_t SentryChassisPowerLimit = 100U;

    enum class Division
    {
        INFANTRY = 0,
        HERO,
        SENTRY
    };

    struct PowerObj
    {
       public:
        float pidOutput;     // torque current command, [-maxOutput, maxOutput], no unit
        float curAv;         // Measured angular velocity, [-maxAv, maxAv], rad/s
        float setAv;         // target angular velocity, [-maxAv, maxAv], rad/s
        float pidMaxOutput;  // pid max output
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

        float powerBuff;
        float fullBuffSet;
        float baseBuffSet;
        float fullMaxPower;
        float baseMaxPower;

        float powerUpperLimit;
        float powerLowerLimit;
        float refereeMaxPower;

        float userConfiguredMaxPower;
        float (*callback)(void);

        float measuredPower;
        float estimatedPower;
        float estimatedCapEnergy;

        float k1;
        float k2;
        float k3;

        size_t lastUpdateTick;

        Math::RLS<2> rls;

        ControllerList powerPD_base;
        ControllerList powerPD_full;

        std::shared_ptr<Robot::Robot_set> robot_set;
        std::shared_ptr<Device::Super_Cap> supercap;
        std::shared_ptr<Device::Dji_referee> referee;

        void init(const std::shared_ptr<Robot::Robot_set> &robot);
        float *getControlledOutput(PowerObj *objs[4]);
        void setMaxPowerConfigured(float maxPower);
        void setMode(uint8_t mode);
        void powerDaemon [[noreturn]] ();
    };

#define POWER_PD_KP 50.0f
    const typename Pid::PidConfig powerPD_base_pid_config{
        POWER_PD_KP, 0.0f, 0.2f, MAX_CAP_POWER_OUT, 0.0f,
    };
    const typename Pid::PidConfig powerPD_full_pid_config{
        POWER_PD_KP, 0.0f, 0.2f, MAX_CAP_POWER_OUT, 0.0f,
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
    float getLatestFeedbackJudgePowerLimit();

    /**
     * @brief Get the controlled output torque current based on current model
     * @param objs The collections of power objects from four wheels, recording the
     * necessary data from the PID controller
     * @retval The controlled output torque current
     */
    float *getControlledOutput(PowerObj *objs[4]);

    /**
     * @brief return the power status of the chassis
     * @retval The power status object
     */
    const volatile PowerStatus &getPowerStatus();

    /**
     * @brief The power controller module initialization function
     * @param manager The manager object
     * @note This function should be called before the scheduler starts
     */

    /**
     * @brief set the user configured max power
     * @param maxPower The max power value
     * @note The max power configured by this function will compete with the basic
     * energy limitation, to ensure system does not die
     */
    void setMaxPowerConfigured(float maxPower);

    void setMode(uint8_t mode);

    void registerPowerCallbackFunc(float (*callback)(void));

    /**
     * @brief Enable for disable the automatically parameters update process
     * @param isUpdate disable with 0, enable with 1
     * @note  The system will automatically disable the update when both referee
     * system and cap is disconnect from the power module
     * @retval None
     */
    void setRLSEnabled(uint8_t isUpdate);

}  // namespace Power
