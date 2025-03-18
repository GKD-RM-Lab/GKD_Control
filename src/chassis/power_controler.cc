#include <algorithm>
#include <thread>

#include "power_controller.hpp"

namespace Power
{

    PowerStatus powerStatus;
    static uint8_t LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL;
    static bool isCapEnergyOut = false;
    static uint16_t motorDisconnectCounter[4] = { 0U, 0U, 0U, 0U };
    static float MIN_MAXPOWER_CONFIGURED = 30.0f;

    static inline bool floatEqual(float a, float b) {
        return fabs(a - b) < 1e-5f;
    }

    static inline float rpm2av(float rpm) {
        return rpm * (float)M_PI / 30.0f;
    }

    static inline float av2rpm(float av) {
        return av * 30.0f / (float)M_PI;
    }

    static inline void setErrorFlag(uint8_t &curFlag, Manager::ErrorFlags setFlag) {
        curFlag |= static_cast<uint8_t>(setFlag);
    }

    static inline void clearErrorFlag(uint8_t &curFlag, Manager::ErrorFlags clearFlag) {
        curFlag &= (~static_cast<uint8_t>(clearFlag));
    }

    static inline bool isFlagged(uint8_t &curFlag, Manager::ErrorFlags flag) {
        return (curFlag & static_cast<uint8_t>(flag)) != 0;
    }

    Manager::Manager(
        std::deque<Hardware::DJIMotor> &motors_,
        const Division division_,
        RLSEnabled rlsEnabled_,
        const float k1_,
        const float k2_,
        const float k3_,
        const float lambda_)

        : rlsEnabled(Manager::RLSEnabled::Enable),
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
        configASSERT(k1_ >= 0);
        configASSERT(k2_ >= 0);
        configASSERT(k3_ >= 0);

        float initParams[2] = { k1_, k2_ };
        rls.setParamVector(Math::Matrixf<2, 1>(initParams));
    }

    static bool isInitialized;
    uint8_t xPowerTaskStack[1024];

    /**
     * @implements
     */
    void Manager::setMaxPowerConfigured(float maxPower) {
        userConfiguredMaxPower = std::clamp(maxPower, MIN_MAXPOWER_CONFIGURED, powerUpperLimit);
    }

    void Manager::setMode(uint8_t mode) {
        setMaxPowerConfigured(mode == 1 ? powerUpperLimit : refereeMaxPower);
    }

    /**
     * @implements
     */

    float *Manager::getControlledOutput(PowerObj *objs[4]) {
        static float newTorqueCurrent[4];
        float torqueConst = 0.3 * ((float)187 / 3591);
        float k0 =
            torqueConst * 20 / 16384;  // torque current rate of the motor, defined as Nm/Output

        float sumCmdPower = 0.0f;
        float cmdPower[4];

        float sumError = 0.0f;
        float error[4];

        float maxPower = std::clamp(userConfiguredMaxPower, fullMaxPower, baseMaxPower);

        float allocatablePower = maxPower;
        float sumPowerRequired = 0.0f;
#if USE_DEBUG
        static float newCmdPower;
#endif

        for (int i = 0; i < 4; i++) {
            PowerObj *p = objs[i];
            cmdPower[i] = p->pidOutput * k0 * p->curAv + fabs(p->curAv) * k1 +
                          p->pidOutput * k0 * p->pidOutput * k0 * k2 + k3 / static_cast<float>(4);
            sumCmdPower += cmdPower[i];
            error[i] = fabs(p->setAv - p->curAv);
            if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f) {
                allocatablePower += -cmdPower[i];
            } else {
                sumError += error[i];
                sumPowerRequired += cmdPower[i];
            }
        }

        // LOG_INFO(
        //     "sum power %d %f %f %f\n",
        //     robot_set->super_cap_info.capEnergy,
        //     measuredPower,
        //     estimatedPower,
        //     sumCmdPower);
        //      update power status
        powerStatus.maxPowerLimited = maxPower;
        powerStatus.sumPowerCmd_before_clamp = sumCmdPower;

        if (sumCmdPower > maxPower) {
            float errorConfidence;
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
                if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f) {
                    newTorqueCurrent[i] = p->pidOutput;
                    continue;
                }
                float powerWeight_Error = fabs(p->setAv - p->curAv) / sumError;
                float powerWeight_Prop = cmdPower[i] / sumPowerRequired;
                float powerWeight = errorConfidence * powerWeight_Error +
                                    (1.0f - errorConfidence) * powerWeight_Prop;
                float delta =
                    p->curAv * p->curAv - 4.0f * k2 *
                                              (k1 * fabs(p->curAv) + k3 / static_cast<float>(4) -
                                               powerWeight * allocatablePower);
                if (floatEqual(delta, 0.0f))  // repeat roots
                {
                    newTorqueCurrent[i] = -p->curAv / (2.0f * k2) / k0;
                } else if (delta > 0.0f)  // distinct roots
                {
                    newTorqueCurrent[i] = p->pidOutput > 0.0f
                                              ? (-p->curAv + sqrtf(delta)) / (2.0f * k2) / k0
                                              : (-p->curAv - sqrtf(delta)) / (2.0f * k2) / k0;
                } else  // imaginary roots
                {
                    newTorqueCurrent[i] = -p->curAv / (2.0f * k2) / k0;
                }
                // WARN: Not sure about this clamp
                // newTorqueCurrent[i] = Utils::Math::clamp(newTorqueCurrent[i], p->pidMaxOutput);
                newTorqueCurrent[i] =
                    std::clamp(newTorqueCurrent[i], -p->pidMaxOutput, p->pidMaxOutput);
            }
        } else {
            for (int i = 0; i < 4; i++) {
                newTorqueCurrent[i] = objs[i]->pidOutput;
            }
        }

#if USE_DEBUG
        newCmdPower = 0.0f;
        for (int i = 0; i < 4; i++) {
            PowerObj *p = objs[i];
            newCmdPower += newTorqueCurrent[i] * k0 * p->curAv + fabs(p->curAv) * manager.k1 +
                           newTorqueCurrent[i] * k0 * newTorqueCurrent[i] * k0 * manager.k2 +
                           manager.k3 / 4.0f;
        }
#endif

        return newTorqueCurrent;
    }

    void Manager::powerDaemon [[noreturn]] () {
        static Math::Matrixf<2, 1> samples;
        static Math::Matrixf<2, 1> params;
        static float effectivePower = 0;

        isInitialized = true;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        lastUpdateTick = clock();

        while (true) {
            float torqueConst = 0.3 * ((float)187 / 3591);
            float k0 =
                torqueConst * 20 / 16384;  // torque current rate of the motor, defined as Nm/Output
            // FIXME: DEBUG SET LEVEL TO 1
            robot_set->referee_info.game_robot_status_data.robot_level = 1;

            size_t now = clock();

            // update rls state and check whether cap energy is out even when cap
            // disconnect to utilize credible data from referee system for the rls model
            // estimate the cap energy if cap disconnect
            // estimated cap energy = cap energy feedback when cap is connected
            isCapEnergyOut = false;
            estimatedCapEnergy = robot_set->super_cap_info.capEnergy / 255.0f * 2100.0f;

            // Set the power buff and buff set based on the current state
            // Take cap message as priority
            // If disconnect from cap or disable the cap, then take the referee system's
            // power buffer as feedback If referee system is disconnected, then we need
            // to disable the energy loop and treat power loop conservatively When both
            // cap and referee are disconnected, we disable the energy loop and
            // therefore no need to update the powerBuff and buffSet
            //
            // Set the energy feedback based on the current error status
            powerBuff = sqrtf(robot_set->super_cap_info.capEnergy);

            // Set the energy target based on the current error status
            fullBuffSet = capFullBuffSet;  // 230
            baseBuffSet = capBaseBuffSet;  // 30

            // Update the referee maximum power limit and user configured power limit
            // If disconnected, then restore the last robot level and find corresponding
            // chassis power limit
            refereeMaxPower = fmax(
                robot_set->super_cap_info.chassisPowerlimit,
                CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD);

            if (robot_set->referee_info.game_robot_status_data.robot_level > 10U)
                LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = 1U;
            else
                LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL =
                    fmax(1U, robot_set->referee_info.game_robot_status_data.robot_level);

            powerUpperLimit = refereeMaxPower + MAX_CAP_POWER_OUT;

            MIN_MAXPOWER_CONFIGURED = refereeMaxPower * 0.8f;

            // energy loop
            // if cap and referee both gg, set the max power to latest power limit *
            // 0.85 and disable energy loop if referee gg, set the max power to latest
            // power limit * 0.95, enable energy loop when cap energy out
            powerPD_base.set(sqrtf(baseBuffSet));
            powerPD_full.set(sqrtf(fullBuffSet));
            baseMaxPower = fmax(refereeMaxPower - powerPD_base.out, MIN_MAXPOWER_CONFIGURED);
            fullMaxPower = fmax(refereeMaxPower - powerPD_full.out, MIN_MAXPOWER_CONFIGURED);

            // Estimate the power based on the current model
            effectivePower = 0;
            samples[0][0] = 0;
            samples[1][0] = 0;
            for (int i = 0; i < 4; i++) {
                effectivePower += motors[i].motor_measure_.given_current * k0 *
                                  rpm2av(motors[i].motor_measure_.speed_rpm);
                samples[0][0] += fabsf(rpm2av(motors[i].motor_measure_.speed_rpm));
                samples[1][0] += motors[i].motor_measure_.given_current * k0 *
                                 motors[i].motor_measure_.given_current * k0;
            }
            estimatedPower = k1 * samples[0][0] + k2 * samples[1][0] + effectivePower + k3;

            // Get the measured power from cap
            // If cap is disconnected, get measured power from referee feedback if cap
            // energy is out Otherwise, set it to estimated power
            measuredPower = robot_set->super_cap_info.chassisPower;
            // LOG_INFO(
            //     "%f %f %f %f %f %f\n", measuredPower, effectivePower, estimatedPower, k1, k2,
            //     k3);
            printf("%f, %f\n", baseMaxPower, fullMaxPower);

            // update power status
            powerStatus.userConfiguredMaxPower = userConfiguredMaxPower;
            powerStatus.effectivePower = effectivePower;
            powerStatus.powerLoss = measuredPower - effectivePower;
            powerStatus.efficiency = std::clamp(effectivePower / measuredPower, 0.0f, 1.0f);
            powerStatus.estimatedCapEnergy =
                static_cast<uint8_t>(estimatedCapEnergy / 2100.0f * 255.0f);
            powerStatus.error = static_cast<Manager::ErrorFlags>(error);

            // Update the RLS parameterMAX_CAP_POWER_OUTs AND
            // Add dead zone AND
            // The Referee System could not detect negative power, leading to failure of
            // real measurement. So use estimated power to evaluate this situtation
            if (fabs(measuredPower) > 5.0f) {
                params = rls.update(samples, measuredPower - effectivePower - k3);
                k1 = fmax(params[0][0],
                          1e-5f);  // In case the k1 diverge to negative number
                k2 = fmax(params[1][0],
                          1e-5f);  // In case the k2 diverge to negative number
            }

            lastUpdateTick = now;

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
        LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = division == Division::SENTRY ? 10U : 1U;
        MIN_MAXPOWER_CONFIGURED = 30.0f;
        powerUpperLimit = CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD + MAX_CAP_POWER_OUT;
        powerPD_base = Pid::PidRad(powerPD_base_pid_config, powerBuff);
        powerPD_full = Pid::PidRad(powerPD_full_pid_config, powerBuff);
    }

}  // namespace Power
