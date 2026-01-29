#ifndef SMOCO_H
#define SMOCO_H

#include <ACAN_T4.h>

enum class ControlMode { STOP, OPEN_LOOP, POSITION, VELOCITY, CURRENT, CALIBRATING };

class Smoco {

private:

    ACAN_T4 *m_canBus;
    uint8_t m_canID;

    //int16_t m_dutyCycle;

    int32_t m_angle;
    int16_t m_angularVelocity;
    uint8_t m_current;

    bool m_ignoreLimit;
    uint16_t m_errorGain;
    uint16_t m_lowPassSmoothingAlpha;

    bool m_isCalibrated;

    uint16_t m_PID[3] = {0,0,0};

    int32_t m_softLimitA;
    int32_t m_softLimitB;

    bool m_limSwitchA;
    bool m_limSwitchB;
    bool m_softLimA;
    bool m_softLimB;

    uint64_t m_pingTime;

    uint8_t m_commandErrorID; // :)
    bool m_isPinging;

    uint64_t m_echoData;

public:
    // constructor
    Smoco(ACAN_T4 *canBus, uint8_t canID);

    void setCanID(uint8_t canID);

    //void sendTelemetry();

    void openLoopDrive(int16_t dutyCycle, bool ignoreLimit = false);
    void setJointAngle(uint32_t targetAngle, uint16_t errorGain, bool ignoreLimit = false);
    void setJointVelocity(uint32_t targetVelocity, uint16_t errorGain, bool ignoreLimit = false);
    void setJointCurrent(int16_t targetCurrent, uint16_t errorGain, bool ignoreLimit = false);
    void setLowPassSmoothingFactor(uint16_t alpha);
    void setPID(uint16_t P, uint16_t I, uint16_t D);
    void setSoftLimitPosition(int32_t positionA, int32_t positionB);
    void startPositionCalibration(int16_t dutyCycle, int32_t limitSwitchPosition);
    void debugTelemetry(uint8_t enable);
    void stopAndReset();
    void echoRequest(uint64_t payload);
    void readIncomingMessage(CANMessage msg);
    void smocoPing();

    // Setters
    void setAngleVariable(int32_t angle);
    void setPIDVariables(uint16_t P, uint16_t I, uint16_t D);
    void setAlphaVariable(uint16_t alpha);
    void setSoftLimitAVariable(int32_t limitA);
    void setSoftLimitBVariable(int32_t limitB);
    void setIgnoreLimitVariable(bool ignoreLim);
    void setPingTimeVariable(uint64_t pingTIme);

    // getters
    int32_t getAngleVariable();
    bool getignoreLimitVariable();

    bool getSoftLimitAVariable();
    bool getSoftLimitBVariable();
    bool getLimitSwitchAVariable();
    bool getLimitSwitchBVariable();
    uint64_t getEchoDataVariable();
    uint64_t getPingTimeVariable();
};
#endif










