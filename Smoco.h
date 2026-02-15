#ifndef SMOCO_H
#define SMOCO_H

#include <ACAN_T4.h>

#define SMOCO_CAN_BAUD_RATE 125000

#define SMOCO_MESSAGE_ID_POSITION 0x0
#define SMOCO_MESSAGE_ID_POSITION_CALIBRATED 0x1
#define SMOCO_MESSAGE_ID_TARGET 0x2
#define SMOCO_MESSAGE_ID_SMOOTHING 0x3
#define SMOCO_MESSAGE_ID_PID 0x4
#define SMOCO_MESSAGE_ID_SOFT_LIMIT 0x5
#define SMOCO_MESSAGE_ID_CALIBRATE 0x6
#define SMOCO_MESSAGE_ID_DEBUG 0x7
#define SMOCO_MESSAGE_ID_STOP 0xC
#define SMOCO_MESSAGE_ID_ERROR 0xD
#define SMOCO_MESSAGE_ID_ECHO_REQUEST 0xE
#define SMOCO_MESSAGE_ID_ECHO_REPLY 0xF

#define SMOCO_MESSAGE_SID_OPEN_LOOP 0x0
#define SMOCO_MESSAGE_SID_POSITION 0x2
#define SMOCO_MESSAGE_SID_VELOCITY 0x4
#define SMOCO_MESSAGE_SID_CURRENT 0x6

union SmocoCANMessage {
    uint64_t data64;
    struct __attribute__((__packed__)) {
        int32_t position;
        int16_t velocity;
        uint8_t current;
        uint8_t flags;
    } position;
    struct __attribute__((__packed__)) {
        uint8_t commandID;
    } commandError;
    struct __attribute__((__packed__)) {
        uint64_t payload;
    } echoReply;
    struct __attribute__((__packed__)) {
        uint8_t sid;
        union {
            struct __attribute__((__packed__)) {
                int16_t dutyCycle;
            } openLoop;
            struct __attribute__((__packed__)) {
                uint16_t errorGain;
                int32_t position;
            } targetPosition;
            struct __attribute__((__packed__)) {
                uint16_t errorGain;
                int32_t velocity;
            } targetVelocity;
            struct __attribute__((__packed__)) {
                uint16_t errorGain;
                int16_t current;
            } targetCurrent;
        };
    } target;
    struct __attribute__((__packed__)) {
        double rampRate;
    } setRampRate;
    struct __attribute__((__packed__)) {
        uint16_t p;
        uint16_t i;
        uint16_t d;
    } setPID;
    struct __attribute__((__packed__)) {
        int32_t aPosition;
        int32_t bPosition;
    } setSoftLimitPosition;
    struct __attribute__((__packed__)) {
        int16_t dutyCycle;
        int32_t limitSwitchPosition;
    } startPositionCalibration;
    struct __attribute__((__packed__)) {
        bool enable;
    } debugTelemetry;
    struct __attribute__((__packed__)) {
        uint64_t payload;
    } echoRequestPayload;
};
struct __attribute__((__packed__)) SmocoDebugTelemetry {
    uint64_t tick;
    int64_t position;
    double velocity;
    double current;
    double pOut;
    double iOut;
    double dOut;
    double error;
    double deltaT;
};

struct PID {
    float P;
    float I;
    float D;
};

class Smoco {
private:
    int32_t m_position; // (step)
    int16_t m_velocity; // (step/s)
    uint8_t m_current;  // (A)

    bool m_ignoreForwardLimit = false; // true: ignore forward limit switch actuation
    bool m_ignoreReverseLimit = false; // true: ignore reverse limit switch actuation
    int16_t m_dutyCycle;               // (1/32768)
    float m_errorGain;
    int32_t m_targetPosition; // (step)
    int32_t m_targetVelocity; // (step/s)
    float m_targetCurrent;    // (A)
    double m_rampRate;        // (1/s)
    struct PID m_PID;
    int32_t m_softLimitAPosition;   // (step)
    int32_t m_softLimitBPosition;   // (step)
    int16_t m_calibrationDutyCycle; // (1/32768)
    int32_t m_calibrationPosition;  // (step)
    bool m_debugTelemetryEnabled;   // true: SMoCo will continuously send debug
                                    // telemetry

    // true: received Position Calibrated message and calibratePosition has not been called since
    bool m_calibrated;

    bool m_limitSwitchA; // true: limit switch A depressed
    bool m_limitSwitchB; // true: limit switch B depressed
    bool m_softLimitA;   // true: soft limit A reached
    bool m_softLimitB;   // true: soft limit B reached

    uint32_t m_lastPingReply;      // (ms) time last ping reply was received
    uint64_t m_echoRequestPayload; // payload of most recent sent Echo Request
    uint64_t m_echoResponse;       // payload of most recent received Echo Reply
    // (ms) ping round trip time or UINT16_MAX after ping is called when millis() > m_echoRequestAt + m_pingTimeout
    uint64_t m_pingTime = UINT16_MAX;

    int32_t m_encoderZeroPosition = 0;
    float m_stepsPerDegree = 1;

    uint8_t m_commandErrorID; // ID of the last sent command SMoCo considered invalid

    SmocoDebugTelemetry m_debugTelemetry;

    bool sendRampRate();
    bool sendPID();
    bool sendSoftLimitPosition();

public:
    // Getters
    int32_t getPosition() const { return m_position; } // (step)
    int16_t getVelocity() const { return m_velocity; } // (step/s)
    uint8_t getCurrent() const { return m_current; }   // (A)

    bool getIgnoreForwardLimit() const { return m_ignoreForwardLimit; } // true: ignore forward limit switch actuation
    bool getIgnoreReverseLimit() const { return m_ignoreReverseLimit; } // true: ignore reverse limit switch actuation
    int16_t getDutyCycle() const { return m_dutyCycle; }                // (1/32768)
    float getErrorGain() const { return m_errorGain; }
    int32_t getTargetPosition() const { return m_targetPosition; } // (step)
    int32_t getTargetVelocity() const { return m_targetVelocity; } // (step/s)
    float getTargetCurrent() const { return m_targetCurrent; }     // (A)
    double getRampRate() const { return m_rampRate; }              // (1/s)
    PID getPID() const { return m_PID; }
    int32_t getSoftLimitAPosition() const { return m_softLimitAPosition; }     // (step)
    int32_t getSoftLimitBPosition() const { return m_softLimitBPosition; }     // (step)
    int16_t getCalibrationDutyCycle() const { return m_calibrationDutyCycle; } // (1/32768)
    int32_t getCalibrationPosition() const { return m_calibrationPosition; }   // (step)
    // true: SMoCo will continuously send debug telemetry
    bool getDebugTelemetryEnabled() const { return m_debugTelemetryEnabled; }

    // true: received Position Calibrated message and calibratePosition has not been called since
    bool getCalibrated() const { return m_calibrated; }

    bool getLimitSwitchA() const { return m_limitSwitchA; } // true: limit switch A depressed
    bool getLimitSwitchB() const { return m_limitSwitchB; } // true: limit switch B depressed
    bool getSoftLimitA() const { return m_softLimitA; }     // true: soft limit A reached
    bool getSoftLimitB() const { return m_softLimitB; }     // true: soft limit B reached
    bool isPositionWithinLimits(uint32_t position) const { return position > m_softLimitAPosition && position < m_softLimitBPosition; } // true: given position is between the soft limits

    uint32_t getLastPingReply() const { return m_lastPingReply; }           // (ms) time echo request was sent
    uint64_t getEchoRequestPayload() const { return m_echoRequestPayload; } // payload of most recent sent Echo Request
    uint64_t getEchoResponse() const { return m_echoResponse; } // payload of most recent received Echo Reply
    uint64_t getPingTime() const { return m_pingTime; }

    // ID of the last sent command SMoCo considered invalid
    uint8_t getCommandErrorID() const { return m_commandErrorID; }

    SmocoDebugTelemetry getDebugTelemetry() const { return m_debugTelemetry; }

    ACAN_T4 *canBus;
    uint32_t canID;              // Upper 2 nybbles of CAN ID
    uint64_t pingTimeout = 3000; // (ms)

    Smoco(ACAN_T4 *canBus, uint8_t canID);

    bool driveOpenLoop(int16_t dutyCycle);
    bool driveTargetPosition(int32_t targetPosition, float errorGain);
    bool driveTargetVelocity(int32_t targetVelocity, float errorGain);
    bool driveTargetCurrent(float targetCurrent, float errorGain);
    void configIgnoreLimits(bool forward, bool reverse); // NOTE: effective only after next driveXXXX() call
    bool setRampRate(double rampRate);
    bool setPID(float P, float I, float D);
    bool setSoftLimitPosition(int32_t positionA, int32_t positionB);
    bool calibratePosition(int16_t dutyCycle, int32_t limitSwitchPosition);
    bool debugTelemetry(bool enable);
    bool stopAndReset();
    bool echoRequest(uint64_t payload);
    void sync(CANMessage msg); // Update telemetry variables from the received
                               // message and send a response if necessary
    bool ping();

    // Angle based functions

    // configure conversion to apply to target position so that you can work with angles instead of raw steps
    void configAngleConversion(int32_t encoderZeroPosition, float stepsPerDegree) { m_encoderZeroPosition = encoderZeroPosition; m_stepsPerDegree = stepsPerDegree; }
    float stepsToDegrees(int32_t steps) const {
        return (steps - m_encoderZeroPosition) / m_stepsPerDegree;
    }
    int32_t degreesToSteps(float degrees) const {
        return (degrees * m_stepsPerDegree) + m_encoderZeroPosition;
    }
    float getAngle() const { return stepsToDegrees(m_position); } // (deg, i.e. units defined in configAngleConversion)
    float getAngularVelocity() const { return stepsToDegrees(m_velocity); } // (deg/s, i.e. units defined in configAngleConversion)
    float getTargetAngle() const { return stepsToDegrees(m_targetPosition); } // (deg, i.e. units defined in configAngleConversion)
    float getTargetAngularVelocity() const { return stepsToDegrees(m_targetVelocity); } // (deg/s, i.e. units defined in configAngleConversion)
    float getSoftLimitAAngle() const { return stepsToDegrees(m_softLimitAPosition); } // (deg, i.e. units defined in configAngleConversion)
    float getSoftLimitBAngle() const { return stepsToDegrees(m_softLimitBPosition); } // (deg, i.e. units defined in configAngleConversion)
    bool isAngleWithinLimits(float angle) const { return isPositionWithinLimits(degreesToSteps(angle)); } // isPositionWithinLimits but with angle conversion applied
    bool driveTargetAngle(float targetAngle, float errorGain) { return driveTargetPosition(degreesToSteps(targetAngle), errorGain); } // driveTargetPosition but with angle conversion applied
    bool driveTargetAngularVelocity(float targetVelocity, float errorGain) { return driveTargetVelocity(degreesToSteps(targetVelocity), errorGain); } // driveTargetVelocity but with angle conversion applied
    bool setSoftLimitAngle(float angleA, float angleB) { return setSoftLimitPosition(degreesToSteps(angleA), degreesToSteps(angleB)); } // setSoftLimitPosition but with angle conversion applied
    bool calibrateAngle(int16_t dutyCycle, float limitSwitchAngle) { return calibratePosition(dutyCycle, degreesToSteps(limitSwitchAngle)); } // calibratePosition but with angle conversion applied

};
#endif
