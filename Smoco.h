#ifndef SMOCO_H
#define SMOCO_H

#include <ACAN_T4.h>

#include "smoco_types.h"

// backwards compatibility
using SmocoDebugTelemetry = DebugTelemetry;
using SmocoCANMessage = SMOCOMessage;

#define SMOCO_CAN_BAUD_RATE 125000

class Smoco {
public:
    struct PID {
        float P;
        float I;
        float D;
    };

    struct DutyCycleRange {
        int16_t minForward; // (1/32768)
        int16_t maxForward; // (1/32768)
        int16_t minReverse; // (1/32768)
        int16_t maxReverse; // (1/32768)
    };

private:
    int32_t m_position; // (step)
    int16_t m_velocity; // (step/s)
    uint8_t m_current;  // (A)

    bool m_ignoreLimitForward = false; // true: ignore limit switch B actuation
    bool m_ignoreLimitReverse = false; // true: ignore limit switch A actuation
    int16_t m_dutyCycle;               // (1/32768)
    DutyCycleRange m_dutyCycleRange =  // (1/32768)
        {.minForward = 0, .maxForward = INT16_MAX, .minReverse = 0, .maxReverse = INT16_MIN};
    int32_t m_targetPosition; // (step)
    float m_targetVelocity;   // (step/s)
    float m_targetCurrent;    // (A)
    float m_rampRate;         // (1/s)
    PID m_PID;
    float m_feedForward = 0;
    int32_t m_softLimitForwardPosition; // (step)
    int32_t m_softLimitReversePosition; // (step)
    int16_t m_calibrationDutyCycle;     // (1/32768)
    int32_t m_calibrationPosition;      // (step)
    bool m_debugTelemetryEnabled;       // true: SMoCo will continuously send debug
                                        // telemetry

    // true: received Position Calibrated message and calibratePosition has not been called since
    bool m_calibrated;

    bool m_limitSwitchForward; // true: limit switch B depressed
    bool m_limitSwitchReverse; // true: limit switch A depressed
    bool m_softLimitForward;   // true: soft limit B reached
    bool m_softLimitReverse;   // true: soft limit A reached

    uint32_t m_lastPingReply;      // (ms) time last ping reply was received
    uint64_t m_echoRequestPayload; // payload of most recent sent Echo Request
    uint64_t m_echoResponse;       // payload of most recent received Echo Reply
    // (ms) ping round trip time or UINT16_MAX after ping is called when millis() > m_echoRequestAt + m_pingTimeout
    uint64_t m_pingTime = UINT16_MAX;

    int32_t m_encoderZeroPosition = 0;
    float m_stepsPerDegree = 1;

    uint8_t m_commandErrorID; // ID of the last sent command SMoCo considered invalid

    SmocoDebugTelemetry m_debugTelemetry;

    // Called when the Smoco requests a parameter

    bool sendRampRate();
    bool sendPI();
    bool sendD();
    bool sendIgnoreLimit();
    bool sendSoftLimitPosition();
    bool sendDutyCycleRange();

public:
    // Getters

    int32_t getPosition() const { return m_position; } // (step)
    int16_t getVelocity() const { return m_velocity; } // (step/s)
    uint8_t getCurrent() const { return m_current; }   // (A)

    bool getIgnoreLimitForward() const { return m_ignoreLimitForward; } // true: ignore limit switch B actuation
    bool getIgnoreLimitReverse() const { return m_ignoreLimitReverse; } // true: ignore limit switch A actuation
    int16_t getDutyCycle() const { return m_dutyCycle; }                // (1/32768)
    DutyCycleRange getDutyCycleRange() const { return m_dutyCycleRange; }
    int32_t getTargetPosition() const { return m_targetPosition; } // (step)
    float getTargetVelocity() const { return m_targetVelocity; }   // (step/s)
    float getTargetCurrent() const { return m_targetCurrent; }     // (A)
    float getRampRate() const { return m_rampRate; }               // (1/s)
    PID getPID() const { return m_PID; }
    float getFeedForward() const { return m_feedForward; }
    int32_t getSoftLimitForwardPosition() const { return m_softLimitForwardPosition; } // (step)
    int32_t getSoftLimitReversePosition() const { return m_softLimitReversePosition; } // (step)
    int16_t getCalibrationDutyCycle() const { return m_calibrationDutyCycle; }         // (1/32768)
    int32_t getCalibrationPosition() const { return m_calibrationPosition; }           // (step)
    // true: SMoCo will continuously send debug telemetry
    bool getDebugTelemetryEnabled() const { return m_debugTelemetryEnabled; }

    // true: received Position Calibrated message and calibratePosition has not been called since
    bool getCalibrated() const { return m_calibrated; }

    bool getLimitSwitchForward() const { return m_limitSwitchForward; } // true: limit switch B depressed
    bool getLimitSwitchReverse() const { return m_limitSwitchReverse; } // true: limit switch A depressed
    bool getSoftLimitForward() const { return m_softLimitForward; }     // true: soft limit B reached
    bool getSoftLimitReverse() const { return m_softLimitReverse; }     // true: soft limit A reached
    // true: given position is between the soft limits
    bool isPositionWithinLimits(int32_t position) const {
        return position > m_softLimitReversePosition && position < m_softLimitForwardPosition;
    }

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
    bool driveTargetPosition(int32_t targetPosition, float feedForward);
    bool driveTargetVelocity(float targetVelocity, float feedForward);
    bool driveTargetCurrent(float targetCurrent, float feedForward);
    bool setDutyCycleRange(int16_t minForward, int16_t maxForward, int16_t minReverse, int16_t maxReverse);
    bool setDutyCycleRange(DutyCycleRange range) {
        return setDutyCycleRange(range.minForward, range.maxForward, range.minReverse, range.maxReverse);
    }
    bool setRampRate(float rampRate);
    bool setPID(float P, float I, float D);
    bool setPID(PID pid) { return setPID(pid.P, pid.I, pid.D); }
    void configFeedForward(float feedForward) { m_feedForward = feedForward; } // Constant added to PID
    bool setIgnoreLimit(bool forward, bool reverse); // NOTE: effective only after next driveXXXX() call
    bool setSoftLimitPosition(int32_t forward, int32_t reverse);
    bool calibratePosition(int16_t dutyCycle, int32_t limitSwitchPosition);
    bool debugTelemetry(bool enable);
    bool stopAndReset();
    bool echoRequest(uint64_t payload);
    void sync(CANMessage msg); // Update telemetry variables from the received
                               // message and send a response if necessary
    bool ping();

    // Angle based functions

    // configure conversion to apply to target position so that you can work with angles instead of raw steps
    void configAngleConversion(int32_t encoderZeroPosition, float stepsPerDegree) {
        m_encoderZeroPosition = encoderZeroPosition;
        m_stepsPerDegree = stepsPerDegree;
    }
    float stepsToDegrees(int32_t steps) const { return (steps - m_encoderZeroPosition) / m_stepsPerDegree; }
    int32_t degreesToSteps(float degrees) const { return (degrees * m_stepsPerDegree) + m_encoderZeroPosition; }
    float degreesToStepsFloat(float degrees) const { return (degrees * m_stepsPerDegree) + m_encoderZeroPosition; }
    // (deg, i.e. units defined in configAngleConversion)
    float getAngle() const { return stepsToDegrees(m_position); }
    // (deg/s, i.e. units defined in configAngleConversion)
    float getAngularVelocity() const { return stepsToDegrees(m_velocity); }
    // (deg, i.e. units defined in configAngleConversion)
    float getTargetAngle() const { return stepsToDegrees(m_targetPosition); }
    // (deg/s, i.e. units defined in configAngleConversion)
    float getTargetAngularVelocity() const { return stepsToDegrees(m_targetVelocity); }
    // (deg, i.e. units defined in configAngleConversion)
    float getSoftLimitForwardAngle() const { return stepsToDegrees(m_softLimitForwardPosition); }
    // (deg, i.e. units defined in configAngleConversion)
    float getSoftLimitReverseAngle() const { return stepsToDegrees(m_softLimitReversePosition); }
    // isPositionWithinLimits but with angle conversion applied
    bool isAngleWithinLimits(float angle) const { return isPositionWithinLimits(degreesToSteps(angle)); }
    // driveTargetPosition but with angle conversion applied
    bool driveTargetAngle(float targetAngle, float feedForward) {
        return driveTargetPosition(degreesToSteps(targetAngle), feedForward);
    }
    // driveTargetVelocity but with angle conversion applied
    bool driveTargetAngularVelocity(float targetVelocity, float feedForward) {
        return driveTargetVelocity(degreesToStepsFloat(targetVelocity), feedForward);
    }
    // setSoftLimitPosition but with angle conversion applied
    bool setSoftLimitAngle(float forward, float reverse) {
        return setSoftLimitPosition(degreesToSteps(forward), degreesToSteps(reverse));
    }
    // calibratePosition but with angle conversion applied
    bool calibrateAngle(int16_t dutyCycle, float limitSwitchAngle) {
        return calibratePosition(dutyCycle, degreesToSteps(limitSwitchAngle));
    }

private:
    bool sendCommand(uint32_t mid, const SmocoCANMessage &&data) {
        return canBus->tryToSend(
            CANMessage{.id = mid, .len = (uint8_t)SMOCO_WIDTH[mid], .data64 = *((uint64_t *)&data)});
    }
};
#endif
