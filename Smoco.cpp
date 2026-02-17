#include "Smoco.h"

Smoco::Smoco(ACAN_T4 *canBus, uint8_t canID) : canBus(canBus), canID(canID) {}

void Smoco::sync(CANMessage message) {
    if (message.id >> 4 == 0x7F) {
        // Update debug telemetry.
        if ((message.id & 0xF) <= 8) ((uint64_t *)&m_debugTelemetry)[message.id & 0xF] = message.data64;
        return;
    }
    if (message.id >> 4 != canID) return; // Not this SMoCo.

    SmocoCANMessage *smocoCANMessage = (SmocoCANMessage *)message.data;
    if (message.rtr) {
        // Send missing parameter.
        switch (message.id & 0xF) {
        case SMOCO_MESSAGE_ID_SMOOTHING:
            sendRampRate();
            break;
        case SMOCO_MESSAGE_ID_PID:
            sendPID();
            break;
        case SMOCO_MESSAGE_ID_SOFT_LIMIT:
            sendSoftLimitPosition();
            break;
        }
    } else {
        switch (message.id & 0xF) {
        case SMOCO_MESSAGE_ID_POSITION:
            m_position = smocoCANMessage->position.position;
            m_velocity = smocoCANMessage->position.velocity;
            m_current = smocoCANMessage->position.current;
            // Get the bit at each position and convert it to a bool.
            m_limitSwitchA = !!(smocoCANMessage->position.flags & (1 << 7));
            m_limitSwitchB = !!(smocoCANMessage->position.flags & (1 << 6));
            m_softLimitA = !!(smocoCANMessage->position.flags & (1 << 5));
            m_softLimitB = !!(smocoCANMessage->position.flags & (1 << 4));
            break;
        case SMOCO_MESSAGE_ID_POSITION_CALIBRATED:
            m_calibrated = true;
            break;
        case SMOCO_MESSAGE_ID_ERROR:
            m_commandErrorID = smocoCANMessage->commandError.commandID;
            break;
        case SMOCO_MESSAGE_ID_ECHO_REPLY:
            m_echoResponse = smocoCANMessage->echoReply.payload;
            m_pingTime = millis() - m_echoResponse;
            m_lastPingReply = millis();
            break;
        }
    }
}

bool Smoco::driveOpenLoop(int16_t dutyCycle) {
    bool ignoreLimit = false;
    if (dutyCycle > 0) {
        ignoreLimit = m_ignoreForwardLimit;
    } else if (dutyCycle < 0) {
        ignoreLimit = m_ignoreReverseLimit;
    }

    m_dutyCycle = dutyCycle;
    return canBus->tryToSend(
        CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_TARGET,
                   .len = 3,
                   .data64 = SmocoCANMessage{.target = {.sid = (uint8_t)(SMOCO_MESSAGE_SID_OPEN_LOOP | ignoreLimit),
                                                        .openLoop = {.dutyCycle = m_dutyCycle}}}
                                 .data64});
}

bool Smoco::driveTargetPosition(int32_t targetPosition, float errorGain) {
    bool ignoreLimit = false;
    if (targetPosition > m_targetPosition) {
        ignoreLimit = m_ignoreForwardLimit;
    } else if (targetPosition < m_targetPosition) {
        ignoreLimit = m_ignoreReverseLimit;
    }

    m_targetPosition = targetPosition;
    m_errorGain = errorGain;

    if (m_targetPosition <= m_softLimitAPosition) {
        m_targetPosition = m_softLimitAPosition;
    }
    if (m_targetPosition >= m_softLimitBPosition) {
        m_targetPosition = m_softLimitBPosition;
    }

    return canBus->tryToSend(
        CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_TARGET,
                   .len = 7,
                   .data64 = SmocoCANMessage{.target = {.sid = (uint8_t)(SMOCO_MESSAGE_SID_POSITION | ignoreLimit),
                                                        .targetPosition = {.errorGain = (uint16_t)(m_errorGain * 1024),
                                                                           .position = m_targetPosition}}}
                                 .data64});
}

bool Smoco::driveTargetVelocity(int32_t targetVelocity, float errorGain) {
    bool ignoreLimit = false;
    if (targetVelocity > 0) {
        ignoreLimit = m_ignoreForwardLimit;
    } else if (targetVelocity < 0) {
        ignoreLimit = m_ignoreReverseLimit;
    }

    m_targetVelocity = targetVelocity;
    m_errorGain = errorGain;
    return canBus->tryToSend(
        CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_TARGET,
                   .len = 7,
                   .data64 = SmocoCANMessage{.target = {.sid = (uint8_t)(SMOCO_MESSAGE_SID_VELOCITY | ignoreLimit),
                                                        .targetVelocity = {.errorGain = (uint16_t)(m_errorGain * 1024),
                                                                           .velocity = m_targetVelocity}}}
                                 .data64});
}

bool Smoco::driveTargetCurrent(float targetCurrent, float errorGain) {
    bool ignoreLimit = false;
    if (targetCurrent > 0) {
        ignoreLimit = m_ignoreForwardLimit;
    } else if (targetCurrent < 0) {
        ignoreLimit = m_ignoreReverseLimit;
    }

    m_targetCurrent = targetCurrent;
    m_errorGain = errorGain;
    return canBus->tryToSend(CANMessage{
        .id = (canID << 4) | SMOCO_MESSAGE_ID_TARGET,
        .len = 5,
        .data64 = SmocoCANMessage{
            .target = {
                .sid = (uint8_t)(SMOCO_MESSAGE_SID_CURRENT | ignoreLimit),
                .targetCurrent = {
                    .errorGain = (uint16_t)(m_targetCurrent * 1024),
                    .current = (int16_t)(targetCurrent * 8) // TODO: Update scaling factor when current is supported
                }}}.data64});
}

void Smoco::configIgnoreLimits(bool forward, bool reverse) {
    m_ignoreForwardLimit = forward;
    m_ignoreReverseLimit = reverse;
}

bool Smoco::setRampRate(double rampRate) {
    m_rampRate = rampRate;
    return sendRampRate();
}

bool Smoco::sendRampRate() {
    return canBus->tryToSend(CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_SMOOTHING,
                                        .len = 8,
                                        .data64 = SmocoCANMessage{.setRampRate = {.rampRate = m_rampRate}}.data64});
}

bool Smoco::setPID(float P, float I, float D) {
    m_PID.P = P;
    m_PID.I = I;
    m_PID.D = D;
    return sendPID();
}

bool Smoco::sendPID() {
    return canBus->tryToSend(CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_PID,
                                        .len = 6,
                                        .data64 = SmocoCANMessage{.setPID = {.p = (uint16_t)(m_PID.P * 256),
                                                                             .i = (uint16_t)(m_PID.I * 256),
                                                                             .d = (uint16_t)(m_PID.D * 256)}}
                                                      .data64});
}

bool Smoco::setSoftLimitPosition(int32_t positionA, int32_t positionB) {
    m_softLimitAPosition = positionA;
    m_softLimitBPosition = positionB;
    return sendSoftLimitPosition();
}

bool Smoco::sendSoftLimitPosition() {
    return canBus->tryToSend(
        CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_SOFT_LIMIT,
                   .len = 8,
                   .data64 = SmocoCANMessage{.setSoftLimitPosition = {.aPosition = m_softLimitAPosition,
                                                                      .bPosition = m_softLimitBPosition}}
                                 .data64});
}

bool Smoco::calibratePosition(int16_t dutyCycle, int32_t limitSwitchPosition) {
    m_calibrationDutyCycle = dutyCycle;
    m_calibrationPosition = limitSwitchPosition;
    m_calibrated = false;
    return canBus->tryToSend(
        CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_CALIBRATE,
                   .len = 6,
                   .data64 = SmocoCANMessage{.startPositionCalibration = {.dutyCycle = m_calibrationDutyCycle,
                                                                          .limitSwitchPosition = m_calibrationPosition}}
                                 .data64});
}

bool Smoco::debugTelemetry(bool enable) {
    m_debugTelemetryEnabled = enable;
    return canBus->tryToSend(
        CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_DEBUG,
                   .len = 1,
                   .data64 = SmocoCANMessage{.debugTelemetry = {.enable = m_debugTelemetryEnabled}}.data64});
}

bool Smoco::stopAndReset() {
    return canBus->tryToSend(CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_STOP, .len = 0});
}

bool Smoco::echoRequest(uint64_t payload) {
    m_echoRequestPayload = payload;
    return canBus->tryToSend(
        CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_ECHO_REQUEST,
                   .len = 8,
                   .data64 = SmocoCANMessage{.echoRequestPayload = {.payload = m_echoRequestPayload}}.data64});
}

bool Smoco::ping() {
    if (millis() > m_lastPingReply + pingTimeout) {
        m_pingTime = pingTimeout;
    };
    return echoRequest(millis());
}
