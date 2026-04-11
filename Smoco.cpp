#include "Smoco.h"

Smoco::Smoco(ACAN_T4 *canBus, uint8_t canID) : canBus(canBus), canID(canID) {}

void Smoco::sync(CANMessage message) {
    if (message.id >> SMOCO_WIDTH_DEBUG == SMOCO_ID_DEBUG >> SMOCO_WIDTH_DEBUG) {
        // Update debug telemetry.
        uint8_t debugIndex = message.id & ((1 << SMOCO_WIDTH_DEBUG) - 1);
        if (debugIndex <= 8) {
            ((uint64_t *)&m_debugTelemetry)[debugIndex] = message.data64;
        }
        return;
    }
    if (message.id >> SMOCO_WIDTH_MID != canID) return; // Not this SMoCo.

    SmocoCANMessage *messageData = (SmocoCANMessage *)message.data;
    if (message.rtr) {
        // Send missing parameter.
        switch (message.id & ((1 << SMOCO_WIDTH_MID) - 1)) {
        case SMOCO_MID_RAMP_RATE:
            sendRampRate();
            break;
        case SMOCO_MID_PI:
            sendPI();
            break;
        case SMOCO_MID_D:
            sendD();
            break;
        case SMOCO_MID_DUTY_CYCLE_RANGE:
            sendDutyCycleRange();
            break;
        case SMOCO_MID_IGNORE_LIMIT:
            sendIgnoreLimit();
            break;
        case SMOCO_MID_SOFT_LIMIT:
            sendSoftLimitPosition();
            break;
        }
    } else {
        switch (message.id & ((1 << SMOCO_WIDTH_MID) - 1)) {
        case SMOCO_MID_POSITION:
            m_position = messageData->SMOCO_MID_POSITION_.position;
            m_velocity = messageData->SMOCO_MID_POSITION_.velocity;
            m_current = messageData->SMOCO_MID_POSITION_.current;
            // Get the bit at each position and convert it to a bool.
            m_limitSwitchReverse = !!(messageData->SMOCO_MID_POSITION_.flags & (1 << 7));
            m_limitSwitchForward = !!(messageData->SMOCO_MID_POSITION_.flags & (1 << 6));
            m_softLimitReverse = !!(messageData->SMOCO_MID_POSITION_.flags & (1 << 5));
            m_softLimitForward = !!(messageData->SMOCO_MID_POSITION_.flags & (1 << 4));
            break;
        case SMOCO_MID_POSITION_CALIBRATED:
            m_calibrated = true;
            break;
        case SMOCO_MID_ERROR:
            m_commandErrorID = messageData->SMOCO_MID_ERROR_.commandID;
            break;
        case SMOCO_MID_ECHO_REPLY:
            m_echoResponse = messageData->SMOCO_MID_ECHO_REPLY_.payload;
            m_pingTime = millis() - m_echoResponse;
            m_lastPingReply = millis();
            break;
        }
    }
}

bool Smoco::driveOpenLoop(int16_t dutyCycle) {
    m_dutyCycle = dutyCycle;
    return sendCommand(SMOCO_MID_OPEN_LOOP, {.SMOCO_MID_OPEN_LOOP_ = {.dutyCycle = m_dutyCycle}});
}

bool Smoco::driveTargetPosition(int32_t targetPosition, float feedForward) {
    m_targetPosition = targetPosition;
    m_feedForward = feedForward;

    if (m_targetPosition <= m_softLimitReversePosition) {
        m_targetPosition = m_softLimitReversePosition;
    }
    if (m_targetPosition >= m_softLimitForwardPosition) {
        m_targetPosition = m_softLimitForwardPosition;
    }

    return sendCommand(SMOCO_MID_TARGET_POSITION,
                       {.SMOCO_MID_TARGET_POSITION_ = {.feedForward = (int16_t)(m_feedForward * INT16_MAX),
                                                       .position = targetPosition}});
}

bool Smoco::driveTargetVelocity(float targetVelocity, float feedForward) {
    m_targetVelocity = targetVelocity;
    m_feedForward = feedForward;
    return sendCommand(SMOCO_MID_TARGET_VELOCITY,
                       {.SMOCO_MID_TARGET_VELOCITY_ = {.feedForward = (int16_t)(m_feedForward * INT16_MAX),
                                                       .velocity = m_targetVelocity}});
}

bool Smoco::driveTargetCurrent(float targetCurrent, float feedForward) {
    m_targetCurrent = targetCurrent;
    m_feedForward = feedForward;
    return sendCommand(SMOCO_MID_TARGET_CURRENT,
                       {.SMOCO_MID_TARGET_CURRENT_ = {.feedForward = (int16_t)(m_feedForward * INT16_MAX),
                                                      .current = (int16_t)(m_targetCurrent * INT16_MAX)}});
}

bool Smoco::setDutyCycleRange(int16_t minForward, int16_t maxForward, int16_t minReverse, int16_t maxReverse) {
    m_dutyCycleRange.minForward = minForward;
    m_dutyCycleRange.maxForward = maxForward;
    m_dutyCycleRange.minReverse = minReverse;
    m_dutyCycleRange.maxReverse = maxReverse;
    return sendDutyCycleRange();
}

bool Smoco::sendDutyCycleRange() {
    return sendCommand(SMOCO_MID_DUTY_CYCLE_RANGE,
                       {.SMOCO_MID_DUTY_CYCLE_RANGE_ = {.fwdMax = m_dutyCycleRange.maxForward,
                                                        .fwdMin = m_dutyCycleRange.minForward,
                                                        .revMin = m_dutyCycleRange.minReverse,
                                                        .revMax = m_dutyCycleRange.maxReverse}});
}

bool Smoco::setRampRate(float rampRate) {
    m_rampRate = rampRate;
    return sendRampRate();
}

bool Smoco::sendRampRate() {
    return sendCommand(SMOCO_MID_RAMP_RATE, {.SMOCO_MID_RAMP_RATE_ = {.rampRate = m_rampRate}});
}

bool Smoco::setPID(float P, float I, float D) {
    m_PID.P = P;
    m_PID.I = I;
    m_PID.D = D;
    return sendPI() && sendD();
}

bool Smoco::sendPI() {
    return sendCommand(SMOCO_MID_PI, {.SMOCO_MID_PI_ = {
                                          .p = m_PID.P,
                                          .i = m_PID.I,
                                      }});
}
bool Smoco::sendD() { return sendCommand(SMOCO_MID_D, {.SMOCO_MID_D_ = {.d = m_PID.D}}); }

bool Smoco::setIgnoreLimit(bool forward, bool reverse) {
    m_ignoreLimitForward = forward;
    m_ignoreLimitReverse = reverse;
    return sendIgnoreLimit();
}

bool Smoco::sendIgnoreLimit() {
    return sendCommand(
        SMOCO_MID_IGNORE_LIMIT,
        {.SMOCO_MID_IGNORE_LIMIT_ = {// limit A is reverse, limit B is forward
                                     .ab = (uint8_t)((m_ignoreLimitForward << 1) | (m_ignoreLimitReverse << 0))}});
}

bool Smoco::setSoftLimitPosition(int32_t positionA, int32_t positionB) {
    m_softLimitReversePosition = positionA;
    m_softLimitForwardPosition = positionB;
    return sendSoftLimitPosition();
}

bool Smoco::sendSoftLimitPosition() {
    return sendCommand(SMOCO_MID_SOFT_LIMIT, {.SMOCO_MID_SOFT_LIMIT_ = {
                                                  .aPosition = m_softLimitReversePosition,
                                                  .bPosition = m_softLimitForwardPosition,
                                              }});
}

bool Smoco::calibratePosition(int16_t dutyCycle, int32_t limitSwitchPosition) {
    m_calibrationDutyCycle = dutyCycle;
    m_calibrationPosition = limitSwitchPosition;
    m_calibrated = false;
    return sendCommand(SMOCO_MID_CALIBRATE, {.SMOCO_MID_CALIBRATE_ = {.dutyCycle = m_calibrationDutyCycle,
                                                                      .limitSwitchPosition = m_calibrationPosition}});
}

bool Smoco::debugTelemetry(bool enable) {
    m_debugTelemetryEnabled = enable;
    return sendCommand(SMOCO_MID_DEBUG, {.SMOCO_MID_DEBUG_ = {.enable = m_debugTelemetryEnabled}});
}

bool Smoco::stopAndReset() { return sendCommand(SMOCO_MID_STOP, {0}); }

bool Smoco::echoRequest(uint64_t payload) {
    m_echoRequestPayload = payload;
    return sendCommand(SMOCO_MID_ECHO_REQUEST, {.SMOCO_MID_ECHO_REQUEST_ = {.payload = m_echoRequestPayload}});
}

bool Smoco::ping() {
    if (millis() > m_lastPingReply + pingTimeout) {
        m_pingTime = pingTimeout;
    };
    return echoRequest(millis());
}
