#include "Smoco.h"

Smoco::Smoco(ACAN_T4 *canBus, uint8_t canID)
    : m_canBus(canBus), m_canID(canID) {}

void Smoco::sync(CANMessage message) {
  if (message.id >> 4 == 0x7F) {
    // Update debug telemetry.
    if ((message.id & 0xF) <= 8)
      ((uint64_t *)&m_debugTelemetry)[message.id & 0xF] = message.data64;
    return;
  }
  if (message.id >> 4 != m_canID)
    return; // Not this SMoCo.

  SmocoCANMessage *smocoCANMessage = (SmocoCANMessage *)message.data;
  if (message.rtr) {
    // Send missing parameter.
    switch (message.id & 0xF) {
    case MESSAGE_ID_SMOOTHING:
      sendLowPassSmoothingFactor();
      break;
    case MESSAGE_ID_PID:
      sendPID();
      break;
    case MESSAGE_ID_SOFT_LIMIT:
      sendSoftLimitPosition();
      break;
    }
  } else {
    switch (message.id & 0xF) {
    case MESSAGE_ID_POSITION:
      m_position = smocoCANMessage->position.position;
      m_velocity = smocoCANMessage->position.velocity;
      m_current = smocoCANMessage->position.current;
      // Get the bit at each position and convert it to a bool.
      m_limitSwitchA = !!(smocoCANMessage->position.flags & (1 << 7));
      m_limitSwitchB = !!(smocoCANMessage->position.flags & (1 << 6));
      m_softLimitA = !!(smocoCANMessage->position.flags & (1 << 5));
      m_softLimitB = !!(smocoCANMessage->position.flags & (1 << 4));
      break;
    case MESSAGE_ID_POSITION_CALIBRATED:
      m_calibrated = true;
      break;
    case MESSAGE_ID_ERROR:
      m_commandErrorID = smocoCANMessage->commandError.commandID;
      break;
    case MESSAGE_ID_ECHO_REPLY:
      m_echoResponse = smocoCANMessage->echoReply.payload;
      m_pingTime = millis() - m_echoResponse;
      m_lastEchoResponseTime = millis();
      break;
    }
  }
}

bool Smoco::driveOpenLoop(int16_t dutyCycle, bool ignoreLimit) {
  m_dutyCycle = dutyCycle;
  m_ignoreLimit = !!ignoreLimit;
  return m_canBus->tryToSend(CANMessage{
      .id = (m_canID << 4) | MESSAGE_ID_TARGET,
      .len = 3,
      .data64 =
          SmocoCANMessage{
              .target = {.sid = MESSAGE_SID_OPEN_LOOP + (uint8_t)m_ignoreLimit,
                         .openLoop = {.dutyCycle = m_dutyCycle}}}
              .data64});
}

bool Smoco::driveTargetPosition(int32_t targetPosition, uint16_t errorGain,
                                bool ignoreLimit) {
  m_targetPosition = targetPosition;
  m_errorGain = errorGain;
  m_ignoreLimit = !!ignoreLimit;
  return m_canBus->tryToSend(CANMessage{
      .id = (m_canID << 4) | MESSAGE_ID_TARGET,
      .len = 7,
      .data64 =
          SmocoCANMessage{
              .target = {.sid = MESSAGE_SID_POSITION + (uint8_t)m_ignoreLimit,
                         .targetPosition = {.errorGain = m_errorGain,
                                            .position = m_targetPosition}}}
              .data64});
}

bool Smoco::driveTargetVelocity(int32_t targetVelocity, uint16_t errorGain,
                                bool ignoreLimit) {
  m_targetVelocity = targetVelocity;
  m_errorGain = errorGain;
  m_ignoreLimit = !!ignoreLimit;
  return m_canBus->tryToSend(CANMessage{
      .id = (m_canID << 4) | MESSAGE_ID_TARGET,
      .len = 7,
      .data64 =
          SmocoCANMessage{
              .target = {.sid = MESSAGE_SID_VELOCITY + (uint8_t)m_ignoreLimit,
                         .targetVelocity = {.errorGain = m_errorGain,
                                            .velocity = m_targetVelocity}}}
              .data64});
}

bool Smoco::driveTargetCurrent(int16_t targetCurrent, uint16_t errorGain,
                               bool ignoreLimit) {
  m_targetCurrent = targetCurrent;
  m_errorGain = errorGain;
  m_ignoreLimit = !!ignoreLimit;
  return m_canBus->tryToSend(CANMessage{
      .id = (m_canID << 4) | MESSAGE_ID_TARGET,
      .len = 5,
      .data64 =
          SmocoCANMessage{
              .target = {.sid = MESSAGE_SID_CURRENT + (uint8_t)m_ignoreLimit,
                         .targetCurrent = {.errorGain = m_targetCurrent,
                                           .current = targetCurrent}}}
              .data64});
}

bool Smoco::setLowPassSmoothingFactor(uint16_t alpha) {
  m_lowPassSmoothingFactor = alpha;
  return sendLowPassSmoothingFactor();
}

bool Smoco::sendLowPassSmoothingFactor() {
  return m_canBus->tryToSend(CANMessage{
      .id = (m_canID << 4) | MESSAGE_ID_SMOOTHING,
      .len = 2,
      .data64 =
          SmocoCANMessage{
              .setLowPassSmoothingFactor = {.alpha = m_lowPassSmoothingFactor}}
              .data64});
}

bool Smoco::setPID(uint16_t P, uint16_t I, uint16_t D) {
  m_PID[0] = P;
  m_PID[1] = I;
  m_PID[2] = D;
  return sendPID();
}

bool Smoco::sendPID() {
  return m_canBus->tryToSend(CANMessage{
      .id = (m_canID << 4) | MESSAGE_ID_PID,
      .len = 6,
      .data64 =
          SmocoCANMessage{
              .setPID = {.p = m_PID[0], .i = m_PID[1], .d = m_PID[2]}}
              .data64});
}

bool Smoco::setSoftLimitPosition(int32_t positionA, int32_t positionB) {
  m_softLimitAPosition = positionA;
  m_softLimitBPosition = positionB;
  return sendSoftLimitPosition();
}

bool Smoco::sendSoftLimitPosition() {
  return m_canBus->tryToSend(CANMessage{
      .id = (m_canID << 4) | MESSAGE_ID_SOFT_LIMIT,
      .len = 8,
      .data64 =
          SmocoCANMessage{
              .setSoftLimitPosition = {.aPosition = m_softLimitAPosition,
                                       .bPosition = m_softLimitBPosition}}
              .data64});
}

bool Smoco::calibratePosition(int16_t dutyCycle, int32_t limitSwitchPosition) {
  m_calibrationDutyCycle = dutyCycle;
  m_calibrationPosition = limitSwitchPosition;
  m_calibrated = false;
  return m_canBus->tryToSend(CANMessage{
      .id = (m_canID << 4) | MESSAGE_ID_CALIBRATE,
      .len = 6,
      .data64 =
          SmocoCANMessage{
              .startPositionCalibration = {.dutyCycle = m_calibrationDutyCycle,
                                           .limitSwitchPosition =
                                               m_calibrationPosition}}
              .data64});
}

bool Smoco::debugTelemetry(uint8_t enable) {
  m_debugTelemetryEnabled = enable;
  return m_canBus->tryToSend(CANMessage{
      .id = (m_canID << 4) | MESSAGE_ID_DEBUG,
      .len = 1,
      .data64 =
          SmocoCANMessage{.debugTelemetry = {.enable = m_debugTelemetryEnabled}}
              .data64});
}

bool Smoco::stopAndReset() {
  return m_canBus->tryToSend(
      CANMessage{.id = (m_canID << 4) | MESSAGE_ID_STOP, .len = 0});
}

bool Smoco::echoRequest(uint64_t payload) {
  if (millis() - m_lastEchoResponseTime > m_pingTimeout){
    // Very big numer indicating smoco dropped
    m_pingTime = UINT16_MAX;
  } 
  m_echoRequestPayload = payload;
  return m_canBus->tryToSend(CANMessage{
      .id = (m_canID << 4) | MESSAGE_ID_ECHO_REQUEST,
      .len = 8,
      .data64 =
          SmocoCANMessage{.echoRequestPayload = {.payload = m_echoRequestPayload}}
              .data64});
}

bool Smoco::ping() {
  m_pinging = true;
  return echoRequest(millis());
}

