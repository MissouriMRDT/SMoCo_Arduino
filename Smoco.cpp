#include "Smoco.h"

Smoco::Smoco(ACAN_T4 *canBus, uint8_t canID){
        m_canBus = canBus;
        m_canID = canID;
    }

void Smoco::setCanID(uint8_t canID) {
    m_canID = canID;
}

enum MessageID {
    MESSAGE_ID_POSITION = 0x0,
    MESSAGE_ID_POSITION_CALIBRATED = 0x1,
    MESSAGE_ID_TARGET = 0x2,
    MESSAGE_ID_SMOOTHING = 0x3,
    MESSAGE_ID_PID = 0x4,
    MESSAGE_ID_SOFT_LIMIT = 0x5,
    MESSAGE_ID_CALIBRATE = 0x6,
    MESSAGE_ID_STOP = 0xC,
    MESSAGE_ID_ERROR = 0xD,
    MESSAGE_ID_ECHO_REQUEST = 0xE,
    MESSAGE_ID_ECHO_REPLY = 0xF,
};

enum TargetID {
    TARGET_ID_OPEN_LOOP = 0x0,
    TARGET_ID_POSITION = 0x2,
    TARGET_ID_VELOCITY = 0x4,
    TARGET_ID_CURRENT = 0x6,
};

void Smoco::readIncomingMessage(CANMessage msg){
        switch( (uint8_t)((msg.id & 0xF)) ){
        case MESSAGE_ID_POSITION: {
            // position telemetry (angle, angular velocity, current, and limits)
            // assigns telemetry values to member variables
            m_angle = 0;
            m_angle = msg.data[0] | (msg.data[1] << 8) | (msg.data[2] << 16) | (msg.data[3] << 24);

            m_angularVelocity = msg.data[4] | (msg.data[5] << 8);

            m_current = msg.data[6];

            m_limSwitchA = msg.data[7] & 0b1000;
            m_limSwitchB = msg.data[7] & 0b0100;
            m_softLimA = msg.data[7] & 0b0010;
            m_softLimB = msg.data[7] & 0b0001;
            break;
        }
        case MESSAGE_ID_POSITION_CALIBRATED: {
            // position calbirated return status
            // calibration finished
            m_isCalibrated = true;
            break;
        }
        case MESSAGE_ID_ERROR: {
            // command error; returns ID of failed can msg
            m_commandErrorID = msg.data[0];
            break;
        }
        case MESSAGE_ID_ECHO_REPLY: {
            // echo reply
            if(m_isPinging){
                m_pingTime = 0;
                m_pingTime = millis() - (*(uint64_t)msg.data);
                m_isPinging = false;
            }
            else{
                m_echoData = (*(uint4_t)msg.data);
            }
            break;
        }
        }
    }

void Smoco::openLoopDrive(int16_t dutyCycle, bool ignoreLimit) {
    CANMessage msg;
    msg.id = (m_canID << 4) | MESSAGE_ID_TARGET;
    m_ignoreLimit = ignoreLimit;
    msg.data[0] = m_ignoreLimit ? TARGET_ID_OPEN_LOOP + 1 : TARGET_ID_OPEN_LOOP;

    msg.data[1] = (uint8_t)(dutyCycle & 0xFF);
    msg.data[2] = (uint8_t)((dutyCycle >> 8) & 0xFF);

    msg.len = 3;
    const bool ok = m_canBus->tryToSend(msg);
}

void Smoco::setJointAngle(uint32_t targetPosition, uint16_t errorGain, bool ignoreLimit) {
    CANMessage msg;
    msg.id = (m_canID << 4) | MESSAGE_ID_TARGET;
    m_ignoreLimit = ignoreLimit;
    msg.data[0] = m_ignoreLimit? TARGET_ID_POSITION + 1: TARGET_ID_OPEN_LOOP;

    msg.data[1] = (uint8_t)(errorGain & 0xFF);
    msg.data[2] = (uint8_t)((errorGain >> 8) & 0xFF);

    msg.data[3] = (uint8_t)(targetPosition & 0xFF);
    msg.data[4] = (uint8_t)((targetPosition >> 8) & 0xFF);
    msg.data[5] = (uint8_t)((targetPosition >> 16) & 0xFF);
    msg.data[6] = (uint8_t)((targetPosition >> 24) & 0xFF);

    msg.len = 7;
    const bool ok = m_canBus->tryToSend(msg);
}

void Smoco::setJointVelocity(uint32_t targetVelocity, uint16_t errorGain, bool ignoreLimit = false) {
    CANMessage msg;
    msg.id = (m_canID << 4) | MESSAGE_ID_TARGET;
    m_ignoreLimit = ignoreLimit;
    msg.data[0] = m_ignoreLimit ? TARGET_ID_VELOCITY + 1 : TARGET_ID_VELOCITY;

    msg.data[1] = (uint8_t)(errorGain & 0xFF);
    msg.data[2] = (uint8_t)((errorGain >> 8) & 0xFF);
    
    msg.data[3] = (uint8_t)(targetVelocity & 0xFF);
    msg.data[4] = (uint8_t)((targetVelocity >> 8) & 0xFF);
    msg.data[5] = (uint8_t)((targetVelocity >> 16) & 0xFF);
    msg.data[6] = (uint8_t)((targetVelocity >> 24) & 0xFF);

    msg.len = 7;
    const bool ok = m_canBus->tryToSend(msg);
}

void Smoco::setJointCurrent(int16_t targetCurrent, uint16_t errorGain, bool ignoreLimit = false) {
    CANMessage msg;
    msg.id = (m_canID << 4) | MESSAGE_ID_TARGET;
    m_ignoreLimit = ignoreLimit;
    msg.data[0] = m_ignoreLimit ? TARGET_ID_CURRENT + 1 : TARGET_ID_CURRENT;

    msg.data[1] = (uint8_t)(errorGain & 0xFF);
    msg.data[2] = (uint8_t)((errorGain >> 8) & 0xFF);

    msg.data[3] = (uint8_t)(targetCurrent & 0xFF);
    msg.data[4] = (uint8_t)((targetCurrent >> 8) & 0xFF);

    msg.len = 5;
    const bool ok = m_canBus->tryToSend(msg);
}

void Smoco::setLowPassSmoothingFactor(uint16_t alpha) {
    CANMessage msg;
    msg.id = (m_canID << 4) | MESSAGE_ID_SMOOTHING;

    msg.data[0] = (uint8_t)(alpha & 0xFF);
    msg.data[1] = (uint8_t)((alpha >> 8) & 0xFF);

    msg.len = 2;
    const bool ok = m_canBus->tryToSend(msg);
}

void Smoco::setPID(uint16_t P, uint16_t I, uint16_t D) {
    CANMessage msg;
    msg.id = (m_canID << 4) | MESSAGE_ID_PID;

    msg.data[0] = (uint8_t)(P & 0xFF);
    msg.data[1] = (uint8_t)((P >> 8) & 0xFF);

    msg.data[2] = (uint8_t)(I & 0xFF);
    msg.data[3] = (uint8_t)((I >> 8) & 0xFF);

    msg.data[4] = (uint8_t)(D & 0xFF);
    msg.data[5] = (uint8_t)((D >> 8) & 0xFF);

    msg.len = 6;
    const bool ok = m_canBus->tryToSend(msg);
}

void Smoco::setSoftLimitPosition(int32_t positionA, int32_t positionB) {
    CANMessage msg;
    msg.id = (m_canID << 4) | MESSAGE_ID_SOFT_LIMIT;

    msg.data[0] = (uint8_t)(positionA & 0xFF);
    msg.data[1] = (uint8_t)((positionA >> 8) & 0xFF);
    msg.data[2] = (uint8_t)((positionA >> 16) & 0xFF);
    msg.data[3] = (uint8_t)((positionA >> 24) & 0xFF);

    msg.data[4] = (uint8_t)(positionB & 0xFF);
    msg.data[5] = (uint8_t)((positionB >> 8) & 0xFF);
    msg.data[6] = (uint8_t)((positionB >> 16) & 0xFF);
    msg.data[7] = (uint8_t)((positionB >> 24) & 0xFF);

    msg.len = 8;
    const bool ok = m_canBus->tryToSend(msg);
}

void Smoco::startPositionCalibration(int16_t dutyCycle, int32_t limitSwitchPosition) {
    CANMessage msg;
    msg.id = (m_canID << 4) | MESSAGE_ID_CALIBRATE;
    msg.data[0] = (uint8_t)(dutyCycle & 0xFF);
    msg.data[1] = (uint8_t)((dutyCycle >> 8) & 0xFF);

    msg.data[2] = (uint8_t)(limitSwitchPosition & 0xFF);
    msg.data[3] = (uint8_t)((limitSwitchPosition >> 8) & 0xFF);
    msg.data[4] = (uint8_t)((limitSwitchPosition >> 16) & 0xFF);
    msg.data[5] = (uint8_t)((limitSwitchPosition >> 24) & 0xFF);

    msg.len = 6;
    const bool ok = m_canBus->tryToSend(msg);
}

void Smoco::debugTelemetry(uint8_t enable){
    CANMessage msg;
    msg.id = (m_canID << 4) | 7;
    msg.data[0] = enable;

    msg.len = 1;
    const bool ok = m_canBus->tryToSend(msg);
}

void Smoco::stopAndReset() {
    CANMessage msg;
    msg.id = (m_canID << 4) | MESSAGE_ID_STOP;
    msg.len = 0;
    const bool ok = m_canBus->tryToSend(msg);
}

void Smoco::echoRequest(uint64_t payload) {
    CANMessage msg;
    msg.id = (m_canID << 4) | MESSAGE_ID_ECHO_REQUEST;

    msg.data[0] = (uint8_t)(payload & 0xFF);
    msg.data[1] = (uint8_t)((payload >> 8) & 0xFF);
    msg.data[2] = (uint8_t)((payload >> 16) & 0xFF);
    msg.data[3] = (uint8_t)((payload >> 24) & 0xFF);
    msg.data[4] = (uint8_t)((payload >> 32) & 0xFF);
    msg.data[5] = (uint8_t)((payload >> 40) & 0xFF);
    msg.data[6] = (uint8_t)((payload >> 48) & 0xFF);
    msg.data[7] = (uint8_t)((payload >> 56) & 0xFF);

    msg.len = 8;

    const uint32_t status = m_canBus->tryToSendReturnStatus(msg);
    if (status) {
        Serial.print("OOPS! ");
        Serial.println(status, HEX);
    }
}

void Smoco::smocoPing(){
    m_isPinging = true;
    echoRequest(millis());
}


void Smoco::setAngleVariable(int32_t angle){
    m_angle = angle;
}

void Smoco::setPIDVariables(uint16_t P, uint16_t I, uint16_t D){
    m_PID[0] = P;
    m_PID[1] = I;
    m_PID[2] = D;
}

void Smoco::setAlphaVariable(uint16_t alpha){
    m_lowPassSmoothingAlpha = alpha;
}

void Smoco::setSoftLimitAVariable(int32_t limitA){
    m_softLimitA = limitA;
}

void Smoco::setSoftLimitBVariable(int32_t limitB){
    m_softLimitB = limitB;
}

void Smoco::setIgnoreLimitVariable(bool ignoreLim){
    m_ignoreLimit = ignoreLim;
}

void Smoco::setPingTimeVariable(uint64_t pingTime){
    m_pingTime = pingTime;
}


bool Smoco::getignoreLimitVariable(){
    return m_ignoreLimit;
}
int32_t Smoco::getAngleVariable(){
    return m_angle;
}
bool Smoco::getSoftLimitAVariable(){
    return m_softLimA;
}
bool Smoco::getSoftLimitBVariable(){
    return m_softLimB;
}
bool Smoco::getLimitSwitchAVariable(){
   return m_limSwitchA;
}
bool Smoco::getLimitSwitchBVariable(){
    return m_limSwitchB;
}
bool Smoco::getEchoDataVariable(){
    return m_echoData;
}


