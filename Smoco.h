#ifndef SMOCO_H
#define SMOCO_H

#include <ACAN_T4.h>

#define MESSAGE_ID_POSITION 0x0
#define MESSAGE_ID_POSITION_CALIBRATED 0x1
#define MESSAGE_ID_TARGET 0x2
#define MESSAGE_ID_SMOOTHING 0x3
#define MESSAGE_ID_PID 0x4
#define MESSAGE_ID_SOFT_LIMIT 0x5
#define MESSAGE_ID_CALIBRATE 0x6
#define MESSAGE_ID_DEBUG 0x7
#define MESSAGE_ID_STOP 0xC
#define MESSAGE_ID_ERROR 0xD
#define MESSAGE_ID_ECHO_REQUEST 0xE
#define MESSAGE_ID_ECHO_REPLY 0xF

#define MESSAGE_SID_OPEN_LOOP 0x0
#define MESSAGE_SID_POSITION 0x2
#define MESSAGE_SID_VELOCITY 0x4
#define MESSAGE_SID_CURRENT 0x6

enum class ControlMode {
  STOP,
  OPEN_LOOP,
  POSITION,
  VELOCITY,
  CURRENT,
  CALIBRATING
};

typedef union {
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
    uint16_t alpha;
  } setLowPassSmoothingFactor;
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
} SmocoCANMessage;
typedef struct __attribute__((__packed__)) {
  uint64_t tick;
  int64_t position;
  double velocity;
  double current;
  double pOut;
  double iOut;
  double dOut;
  double error;
  double deltaT;
} SmocoDebugTelemetry;

class Smoco {
public:
  ACAN_T4 *m_canBus;

  uint32_t m_canID;

  int32_t m_position;
  int16_t m_velocity;
  uint8_t m_current;

  bool m_ignoreLimit;
  int16_t m_dutyCycle;
  uint16_t m_errorGain;
  int32_t m_targetPosition;
  int32_t m_targetVelocity;
  uint16_t m_targetCurrent;
  uint16_t m_lowPassSmoothingFactor;
  uint16_t m_PID[3];
  int32_t m_softLimitAPosition;
  int32_t m_softLimitBPosition;
  int16_t m_calibrationDutyCycle;
  int32_t m_calibrationPosition;
  bool m_debugTelemetryEnabled;

  bool m_calibrated;

  bool m_limitSwitchA;
  bool m_limitSwitchB;
  bool m_softLimitA;
  bool m_softLimitB;

  uint64_t m_echoRequestPayload;
  uint64_t m_echoResponse;
  bool m_pinging = false;
  uint64_t m_pingTime = UINT16_MAX;
  uint64_t m_lastEchoResponseTime = 0;
  uint64_t m_pingTimeout = 10000;

  uint8_t m_commandErrorID;

  SmocoDebugTelemetry m_debugTelemetry;

  bool sendLowPassSmoothingFactor();
  bool sendPID();
  bool sendSoftLimitPosition();

  Smoco(ACAN_T4 *canBus, uint8_t canID);

  bool driveOpenLoop(int16_t dutyCycle, bool ignoreLimit = false);
  bool driveTargetPosition(int32_t targetPosition, uint16_t errorGain,
                           bool ignoreLimit = false);
  bool driveTargetVelocity(int32_t targetVelocity, uint16_t errorGain,
                           bool ignoreLimit = false);
  bool driveTargetCurrent(int16_t targetCurrent, uint16_t errorGain,
                          bool ignoreLimit = false);
  bool setLowPassSmoothingFactor(uint16_t alpha);
  bool setPID(uint16_t P, uint16_t I, uint16_t D);
  bool setSoftLimitPosition(int32_t positionA, int32_t positionB);
  bool calibratePosition(int16_t dutyCycle, int32_t limitSwitchPosition);
  bool debugTelemetry(uint8_t enable);
  bool stopAndReset();
  bool echoRequest(uint64_t payload);
  void sync(CANMessage msg);
  bool ping();
};
#endif
