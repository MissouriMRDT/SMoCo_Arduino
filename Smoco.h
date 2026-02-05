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

  uint32_t m_canID; // Upper 2 nybbles of CAN ID

  int32_t m_position; // (step)
  int16_t m_velocity; // (step/s)
  uint8_t m_current;  // (A)

  bool m_ignoreLimit;       // true: ignore limit switch actuation
  int16_t m_dutyCycle;      // (1/32768)
  uint16_t m_errorGain;     // (1/1024)
  int32_t m_targetPosition; // (step)
  int32_t m_targetVelocity; // (step/s)
  uint16_t m_targetCurrent; // (A)
  double m_rampRate;        // (1/s)
  uint16_t m_PID[3];
  int32_t m_softLimitAPosition;   // (step)
  int32_t m_softLimitBPosition;   // (step)
  int16_t m_calibrationDutyCycle; // (1/32768)
  int32_t m_calibrationPosition;  // (step)
  bool m_debugTelemetryEnabled;   // true: SMoCo will continuously send debug
                                  // telemetry

  bool m_calibrated; // true: received Position Calibrated message and
                     // calibratePosition has not been called since

  bool m_limitSwitchA; // true: limit switch A depressed
  bool m_limitSwitchB; // true: limit switch B depressed
  bool m_softLimitA;   // true: soft limit A reached
  bool m_softLimitB;   // true: soft limit B reached

  uint32_t m_echoRequestAt;      // (ms) time echo request was sent
  uint64_t m_echoRequestPayload; // payload of most recent sent Echo Request
  uint64_t m_echoResponse;       // payload of most recent received Echo Reply
  uint64_t m_pingTime =
      UINT16_MAX; // (ms) ping round trip time or UINT16_MAX after ping is
                  // called when millis() > m_echoRequestAt + m_pingTimeout
  uint64_t m_pingTimeout = 10000; // (ms)

  uint8_t
      m_commandErrorID; // ID of the last sent command SMoCo considered invalid

  SmocoDebugTelemetry m_debugTelemetry;

  bool sendRampRate();
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
  bool setRampRate(double rampRate);
  bool setPID(uint16_t P, uint16_t I, uint16_t D);
  bool setSoftLimitPosition(int32_t positionA, int32_t positionB);
  bool calibratePosition(int16_t dutyCycle, int32_t limitSwitchPosition);
  bool debugTelemetry(uint8_t enable);
  bool stopAndReset();
  bool echoRequest(uint64_t payload);
  void sync(CANMessage msg); // Update telemetry variables from the received
                             // message and send a response if necessary
  bool ping();
};
#endif
