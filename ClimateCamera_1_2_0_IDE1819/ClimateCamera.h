#ifndef CLIMATE_CAMERA_H
#define CLIMATE_CAMERA_H
#include <stdint.h>

// Климатические структуры копируются между ядрами. Arduino String здесь нет.
struct Config {
  bool enabled;
  float tempTarget, tempHyst, humTarget, humHyst;
  uint16_t fanWorkMin, fanPauseMin, fanTargetRPM;
  float tempOffset, humOffset, pidKp, pidKi;
};
struct CameraSnapshot {
  Config config;
  float temp, hum, rpm, pwm;
  bool sensorOk, pwmReady, fanTachFault;
  bool fridge, humid, fan, otaMode, uiEditing;
  uint32_t uptimeSec, fridgeWaitMs, fridgeRunMs, sensorAgeMs;
};
enum class CameraCommandKind : uint8_t {
  SetConfig, SystemOn, SystemOff, Acknowledge, Save, EnterOta, LeaveOta
};
struct CameraCommand {
  CameraCommandKind kind;
  uint32_t id;
  Config config;
  bool save;
};
struct CameraReply {
  uint32_t id;
  bool ok;
  uint16_t httpStatus;
  char message[128];
};
void cameraSetup();
void cameraLoop();
bool cameraReadSnapshot(CameraSnapshot &out);
bool cameraSubmitCommand(const CameraCommand &command, CameraReply &reply,
                         uint32_t timeoutMs=3000);
#endif
