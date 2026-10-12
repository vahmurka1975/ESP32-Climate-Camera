#ifndef SHT30_RELIABLE_H
#define SHT30_RELIABLE_H
#include <Arduino.h>
#include <Wire.h>

// SHT30/SHT31, single shot, high repeatability, no clock stretching.
class Sht30Reliable {
 public:
  Sht30Reliable(TwoWire &wire,uint8_t sda,uint8_t scl,uint8_t address);
  bool begin();
  bool recover();
  bool startMeasurement();
  bool readMeasurement(float &temperature,float &humidity);
  static uint8_t crc8(const uint8_t *data,size_t length);
 private:
  TwoWire &bus;
  uint8_t pinSda,pinScl,addr;
  bool command(uint16_t value);
  bool readBytes(uint8_t *data,size_t length);
  bool waitClockHigh();
};
#endif
