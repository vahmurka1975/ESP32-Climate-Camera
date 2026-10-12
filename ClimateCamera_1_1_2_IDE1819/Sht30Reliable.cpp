#include "Sht30Reliable.h"
#include <math.h>

Sht30Reliable::Sht30Reliable(TwoWire &wire,uint8_t sda,uint8_t scl,uint8_t address)
  : bus(wire),pinSda(sda),pinScl(scl),addr(address) {}

uint8_t Sht30Reliable::crc8(const uint8_t *data,size_t length) {
  uint8_t crc=0xFF;
  for(size_t i=0;i<length;++i) {
    crc^=data[i];
    for(uint8_t bit=0;bit<8;++bit)
      crc=(crc&0x80)?(uint8_t)((crc<<1)^0x31):(uint8_t)(crc<<1);
  }
  return crc;
}
bool Sht30Reliable::command(uint16_t value) {
  bus.beginTransmission(addr);
  size_t written=bus.write((uint8_t)(value>>8));
  written+=bus.write((uint8_t)value);
  uint8_t error=bus.endTransmission(true);
  return written==2 && error==0;
}
bool Sht30Reliable::readBytes(uint8_t *data,size_t length) {
  size_t received=bus.requestFrom(addr,length,true);
  if(received!=length) {
    while(bus.available()) bus.read();
    return false;
  }
  for(size_t i=0;i<length;++i) {
    if(!bus.available()) return false;
    data[i]=(uint8_t)bus.read();
  }
  return true;
}
bool Sht30Reliable::waitClockHigh() {
  uint32_t start=micros();
  while(digitalRead(pinScl)==LOW) {
    if((uint32_t)(micros()-start)>=1000UL) return false;
    delayMicroseconds(5);
  }
  return true;
}
bool Sht30Reliable::begin() { return recover(); }
bool Sht30Reliable::recover() {
  // Release a slave waiting in the middle of a byte. Never drive a HIGH level.
  bus.end();
  digitalWrite(pinSda,HIGH); pinMode(pinSda,OUTPUT_OPEN_DRAIN);
  digitalWrite(pinScl,HIGH); pinMode(pinScl,OUTPUT_OPEN_DRAIN);
  bool released=waitClockHigh();
  if(released) {
    for(uint8_t pulse=0;pulse<9 && digitalRead(pinSda)==LOW;++pulse) {
      digitalWrite(pinScl,LOW); delayMicroseconds(5);
      digitalWrite(pinScl,HIGH);
      if(!waitClockHigh()) { released=false; break; }
      delayMicroseconds(5);
    }
  }
  if(released) {
    // STOP: SDA rises while SCL is high; account for external clock stretching.
    digitalWrite(pinScl,LOW); digitalWrite(pinSda,LOW); delayMicroseconds(5);
    digitalWrite(pinScl,HIGH);
    released=waitClockHigh();
    delayMicroseconds(5); digitalWrite(pinSda,HIGH); delayMicroseconds(5);
    released=released && digitalRead(pinSda)==HIGH;
  }
  // Leave failed lines released as well. Same GPIO as the original firmware.
  digitalWrite(pinSda,HIGH); digitalWrite(pinScl,HIGH);
  pinMode(pinSda,INPUT_PULLUP); pinMode(pinScl,INPUT_PULLUP);
  if(!bus.begin(pinSda,pinScl,100000)) return false;
  bus.setClock(100000);
  bus.setTimeOut(25); // ESP32 transaction timeout in milliseconds.
  if(!released) return false;
  if(!command(0x30A2)) return false; // SHT3x soft reset; heater defaults to OFF.
  delay(3);
  if(!command(0x3041)) return false; // clear status after reset
  delay(1);
  if(!command(0xF32D)) return false; // verify status + its CRC
  uint8_t status[3];
  if(!readBytes(status,sizeof(status))) return false;
  if(crc8(status,2)!=status[2]) return false;
  uint16_t flags=((uint16_t)status[0]<<8)|status[1];
  return (flags&0x0003)==0; // command/checksum errors
}
bool Sht30Reliable::startMeasurement() {
  return command(0x2400); // simultaneous temperature/humidity, high repeatability
}
bool Sht30Reliable::readMeasurement(float &temperature,float &humidity) {
  uint8_t bytes[6];
  if(!readBytes(bytes,sizeof(bytes))) return false;
  if(crc8(bytes,2)!=bytes[2] || crc8(bytes+3,2)!=bytes[5]) return false;
  uint16_t rawT=((uint16_t)bytes[0]<<8)|bytes[1];
  uint16_t rawH=((uint16_t)bytes[3]<<8)|bytes[4];
  float t=-45.0f+175.0f*((float)rawT/65535.0f);
  float h=100.0f*((float)rawH/65535.0f);
  if(!isfinite(t) || !isfinite(h) || t<-40.0f || t>125.0f || h<0.0f || h>100.0f)
    return false;
  temperature=t; humidity=h;
  return true;
}
