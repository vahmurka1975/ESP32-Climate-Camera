#include "Sht30Reliable.h"
#include <math.h>
#include <stdio.h>

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
const char *Sht30Reliable::errorName() const {
  switch(diag.error) {
    case Error::None: return "none";
    case Error::I2cBegin: return "i2c_begin";
    case Error::ClockLow: return "scl_stuck_low";
    case Error::DataLow: return "sda_stuck_low";
    case Error::WriteCommand: return "command_write";
    case Error::EndTransmission: return "end_transmission";
    case Error::ShortRead: return "short_read";
    case Error::TemperatureCrc: return "temperature_crc";
    case Error::HumidityCrc: return "humidity_crc";
    case Error::BothCrc: return "temperature_and_humidity_crc";
    case Error::StatusCrc: return "status_crc";
    case Error::StatusFlags: return "status_flags";
    case Error::ValueOutOfRange: return "value_out_of_range";
  }
  return "unknown";
}
void Sht30Reliable::formatError(char *out,size_t length) const {
  if(!out || !length) return;
  switch(diag.error) {
    case Error::None: snprintf(out,length,"нет ошибки"); break;
    case Error::I2cBegin: snprintf(out,length,"не удалось запустить I2C"); break;
    case Error::ClockLow: snprintf(out,length,"SCL удерживается в LOW"); break;
    case Error::DataLow: snprintf(out,length,"SDA удерживается в LOW"); break;
    case Error::WriteCommand:
      snprintf(out,length,"ошибка записи команды 0x%04X",diag.command); break;
    case Error::EndTransmission:
      snprintf(out,length,"ошибка endTransmission(): код %u, команда 0x%04X",diag.wireCode,diag.command); break;
    case Error::ShortRead:
      snprintf(out,length,"получено меньше %u байт: %u/%u",diag.expected,diag.received,diag.expected); break;
    case Error::TemperatureCrc:
      snprintf(out,length,"неверная CRC температуры: 0x%02X, ожидалась 0x%02X",diag.crcReceived,diag.crcExpected); break;
    case Error::HumidityCrc:
      snprintf(out,length,"неверная CRC влажности: 0x%02X, ожидалась 0x%02X",diag.humCrcReceived,diag.humCrcExpected); break;
    case Error::BothCrc:
      snprintf(out,length,"неверная CRC температуры 0x%02X/0x%02X; неверная CRC влажности 0x%02X/0x%02X",
               diag.crcReceived,diag.crcExpected,diag.humCrcReceived,diag.humCrcExpected); break;
    case Error::StatusCrc:
      snprintf(out,length,"неверная CRC статуса: 0x%02X, ожидалась 0x%02X",diag.crcReceived,diag.crcExpected); break;
    case Error::StatusFlags:
      snprintf(out,length,"ошибка статуса SHT30: 0x%04X",diag.status); break;
    case Error::ValueOutOfRange:
      snprintf(out,length,"измерения вне допустимого диапазона"); break;
  }
}
bool Sht30Reliable::command(uint16_t value) {
  diag.command=value;
  bus.beginTransmission(addr);
  size_t written=bus.write((uint8_t)(value>>8));
  written+=bus.write((uint8_t)value);
  uint8_t error=bus.endTransmission(true);
  if(error) { diag.error=Error::EndTransmission; diag.wireCode=error; return false; }
  if(written!=2) { diag.error=Error::WriteCommand; return false; }
  return true;
}
bool Sht30Reliable::readBytes(uint8_t *data,size_t length) {
  size_t received=bus.requestFrom(addr,length,true);
  diag.expected=(uint8_t)length;
  diag.received=(uint8_t)received;
  if(received!=length) {
    while(bus.available()) bus.read();
    diag.error=Error::ShortRead;
    return false;
  }
  for(size_t i=0;i<length;++i) {
    if(!bus.available()) {
      diag.received=(uint8_t)i;
      diag.error=Error::ShortRead;
      return false;
    }
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
  diag=Diagnostic();
  // Release a slave in the middle of a byte. Never drive a HIGH level.
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
    digitalWrite(pinScl,LOW); digitalWrite(pinSda,LOW); delayMicroseconds(5);
    digitalWrite(pinScl,HIGH);
    released=waitClockHigh();
    delayMicroseconds(5); digitalWrite(pinSda,HIGH); delayMicroseconds(5);
    released=released && digitalRead(pinSda)==HIGH;
  }
  bool clockLow=digitalRead(pinScl)==LOW;
  digitalWrite(pinSda,HIGH); digitalWrite(pinScl,HIGH);
  pinMode(pinSda,INPUT_PULLUP); pinMode(pinScl,INPUT_PULLUP);
  if(!bus.begin(pinSda,pinScl,100000)) { diag.error=Error::I2cBegin; return false; }
  bus.setClock(100000); bus.setTimeOut(25);
  if(!released) { diag.error=clockLow?Error::ClockLow:Error::DataLow; return false; }
  if(!command(0x30A2)) return false;
  delay(3);
  if(!command(0x3041)) return false;
  delay(1);
  if(!command(0xF32D)) return false;
  uint8_t status[3];
  if(!readBytes(status,sizeof(status))) return false;
  diag.crcExpected=crc8(status,2); diag.crcReceived=status[2];
  if(diag.crcExpected!=diag.crcReceived) { diag.error=Error::StatusCrc; return false; }
  diag.status=((uint16_t)status[0]<<8)|status[1];
  if(diag.status&0x0003) { diag.error=Error::StatusFlags; return false; }
  return true;
}
bool Sht30Reliable::startMeasurement() {
  diag=Diagnostic();
  return command(0x2400);
}
bool Sht30Reliable::readMeasurement(float &temperature,float &humidity) {
  diag=Diagnostic();
  uint8_t bytes[6];
  if(!readBytes(bytes,sizeof(bytes))) return false;
  diag.crcExpected=crc8(bytes,2); diag.crcReceived=bytes[2];
  diag.humCrcExpected=crc8(bytes+3,2); diag.humCrcReceived=bytes[5];
  bool badT=diag.crcExpected!=diag.crcReceived;
  bool badH=diag.humCrcExpected!=diag.humCrcReceived;
  if(badT || badH) {
    diag.error=(badT && badH)?Error::BothCrc:(badT?Error::TemperatureCrc:Error::HumidityCrc);
    return false;
  }
  uint16_t rawT=((uint16_t)bytes[0]<<8)|bytes[1];
  uint16_t rawH=((uint16_t)bytes[3]<<8)|bytes[4];
  float t=-45.0f+175.0f*((float)rawT/65535.0f);
  float h=100.0f*((float)rawH/65535.0f);
  if(!isfinite(t) || !isfinite(h) || t<-40.0f || t>125.0f || h<0.0f || h>100.0f) {
    diag.error=Error::ValueOutOfRange; return false;
  }
  // Publish the pair only after BOTH CRCs and both ranges pass.
  temperature=t; humidity=h;
  return true;
}
