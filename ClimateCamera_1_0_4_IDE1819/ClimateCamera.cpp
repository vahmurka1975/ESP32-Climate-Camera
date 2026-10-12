/*
 * Климатическая камера ESP32 1.0.4 — Arduino IDE 1.8.19.
 * Основа: приложенные исходники 1.0.2.
 * Удалён отсутствующий исполнительный канал IN3.
 * SHT30: одно совместное измерение, CRC, тайм-аут и восстановление I2C.
 * GPIO и код PWM/PI вентилятора сохранены. IN3 постоянно выключен.
 * Графика и восстановление TFT из 1.0.2 сохранены.
 */
#include "ClimateCamera.h"
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include "Sht30Reliable.h"
#include <esp_arduino_version.h>
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#define FW_VERSION "1.0.4"
// В пропущенных фрагментах исходного кода направление ISR не было видно.
#ifndef ENCODER_DIRECTION
#define ENCODER_DIRECTION 1
#endif
#if !defined(ST7789_DRIVER)
#error "TFT_eSPI: select ST7789_DRIVER in the active setup"
#endif
#if !defined(TFT_DC) || TFT_DC != 25
#error "TFT_eSPI: TFT_DC must be GPIO25"
#endif
#if !defined(TFT_RST) || TFT_RST != 13
#error "TFT_eSPI: TFT_RST must be GPIO13"
#endif
#if !defined(TFT_SCLK) || TFT_SCLK != 18
#error "TFT_eSPI: TFT_SCLK must be GPIO18"
#endif
#if !defined(TFT_MOSI) || TFT_MOSI != 23
#error "TFT_eSPI: TFT_MOSI must be GPIO23"
#endif
#if defined(TFT_CS) && TFT_CS >= 0
#error "TFT CS wiring is unspecified: confirm wiring before using a separate CS pin"
#endif

#if !defined(SPI_FREQUENCY) || SPI_FREQUENCY != 20000000
#error "TFT_eSPI: set SPI_FREQUENCY to 20000000 in the active setup (firmware 1.0.4)"
#endif

struct Config {
  bool enabled;
  float tempTarget, tempHyst, humTarget, humHyst;
  uint16_t fanWorkMin, fanPauseMin, fanTargetRPM;
  float tempOffset, humOffset, pidKp, pidKi;
};
struct CyrillicGlyph { uint16_t cp; uint8_t col[5]; };
struct EventRecord { uint32_t at; char text[96]; };
// Записывается одним NVS blob. Явные типы, нулевое заполнение и CRC.
struct __attribute__((packed)) StoredConfig {
  uint32_t magic;
  uint16_t version, size;
  float tempTarget, tempHyst, humTarget, humHyst;
  uint16_t fanWorkMin, fanPauseMin, fanTargetRPM;
  uint16_t reserved;
  float tempOffset, humOffset, pidKp, pidKi;
  uint32_t crc;
};
static_assert(sizeof(StoredConfig)==52, "Unexpected config layout");
enum : uint8_t { BTN_NONE, BTN_SHORT, BTN_LONG };
enum : uint8_t { UI_PAGE, UI_MENU, UI_EDIT };
enum : uint8_t {
  M_SYSTEM, M_TEMP, M_TEMP_HYST, M_HUM, M_HUM_HYST,
  M_FAN_WORK, M_FAN_PAUSE, M_FAN_RPM, M_TEMP_OFFSET,
  M_HUM_OFFSET, M_KP, M_KI, M_SAVE, M_EXIT, MENU_COUNT
};
static constexpr uint8_t PIN_I2C_SDA=21, PIN_I2C_SCL=22, SHT_ADDR=0x44;
static constexpr uint8_t PIN_ENC_A=32, PIN_ENC_B=33, PIN_ENC_SW=4;
static constexpr uint8_t PIN_FAN_PWM=16, PIN_FAN_TACH=27;
static constexpr uint8_t PIN_RELAY_IN1=15, PIN_RELAY_IN2=17;
static constexpr uint8_t PIN_RELAY_IN3=14, PIN_RELAY_IN4=26;
static constexpr uint8_t RELAY_ON=HIGH, RELAY_OFF=LOW;
static constexpr int16_t SCREEN_W=240, SCREEN_H=240;
static constexpr uint8_t INFO_PAGES=5, EVENT_CAPACITY=40;
static constexpr uint32_t SENSOR_PERIOD_MS=2000, SENSOR_STALE_MS=7000;
static constexpr uint32_t DISPLAY_PERIOD_MS=500, HOME_TIMEOUT_MS=300000;
static constexpr uint32_t COMPRESSOR_MIN_ON_MS=120000;
// Холодильник: пять минут после подачи питания и после каждого выключения.
static constexpr uint32_t FRIDGE_MIN_OFF_MS=300000;
static constexpr uint32_t FRIDGE_START_DELAY_MS=300000;
static constexpr uint32_t FAN_PWM_FREQ=25000;
static constexpr uint8_t FAN_PWM_RES=8, FAN_PWM_CH=0;
static constexpr float TACH_PULSES_PER_REV=2.0f;
static constexpr uint32_t CFG_MAGIC=0x434C4D43UL;
static constexpr uint16_t CFG_SCHEMA=1;
static constexpr uint32_t BUTTON_LONG_MS=800, BUTTON_DEBOUNCE_MS=30;
static constexpr uint8_t MENU_VISIBLE=3;

Config cfg, editCfg;
Preferences prefs;
Sht30Reliable sht(Wire,PIN_I2C_SDA,PIN_I2C_SCL,SHT_ADDR);
TFT_eSPI tft;
// Экранный буфер принадлежит только графике. Никаких дополнительных задач/ISR.
TFT_eSprite canvas(&tft);
static constexpr uint8_t DISPLAY_TILE_H=16;
static constexpr uint8_t DISPLAY_TILE_COUNT=SCREEN_H/DISPLAY_TILE_H;
static_assert(SCREEN_H%DISPLAY_TILE_H==0 && DISPLAY_TILE_COUNT<=16,"TFT tile layout");
static constexpr uint32_t PANEL_REFRESH_MS=120000;
static constexpr uint32_t DISPLAY_STALL_MS=3000;
uint32_t displayTileHash[DISPLAY_TILE_COUNT]={0};
uint16_t displayPendingTiles=0;
uint8_t displayBytesPerPixel=2;
bool displayCacheValid=false;
uint32_t lastPanelRefresh=0;
uint32_t lastDisplayTransferAt=0;
volatile uint32_t tachLastUs=0;
volatile uint32_t encLastUs=0;

EventRecord events[EVENT_CAPACITY];
uint8_t eventHead=0, eventCount=0;
bool relayFridge=false, relayHumid=false, relayFan=false;
bool sensorReady=false, sensorValid=false, filterInitialized=false;
bool fanCycleOn=true, fanTachFault=false, pwmReady=false, displayReady=false;
bool fanWasActive=false, uiDirty=true, serialOverflow=false;
bool humidDemand=false, humidCycleOn=true;
float rawTemp=0, rawHum=0, curTemp=0, curHum=0;
float fanMeasuredRPM=0, fanPWMDuty=0, fanIntegral=0;
uint8_t sensorFailCount=0;
uint32_t bootAt=0, refrigeratorChangedAt=0;
uint32_t lastSensorRead=0, lastSensorGood=0, lastSensorRetry=0;
static constexpr uint32_t SENSOR_RETRY_MS=2000, SENSOR_CONVERSION_MS=20;
bool sensorMeasurementPending=false;
uint32_t sensorConversionStartedAt=0;
uint32_t fanCycleChangedAt=0, fanFaultSince=0, lastFanControl=0;
uint32_t humidCycleChangedAt=0;
uint32_t lastUiActivity=0, lastDraw=0;
uint8_t currentPage=0, uiMode=UI_PAGE;
int8_t menuIndex=0, menuScroll=0;
volatile uint32_t tachPulseCount=0;
volatile int32_t encoderTransitions=0;
volatile uint8_t encoderLastState=0;
portMUX_TYPE ioMux=portMUX_INITIALIZER_UNLOCKED;
char serialBuffer[80];
uint8_t serialLength=0;

// Явные объявления: .cpp не проходит генерацию прототипов Arduino IDE.
void logEvent(const char* fmt, ...);
float safeFloat(float value,float fallback,float low,float high);
void setDefaultSettings(Config &c);
void validateSettings(Config &c);
uint32_t crc32Bytes(const uint8_t *data,size_t count);
StoredConfig packConfig(const Config &c);
void unpackConfig(const StoredConfig &s,Config &c);
bool saveConfig(const Config &source);
void loadSettings();
void setRelayFridge(bool state);
void setRelayHumid(bool state);
void setRelayFan(bool state);
void allRelaysOff();
void disableSystem(const char *reason);
bool enableSystem();
void acknowledgeFault();
uint32_t fridgeStartWait(uint32_t now);
void updateClimateLogic();
void markSensorInvalid(const char *reason);
void sensorReadFailed(const char *reason);
void updateSensor();
void IRAM_ATTR onTachPulse();
void IRAM_ATTR onEncoderChange();
int8_t takeEncoderSteps();
void setFanDuty(uint8_t duty);
void setupFanPWM();
void updateFanControl();
uint8_t readButtonEvent();
const char *menuLabel(uint8_t index);
void formatSetting(uint8_t index,const Config &c,char *buffer,size_t size);
void editSetting(int8_t steps);
void cancelEdit();
void applyConfig(const Config &candidate);
void handleUI();
void printStatus();
void relayTest();
void handleSerialCommand(char *cmd);
void handleSerial();
uint16_t nextUtf8(const char *&p);
int16_t utf8Width(const char *text,uint8_t scale);
const uint8_t *glyphFor(uint16_t cp);
void drawUtf8(int16_t x,int16_t y,const char *text,uint16_t color,uint8_t scale=1,uint16_t bg=TFT_BLACK);
void drawCenteredUtf8(int16_t y,const char *text,uint16_t color,uint8_t scale=1);
void drawHeader(const char *title,uint16_t color);
void drawFooter();
void drawHomePage();
void drawOutputsPage();
void drawFanPage();
void drawTargetsPage();
void drawEventsPage();
void drawMenu();
void drawEdit();
void initDisplay();
uint32_t hashDisplayTile(uint8_t tile);
void transferDisplayTile();
void recoverDisplay();
void drawFitCentered(int16_t y,const char *text,uint16_t color,uint8_t maxScale);
void drawWrapped(int16_t x,int16_t y,const char *text,uint16_t color,uint8_t maxLines=2);
void drawDisplay();

void logEvent(const char* fmt, ...) {
  EventRecord &e=events[eventHead];
  e.at=millis();
  va_list args; va_start(args,fmt);
  vsnprintf(e.text,sizeof(e.text),fmt,args);
  va_end(args);
  // Если snprintf обрезал UTF-8, не оставлять незавершённый символ.
  size_t n=strlen(e.text);
  if(n) {
    size_t i=n-1;
    while(i>0 && ((uint8_t)e.text[i]&0xC0)==0x80) --i;
    uint8_t b=(uint8_t)e.text[i];
    size_t need=b<0x80?1:((b&0xE0)==0xC0?2:((b&0xF0)==0xE0?3:4));
    if(n-i<need) e.text[i]='\0';
  }
  Serial.printf("[%lu] %s\n",(unsigned long)(e.at/1000),e.text);
  eventHead=(eventHead+1)%EVENT_CAPACITY;
  if(eventCount<EVENT_CAPACITY) ++eventCount;
  uiDirty=true;
}
float safeFloat(float value,float fallback,float low,float high) {
  if(!isfinite(value)) return fallback;
  return constrain(value,low,high);
}
void setDefaultSettings(Config &c) {
  c.enabled=true;
  c.tempTarget=12.0f; c.tempHyst=1.0f;
  c.humTarget=80.0f; c.humHyst=3.0f;
  c.fanWorkMin=10; c.fanPauseMin=30; c.fanTargetRPM=500;
  c.tempOffset=0; c.humOffset=0; c.pidKp=0.08f; c.pidKi=0.02f;
}
void validateSettings(Config &c) {
  c.tempTarget=safeFloat(c.tempTarget,12,2,30);
  c.tempHyst=safeFloat(c.tempHyst,1,0.2f,5);
  c.humTarget=safeFloat(c.humTarget,80,30,98);
  c.humHyst=safeFloat(c.humHyst,3,1,10);
  c.tempOffset=safeFloat(c.tempOffset,0,-5,5);
  c.humOffset=safeFloat(c.humOffset,0,-15,15);
  c.pidKp=safeFloat(c.pidKp,0.08f,0,1);
  c.pidKi=safeFloat(c.pidKi,0.02f,0,1);
  c.fanWorkMin=constrain(c.fanWorkMin,(uint16_t)1,(uint16_t)180);
  c.fanPauseMin=constrain(c.fanPauseMin,(uint16_t)0,(uint16_t)360);
  c.fanTargetRPM=constrain(c.fanTargetRPM,(uint16_t)300,(uint16_t)1500);
}
uint32_t crc32Bytes(const uint8_t *data,size_t count) {
  uint32_t crc=0xFFFFFFFFUL;
  for(size_t i=0;i<count;++i) {
    crc^=data[i];
    for(uint8_t j=0;j<8;++j) crc=(crc>>1)^((crc&1)?0xEDB88320UL:0);
  }
  return ~crc;
}
StoredConfig packConfig(const Config &c) {
  StoredConfig s={};
  s.magic=CFG_MAGIC; s.version=CFG_SCHEMA; s.size=sizeof(s);
  s.tempTarget=c.tempTarget; s.tempHyst=c.tempHyst;
  s.humTarget=c.humTarget; s.humHyst=c.humHyst;
  s.fanWorkMin=c.fanWorkMin; s.fanPauseMin=c.fanPauseMin;
  s.fanTargetRPM=c.fanTargetRPM;
  s.tempOffset=c.tempOffset; s.humOffset=c.humOffset;
  s.pidKp=c.pidKp; s.pidKi=c.pidKi;
  s.crc=crc32Bytes((const uint8_t*)&s,offsetof(StoredConfig,crc));
  return s;
}
void unpackConfig(const StoredConfig &s,Config &c) {
  c.enabled=true; // автозапуск после подачи питания
  c.tempTarget=s.tempTarget; c.tempHyst=s.tempHyst;
  c.humTarget=s.humTarget; c.humHyst=s.humHyst;
  c.fanWorkMin=s.fanWorkMin; c.fanPauseMin=s.fanPauseMin;
  c.fanTargetRPM=s.fanTargetRPM;
  c.tempOffset=s.tempOffset; c.humOffset=s.humOffset;
  c.pidKp=s.pidKp; c.pidKi=s.pidKi;
  validateSettings(c);
}
bool saveConfig(const Config &source) {
  Config candidate=source;
  validateSettings(candidate);
  StoredConfig s=packConfig(candidate), check={};
  if(!prefs.begin("climate",false)) { logEvent("Ошибка открытия NVS"); return false; }
  size_t wrote=prefs.putBytes("cfg_v43",&s,sizeof(s));
  size_t read=prefs.getBytes("cfg_v43",&check,sizeof(check));
  prefs.end();
  if(wrote!=sizeof(s) || read!=sizeof(s) || memcmp(&s,&check,sizeof(s))!=0) {
    logEvent("Ошибка записи NVS: не сохранено"); return false;
  }
  logEvent("Настройки сохранены в NVS");
  return true;
}
void loadSettings() {
  setDefaultSettings(cfg);
  if(!prefs.begin("climate",true)) {
    logEvent("Нет NVS: безопасные настройки"); return;
  }
  size_t length=prefs.getBytesLength("cfg_v43");
  if(length>0) {
    StoredConfig s={};
    bool ok=length==sizeof(s) && prefs.getBytes("cfg_v43",&s,sizeof(s))==sizeof(s);
    ok=ok && s.magic==CFG_MAGIC && s.version==CFG_SCHEMA && s.size==sizeof(s);
    ok=ok && s.crc==crc32Bytes((const uint8_t*)&s,offsetof(StoredConfig,crc));
    prefs.end();
    if(ok) {unpackConfig(s,cfg); logEvent("Настройки NVS загружены");}
    else logEvent("NVS повреждена: безопасные настройки");
    return;
  }
  bool legacy=prefs.getBool("cfg_init",false);
  if(legacy) {
    cfg.tempTarget=prefs.getFloat("tt",12); cfg.tempHyst=prefs.getFloat("th",1);
    cfg.humTarget=prefs.getFloat("ht",80); cfg.humHyst=prefs.getFloat("hh",3);
    cfg.fanWorkMin=prefs.getUShort("fw",10); cfg.fanPauseMin=prefs.getUShort("fp",30);
    cfg.fanTargetRPM=prefs.getUShort("fr",500);
    cfg.tempOffset=prefs.getFloat("to",0); cfg.humOffset=prefs.getFloat("ho",0);
    cfg.pidKp=prefs.getFloat("kp",0.08f); cfg.pidKi=prefs.getFloat("ki",0.02f);
  }
  prefs.end(); validateSettings(cfg); cfg.enabled=true;
  if(legacy) {
    logEvent("Настройки 0.0.42 импортированы");
    saveConfig(cfg); // старые ключи не удаляются
  }
}

void setRelayFridge(bool state) {
  if(relayFridge==state) return;
  digitalWrite(PIN_RELAY_IN4,state?RELAY_ON:RELAY_OFF);
  relayFridge=state; refrigeratorChangedAt=millis();
  logEvent("IN4 холодильник: %s",state?"ВКЛ":"ВЫКЛ");
}
void setRelayHumid(bool state) {
  if(relayHumid==state) return;
  digitalWrite(PIN_RELAY_IN2,state?RELAY_ON:RELAY_OFF);
  relayHumid=state; logEvent("IN2 увлажнитель: %s",state?"ВКЛ":"ВЫКЛ");
}

void setRelayFan(bool state) {
  if(relayFan==state) return;
  digitalWrite(PIN_RELAY_IN1,state?RELAY_ON:RELAY_OFF);
  relayFan=state; logEvent("IN1 вентилятор: %s",state?"ВКЛ":"ВЫКЛ");
}
void allRelaysOff() {
  setRelayFridge(false); setRelayHumid(false);
  // Сохранено исходное выключение PWM; питание снимается реле IN1.
  setFanDuty(0); fanPWMDuty=0; setRelayFan(false);
  fanIntegral=0; fanWasActive=false; fanCycleOn=true;
  fanCycleChangedAt=millis(); fanFaultSince=0;
  humidDemand=false; humidCycleOn=true; humidCycleChangedAt=millis();
  // fanTachFault намеренно не сбрасывается: авария должна быть подтверждена.
}
void disableSystem(const char *reason) {
  bool was=cfg.enabled;
  cfg.enabled=false; allRelaysOff();
  if(was || reason) logEvent("Система ВЫКЛ: %s",reason?reason:"оператор");
  uiDirty=true;
}
bool enableSystem() {
  if(!pwmReady) {logEvent("Запуск запрещён: ошибка ШИМ"); return false;}
  if(!sensorValid || millis()-lastSensorGood>=SENSOR_STALE_MS) {
    logEvent("Запуск запрещён: нет данных SHT30"); return false;
  }
  if(fanTachFault) {logEvent("Запуск запрещён: подтвердите аварию"); return false;}
  if(!cfg.enabled) {
    cfg.enabled=true; fanWasActive=false;
    fanCycleOn=true; fanCycleChangedAt=millis();
    fanIntegral=0; fanFaultSince=0;
    logEvent("Система ВКЛ: оператор");
  }
  return true;
}
void acknowledgeFault() {
  if(cfg.enabled || relayFan) {logEvent("Сначала выключите систему"); return;}
  if(fanTachFault) {
    fanTachFault=false; fanFaultSince=0;
    logEvent("Авария подтверждена; устраните причину до запуска");
  } else logEvent("Нет аварии для подтверждения");
}

uint32_t fridgeStartWait(uint32_t now) {
  uint32_t uptime=now-bootAt, offTime=now-refrigeratorChangedAt;
  uint32_t a=uptime<FRIDGE_START_DELAY_MS?FRIDGE_START_DELAY_MS-uptime:0;
  uint32_t b=offTime<FRIDGE_MIN_OFF_MS?FRIDGE_MIN_OFF_MS-offTime:0;
  return a>b?a:b;
}
void updateClimateLogic() {
  if(!cfg.enabled || !sensorValid || !pwmReady || fanTachFault) {
    allRelaysOff(); return;
  }
  uint32_t now=millis();

  // Холодильник включается не ранее чем через 5 минут после старта и выключения.
  if(relayFridge) {
    if(curTemp<=cfg.tempTarget && now-refrigeratorChangedAt>=COMPRESSOR_MIN_ON_MS)
      setRelayFridge(false);
  } else if(fridgeStartWait(now)==0 && curTemp>cfg.tempTarget+cfg.tempHyst) {
    setRelayFridge(true);
  }

  // Запрос увлажнения фиксируется ниже нижней границы и снимается только
  // при достижении уставки. Это не даёт циклу прерваться в зоне гистерезиса.
  if(curHum>=cfg.humTarget) humidDemand=false;
  else if(curHum<cfg.humTarget-cfg.humHyst) humidDemand=true;

  if(!humidDemand) {
    if(relayHumid) setRelayHumid(false);
    humidCycleOn=true;
    humidCycleChangedAt=now;
  } else {
    // Увлажнитель: 20 с включён, затем 2 мин выключен. Повторять до цели.
    if(humidCycleOn) {
      if(now-humidCycleChangedAt>=20000UL) {
        setRelayHumid(false);
        humidCycleOn=false;
        humidCycleChangedAt=now;
      } else if(!relayHumid) {
        setRelayHumid(true);
      }
    } else if(now-humidCycleChangedAt>=120000UL) {
      humidCycleOn=true;
      humidCycleChangedAt=now;
      setRelayHumid(true);
    }
  }


}
void markSensorInvalid(const char *reason) {
  bool wasValid=sensorValid;
  sensorValid=false; filterInitialized=false;
  // Реальное отключение происходит до записи сообщения, а не позже в loop.
  allRelaysOff();
  if(wasValid) logEvent("SHT30: %s; выходы отключены",reason);
}
void sensorReadFailed(const char *reason) {
  sensorMeasurementPending=false;
  if(sensorFailCount<255) ++sensorFailCount;
  // Не продолжать управление нагрузками по последнему успешному измерению.
  markSensorInvalid(reason);
  uiDirty=true;
  if(sensorFailCount>=3) {
    sensorReady=false;
    // На следующем проходе выполняется ограниченное по времени восстановление.
    lastSensorRetry=millis()-SENSOR_RETRY_MS;
  }
}
void updateSensor() {
  uint32_t now=millis();
  if(sensorValid && (uint32_t)(now-lastSensorGood)>=SENSOR_STALE_MS)
    markSensorInvalid("данные устарели");

  if(!sensorReady) {
    if((uint32_t)(now-lastSensorRetry)<SENSOR_RETRY_MS) return;
    lastSensorRetry=now;
    sensorMeasurementPending=false;
    sensorReady=sht.recover();
    if(!sensorReady) return;
    sensorFailCount=0;
    lastSensorRead=millis()-SENSOR_PERIOD_MS;
    logEvent("SHT30: шина и датчик восстановлены");
    now=millis();
  }

  if(!sensorMeasurementPending) {
    if((uint32_t)(now-lastSensorRead)<SENSOR_PERIOD_MS) return;
    lastSensorRead=now;
    if(!sht.startMeasurement()) {
      sensorReadFailed("ошибка команды I2C");
      return;
    }
    sensorConversionStartedAt=millis();
    sensorMeasurementPending=true;
    return; // 20 мс ожидания конверсии не блокируют интерфейс и регулирование.
  }
  if((uint32_t)(now-sensorConversionStartedAt)<SENSOR_CONVERSION_MS) return;
  sensorMeasurementPending=false;
  float t=0, h=0;
  if(!sht.readMeasurement(t,h)) {
    sensorReadFailed("ошибка чтения или CRC");
    return;
  }
  sensorFailCount=0; lastSensorGood=millis(); rawTemp=t; rawHum=h;
  float adjustedT=t+cfg.tempOffset;
  float adjustedH=constrain(h+cfg.humOffset,0.0f,100.0f);
  if(!filterInitialized) {curTemp=adjustedT; curHum=adjustedH; filterInitialized=true;}
  else {curTemp+=0.3f*(adjustedT-curTemp); curHum+=0.3f*(adjustedH-curHum);}
  if(!sensorValid) logEvent("SHT30: корректные измерения");
  sensorValid=true; uiDirty=true;
}

void IRAM_ATTR onTachPulse() {
  uint32_t now=micros();
  if((uint32_t)(now-tachLastUs)<120) return; // подавление помех/дребезга
  tachLastUs=now;
  portENTER_CRITICAL_ISR(&ioMux); ++tachPulseCount; portEXIT_CRITICAL_ISR(&ioMux);
}

void IRAM_ATTR onEncoderChange() {
  uint32_t now=micros();
  if((uint32_t)(now-encLastUs)<80) return; // подавление импульсных помех
  encLastUs=now;

  static const int8_t table[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  uint8_t state=(digitalRead(PIN_ENC_A)<<1)|digitalRead(PIN_ENC_B);
  portENTER_CRITICAL_ISR(&ioMux);
  int8_t delta=table[(encoderLastState<<2)|state]*ENCODER_DIRECTION;
  // Ограничение защищает от переполнения при помехах; обычные шаги не теряются.
  if((delta>0 && encoderTransitions<1000000)||(delta<0 && encoderTransitions>-1000000))
    encoderTransitions+=delta;
  encoderLastState=state;
  portEXIT_CRITICAL_ISR(&ioMux);
}

int8_t takeEncoderSteps() {
  portENTER_CRITICAL(&ioMux);
  int32_t steps=encoderTransitions/4;
  steps=constrain(steps,(int32_t)-10,(int32_t)10);
  encoderTransitions-=steps*4; // оставляем дробные И лишние полные шаги
  portEXIT_CRITICAL(&ioMux);
  return (int8_t)steps;
}
void setFanDuty(uint8_t duty) {
  if(!pwmReady) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_FAN_PWM,duty);
#else
  ledcWrite(FAN_PWM_CH,duty);
#endif
}
void setupFanPWM() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  pwmReady=ledcAttach(PIN_FAN_PWM,FAN_PWM_FREQ,FAN_PWM_RES);
#else
  double actual=ledcSetup(FAN_PWM_CH,FAN_PWM_FREQ,FAN_PWM_RES);
  pwmReady=actual>0;
  if(pwmReady) ledcAttachPin(PIN_FAN_PWM,FAN_PWM_CH);
#endif
  if(!pwmReady) logEvent("Ошибка ШИМ вентилятора");
  setFanDuty(0);
  pinMode(PIN_FAN_TACH,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FAN_TACH),onTachPulse,FALLING);
  lastFanControl=millis();
}
void updateFanControl() {
  uint32_t now=millis();
  if(now-lastFanControl<1000) return;
  float dt=(now-lastFanControl)/1000.0f; lastFanControl=now;
  uint32_t pulses;
  portENTER_CRITICAL(&ioMux); pulses=tachPulseCount; tachPulseCount=0; portEXIT_CRITICAL(&ioMux);
  fanMeasuredRPM=pulses*60.0f/(TACH_PULSES_PER_REV*dt);
  bool active=cfg.enabled && sensorValid && pwmReady && !fanTachFault;
  if(active && !fanWasActive) {
    fanCycleOn=true; fanCycleChangedAt=now; fanIntegral=0; fanFaultSince=0;
  }
  fanWasActive=active;
  if(!active) {
    setFanDuty(0); fanPWMDuty=0; setRelayFan(false);
    fanIntegral=0; fanCycleOn=true; fanCycleChangedAt=now; fanFaultSince=0;
    return;
  }
  if(cfg.fanPauseMin==0) {
    // Возобновление из паузы должно получить штатные две секунды запуска.
    if(!fanCycleOn) {
      fanCycleOn=true; fanCycleChangedAt=now; fanIntegral=0; fanFaultSince=0;
      logEvent("Вентилятор: непрерывная работа");
    }
  } else {
    uint32_t duration=(uint32_t)(fanCycleOn?cfg.fanWorkMin:cfg.fanPauseMin)*60000UL;
    if(now-fanCycleChangedAt>=duration) {
      fanCycleOn=!fanCycleOn; fanCycleChangedAt=now; fanIntegral=0; fanFaultSince=0;
      logEvent("Вентилятор: %s",fanCycleOn?"работа":"пауза");
    }
  }
  if(!fanCycleOn) {
    setFanDuty(0); fanPWMDuty=0; setRelayFan(false); fanIntegral=0; fanFaultSince=0;
    return;
  }
  // Формула PI, anti-windup, диапазон и ИНВЕРСИЯ PWM сохранены из 0.0.42.
  float error=(float)cfg.fanTargetRPM-fanMeasuredRPM;
  float proposed=constrain(fanIntegral+error*dt*cfg.pidKi,0.0f,255.0f);
  float raw=cfg.pidKp*error+proposed;
  if((raw>=25 && raw<=255)||(raw>255 && error<0)||(raw<25 && error>0)) fanIntegral=proposed;
  float output=constrain(cfg.pidKp*error+fanIntegral,25.0f,255.0f);
  if(now-fanCycleChangedAt<2000UL) output=255.0f;
  fanPWMDuty=output;
  setFanDuty((uint8_t)(255-output));
  setRelayFan(true);
  if(output>100 && fanMeasuredRPM<100) {
    if(fanFaultSince==0) fanFaultSince=now;
    if(now-fanFaultSince>=10000UL) {
      fanTachFault=true;
      disableSystem("авария вращения вентилятора");
      logEvent("Проверьте вентилятор и тахометр; сброс: ack");
    }
  } else fanFaultSince=0;
  uiDirty=true;
}

uint8_t readButtonEvent() {
  static bool rawOld=true, stable=true, longSent=false;
  static uint32_t changedAt=0, pressedAt=0;
  uint32_t now=millis(); bool raw=digitalRead(PIN_ENC_SW)!=LOW;
  if(raw!=rawOld) {rawOld=raw; changedAt=now;}
  if(raw!=stable && now-changedAt>=BUTTON_DEBOUNCE_MS) {
    stable=raw;
    if(!stable) {pressedAt=now; longSent=false;}
    else if(!longSent) return BTN_SHORT;
  }
  if(!stable && !raw && !longSent && now-pressedAt>=BUTTON_LONG_MS) {
    longSent=true; return BTN_LONG; // событие во время удержания
  }
  return BTN_NONE;
}
const char *menuLabel(uint8_t index) {
  static const char *labels[MENU_COUNT]={
    "СИСТЕМА / АВАРИЯ", "ТЕМПЕРАТУРА", "ГИСТЕРЕЗИС Т", "ВЛАЖНОСТЬ", "ГИСТЕРЕЗИС В",
    "РАБОТА ВЕНТ.", "ПАУЗА ВЕНТ.", "ОБОРОТЫ ВЕНТ.", "ПОПРАВКА Т",
    "ПОПРАВКА В", "PI KP", "PI KI", "СОХРАНИТЬ ВСЕ", "НА ГЛАВНУЮ"
  };
  return index<MENU_COUNT?labels[index]:"";
}
void formatSetting(uint8_t index,const Config &c,char *buffer,size_t size) {
  switch(index) {
    case M_SYSTEM: snprintf(buffer,size,"%s",fanTachFault?"АВАРИЯ":(c.enabled?"ВКЛ":"ВЫКЛ")); break;
    case M_TEMP: snprintf(buffer,size,"%.1f C",c.tempTarget); break;
    case M_TEMP_HYST: snprintf(buffer,size,"%.1f C",c.tempHyst); break;
    case M_HUM: snprintf(buffer,size,"%.1f %%",c.humTarget); break;
    case M_HUM_HYST: snprintf(buffer,size,"%.1f %%",c.humHyst); break;
    case M_FAN_WORK: snprintf(buffer,size,"%u МИН",(unsigned)c.fanWorkMin); break;
    case M_FAN_PAUSE: snprintf(buffer,size,"%u МИН",(unsigned)c.fanPauseMin); break;
    case M_FAN_RPM: snprintf(buffer,size,"%u ОБ/МИН",(unsigned)c.fanTargetRPM); break;
    case M_TEMP_OFFSET: snprintf(buffer,size,"%+.1f C",c.tempOffset); break;
    case M_HUM_OFFSET: snprintf(buffer,size,"%+.1f %%",c.humOffset); break;
    case M_KP: snprintf(buffer,size,"%.3f",c.pidKp); break;
    case M_KI: snprintf(buffer,size,"%.3f",c.pidKi); break;
    default: buffer[0]='\0'; break;
  }
}
void editSetting(int8_t steps) {
  switch(menuIndex) {
    case M_TEMP: editCfg.tempTarget+=0.1f*steps; break;
    case M_TEMP_HYST: editCfg.tempHyst+=0.1f*steps; break;
    case M_HUM: editCfg.humTarget+=0.1f*steps; break;
    case M_HUM_HYST: editCfg.humHyst+=0.1f*steps; break;
    case M_FAN_WORK: editCfg.fanWorkMin=constrain((int)editCfg.fanWorkMin+steps,1,180); break;
    case M_FAN_PAUSE: editCfg.fanPauseMin=constrain((int)editCfg.fanPauseMin+steps,0,360); break;
    case M_FAN_RPM: editCfg.fanTargetRPM=constrain((int)editCfg.fanTargetRPM+10*steps,300,1500); break;
    case M_TEMP_OFFSET: editCfg.tempOffset+=0.1f*steps; break;
    case M_HUM_OFFSET: editCfg.humOffset+=0.1f*steps; break;
    case M_KP: editCfg.pidKp+=0.001f*steps; break;
    case M_KI: editCfg.pidKi+=0.001f*steps; break;
  }
  validateSettings(editCfg);
}
void cancelEdit() {
  if(uiMode==UI_EDIT) logEvent("Редактирование отменено");
  uiMode=UI_MENU; uiDirty=true;
}
void applyConfig(const Config &candidate) {
  bool offsetsChanged=candidate.tempOffset!=cfg.tempOffset || candidate.humOffset!=cfg.humOffset;
  bool gainsChanged=candidate.pidKp!=cfg.pidKp || candidate.pidKi!=cfg.pidKi;
  bool enabled=cfg.enabled;
  cfg=candidate; cfg.enabled=enabled;
  if(offsetsChanged) {
    filterInitialized=false;
    if(sensorValid) {
      curTemp=rawTemp+cfg.tempOffset;
      curHum=constrain(rawHum+cfg.humOffset,0.0f,100.0f);
      filterInitialized=true;
    }
  }
  if(gainsChanged) fanIntegral=0;
  uiDirty=true;
}
void handleUI() {
  int8_t steps=takeEncoderSteps(); uint8_t button=readButtonEvent();
  uint32_t now=millis();
  if(steps || button!=BTN_NONE) {lastUiActivity=now; uiDirty=true;}
  if(uiMode==UI_PAGE) {
    if(steps) currentPage=(currentPage+INFO_PAGES+steps%INFO_PAGES)%INFO_PAGES;
    if(button==BTN_SHORT) {uiMode=UI_MENU; menuIndex=0; menuScroll=0;}
    else if(button==BTN_LONG) currentPage=0;
  } else if(uiMode==UI_MENU) {
    if(steps) {
      menuIndex=constrain((int)menuIndex+steps,0,(int)MENU_COUNT-1);
      if(menuIndex<menuScroll) menuScroll=menuIndex;
      if(menuIndex>=menuScroll+MENU_VISIBLE) menuScroll=menuIndex-MENU_VISIBLE+1;
    }
    if(button==BTN_LONG) {uiMode=UI_PAGE; currentPage=0;}
    else if(button==BTN_SHORT) {
      if(menuIndex==M_SYSTEM) {
        if(cfg.enabled) disableSystem("оператор");
        else if(fanTachFault) acknowledgeFault();
        else enableSystem();
      } else if(menuIndex==M_SAVE) saveConfig(cfg);
      else if(menuIndex==M_EXIT) {uiMode=UI_PAGE; currentPage=0;}
      else {editCfg=cfg; uiMode=UI_EDIT;}
    }
  } else {
    if(steps) editSetting(steps);
    if(button==BTN_LONG) cancelEdit();
    else if(button==BTN_SHORT) {
      validateSettings(editCfg);
      if(saveConfig(editCfg)) {applyConfig(editCfg); uiMode=UI_MENU;}
    }
  }
  if(now-lastUiActivity>=HOME_TIMEOUT_MS && (uiMode!=UI_PAGE || currentPage!=0)) {
    if(uiMode==UI_EDIT) logEvent("Тайм-аут: изменения отменены");
    uiMode=UI_PAGE; currentPage=0; uiDirty=true;
  }
}
void printStatus() {
  Serial.printf("FW %s | system=%s | sensor=%s | PWM=%s | tachFault=%s\n",FW_VERSION,
    cfg.enabled?"ON":"OFF",sensorValid?"OK":"INVALID",pwmReady?"OK":"FAIL",fanTachFault?"YES":"NO");
  if(sensorValid) Serial.printf("T=%.2f C H=%.2f %% | raw T=%.2f H=%.2f\n",curTemp,curHum,rawTemp,rawHum);
  Serial.printf("IN1=%u IN2=%u IN4=%u | RPM=%.0f PI=%.1f %%\n",
    relayFan,relayHumid,relayFridge,fanMeasuredRPM,fanPWMDuty*100.0f/255.0f);
  Serial.printf("Targets: %.1f C / %.1f %% | hyst %.1f / %.1f\n",cfg.tempTarget,cfg.humTarget,cfg.tempHyst,cfg.humHyst);
  uint32_t now=millis();
  Serial.printf("Compressor wait: fridge=%lu s\n",
    (unsigned long)(relayFridge?0:(fridgeStartWait(now)+999)/1000));
}
void relayTest() {
  // Никаких прямых digitalWrite и импульсов компрессору.
  Serial.println("relaytest: физическое переключение запрещено. Используйте status.");
  Serial.println("IN1 вентилятор GPIO15; IN2 увлажнитель GPIO17;");
  Serial.println("IN4 холодильник GPIO26.");
  printStatus();
}
void handleSerialCommand(char *cmd) {
  while(*cmd==' ' || *cmd=='\t') ++cmd;
  size_t n=strlen(cmd);
  while(n && (cmd[n-1]==' ' || cmd[n-1]=='\t')) cmd[--n]='\0';
  if(!n) return;
  if(!strcmp(cmd,"help")) {
    Serial.println("help | status | on | off | ack | save | defaults | events | clear | relaytest");
    Serial.println("on: ручной запуск; ack: подтверждение аварии (не запускает).");
    Serial.println("defaults: выключение и запись заводских настроек. Serial 115200, LF/CRLF.");
  } else if(!strcmp(cmd,"status")) printStatus();
  else if(!strcmp(cmd,"on")) enableSystem();
  else if(!strcmp(cmd,"off")) disableSystem("Serial");
  else if(!strcmp(cmd,"ack")) acknowledgeFault();
  else if(!strcmp(cmd,"save")) {
    if(uiMode==UI_EDIT) logEvent("Завершите или отмените редактирование");
    else saveConfig(cfg);
  } else if(!strcmp(cmd,"defaults")) {
    disableSystem("заводские настройки");
    if(uiMode==UI_EDIT) cancelEdit();
    Config defaults; setDefaultSettings(defaults);
    if(saveConfig(defaults)) applyConfig(defaults);
  } else if(!strcmp(cmd,"events")) {
    for(uint8_t i=0;i<eventCount;++i) {
      uint8_t idx=(eventHead+EVENT_CAPACITY-eventCount+i)%EVENT_CAPACITY;
      Serial.printf("[%lu] %s\n",(unsigned long)(events[idx].at/1000),events[idx].text);
    }
  } else if(!strcmp(cmd,"clear")) {eventHead=0; eventCount=0; logEvent("Журнал очищен");}
  else if(!strcmp(cmd,"relaytest")) relayTest();
  else Serial.println("Неизвестная команда. Введите help.");
}
void handleSerial() {
  uint8_t budget=64;
  while(Serial.available() && budget--) {
    char c=(char)Serial.read();
    if(c=='\r' || c=='\n') {
      if(serialOverflow) Serial.println("Команда слишком длинная: отброшена");
      else if(serialLength) {serialBuffer[serialLength]='\0'; handleSerialCommand(serialBuffer);}
      serialLength=0; serialOverflow=false;
    } else if(c=='\b' || c==127) {if(serialLength && !serialOverflow) --serialLength;}
    else if((uint8_t)c>=32 && !serialOverflow) {
      if(serialLength<sizeof(serialBuffer)-1) serialBuffer[serialLength++]=c;
      else serialOverflow=true;
    }
  }
}

// Кириллица 5x7. Строчные отображаются соответствующими заглавными.
static const CyrillicGlyph cyrillic[]={
  {0x0410,{0x7E,0x11,0x11,0x11,0x7E}}, {0x0411,{0x7F,0x49,0x49,0x49,0x31}},
  {0x0412,{0x7F,0x49,0x49,0x49,0x36}}, {0x0413,{0x7F,0x01,0x01,0x01,0x01}},
  {0x0414,{0x60,0x3E,0x21,0x3F,0x60}}, {0x0415,{0x7F,0x49,0x49,0x49,0x41}},
  {0x0401,{0x7C,0x55,0x54,0x55,0x44}}, {0x0416,{0x77,0x08,0x7F,0x08,0x77}},
  {0x0417,{0x41,0x49,0x49,0x49,0x36}}, {0x0418,{0x7F,0x20,0x10,0x08,0x7F}},
  {0x0419,{0x7E,0x21,0x12,0x09,0x7E}}, {0x041A,{0x7F,0x08,0x14,0x22,0x41}},
  {0x041B,{0x40,0x3E,0x01,0x01,0x7F}}, {0x041C,{0x7F,0x02,0x0C,0x02,0x7F}},
  {0x041D,{0x7F,0x08,0x08,0x08,0x7F}}, {0x041E,{0x3E,0x41,0x41,0x41,0x3E}},
  {0x041F,{0x7F,0x01,0x01,0x01,0x7F}}, {0x0420,{0x7F,0x09,0x09,0x09,0x06}},
  {0x0421,{0x3E,0x41,0x41,0x41,0x22}}, {0x0422,{0x01,0x01,0x7F,0x01,0x01}},
  {0x0423,{0x07,0x48,0x48,0x48,0x3F}}, {0x0424,{0x1C,0x22,0x7F,0x22,0x1C}},
  {0x0425,{0x63,0x14,0x08,0x14,0x63}}, {0x0426,{0x3F,0x20,0x20,0x3F,0x60}},
  {0x0427,{0x0F,0x08,0x08,0x08,0x7F}}, {0x0428,{0x7F,0x40,0x7F,0x40,0x7F}},
  {0x0429,{0x3F,0x20,0x3F,0x20,0x7F}}, {0x042A,{0x01,0x7F,0x48,0x48,0x30}},
  {0x042B,{0x7F,0x48,0x30,0x00,0x7F}}, {0x042C,{0x7F,0x48,0x48,0x48,0x30}},
  {0x042D,{0x22,0x41,0x49,0x49,0x3E}}, {0x042E,{0x7F,0x08,0x3E,0x41,0x3E}},
  {0x042F,{0x46,0x29,0x19,0x09,0x7F}}
};
uint16_t nextUtf8(const char *&p) {
  uint8_t a=(uint8_t)*p;
  if(!a) return 0;
  ++p;
  if(a<0x80) return a;
  if((a&0xE0)==0xC0) {
    uint8_t b=(uint8_t)*p;
    if((b&0xC0)!=0x80) return '?';
    ++p; return ((a&31)<<6)|(b&63);
  }
  if((a&0xF0)==0xE0) {
    uint8_t b=(uint8_t)*p;
    if((b&0xC0)!=0x80) return '?';
    ++p; uint8_t c=(uint8_t)*p;
    if((c&0xC0)!=0x80) return '?';
    ++p; return ((a&15)<<12)|((b&63)<<6)|(c&63);
  }
  while(((uint8_t)*p&0xC0)==0x80) ++p;
  return '?';
}
int16_t utf8Width(const char *text,uint8_t scale) {
  int16_t width=0;
  while(*text) {nextUtf8(text); width+=6*scale;}
  return width;
}
const uint8_t *glyphFor(uint16_t cp) {
  if(cp>=0x0430 && cp<=0x044F) cp-=0x20;
  if(cp==0x0451) cp=0x0401;
  for(size_t i=0;i<sizeof(cyrillic)/sizeof(cyrillic[0]);++i)
    if(cyrillic[i].cp==cp) return cyrillic[i].col;
  return nullptr;
}
void drawUtf8(int16_t x,int16_t y,const char *text,uint16_t color,uint8_t scale,uint16_t bg) {
  // Каждая операция рисует только в RAM, не открывает транзакцию SPI.
  while(*text && x+6*scale<=SCREEN_W) {
    uint16_t cp=nextUtf8(text);
    const uint8_t *glyph=glyphFor(cp);
    if(glyph) {
      // Смежные вертикальные пиксели объединяются в прямоугольники.
      for(uint8_t col=0;col<5;++col) {
        uint8_t row=0;
        while(row<7) {
          if(!(glyph[col]&(1<<row))) {++row; continue;}
          uint8_t first=row;
          while(row<7 && (glyph[col]&(1<<row))) ++row;
          canvas.fillRect(x+col*scale,y+first*scale,scale,(row-first)*scale,color);
        }
      }
    } else canvas.drawChar(x,y,(uint16_t)(cp<128?cp:'?'),color,bg,scale);
    x+=6*scale;
  }
}
void drawCenteredUtf8(int16_t y,const char *text,uint16_t color,uint8_t scale) {
  int16_t x=(SCREEN_W-utf8Width(text,scale))/2;
  drawUtf8(x<0?0:x,y,text,color,scale);
}
void drawFitCentered(int16_t y,const char *text,uint16_t color,uint8_t maxScale) {
  uint8_t scale=maxScale;
  while(scale>2 && utf8Width(text,scale)>SCREEN_W-16) --scale;
  drawCenteredUtf8(y,text,color,scale);
}
void drawWrapped(int16_t x,int16_t y,const char *text,uint16_t color,uint8_t maxLines) {
  // Обрезка только по границам UTF-8; 18 крупных символов на строку.
  const uint8_t capacity=(SCREEN_W-x-8)/12;
  for(uint8_t line=0;line<maxLines && *text;++line) {
    char out[64]; size_t n=0;
    uint8_t count=0;
    while(*text && count<capacity) {
      const char *begin=text; nextUtf8(text);
      size_t bytes=text-begin;
      if(n+bytes>=sizeof(out)) break;
      memcpy(out+n,begin,bytes); n+=bytes; ++count;
    }
    out[n]='\0';
    if(*text && line+1==maxLines && count>=3) {
      // Заменить три последних символа многоточием без повреждения UTF-8.
      const char *p=out; uint8_t c=0;
      while(*p && c<count-3) {nextUtf8(p); ++c;}
      size_t offset=p-out; memcpy(out+offset,"...",4);
    }
    drawUtf8(x,y+line*18,out,color,2);
  }
}
void drawHeader(const char *title,uint16_t color) {
  drawFitCentered(8,title,color,2);
  canvas.drawFastHLine(8,30,224,TFT_DARKGREY);
}
void drawFooter() {
  canvas.drawFastHLine(8,220,224,TFT_DARKGREY);
  char b[40];
  if(uiMode==UI_PAGE) {
    snprintf(b,sizeof(b),"%u/5  ВЕРСИЯ " FW_VERSION,(unsigned)currentPage+1);
    drawCenteredUtf8(224,b,TFT_LIGHTGREY,2);
  } else if(uiMode==UI_MENU) drawCenteredUtf8(224,"КНОПКА: ВЫБОР",TFT_LIGHTGREY,2);
  else drawCenteredUtf8(224,"КНОПКА: СОХРАНИТЬ",TFT_LIGHTGREY,2);
}
void drawHomePage() {
  drawHeader("КЛИМАТ КАМЕРА",TFT_CYAN);
  char b[64];
  canvas.drawRoundRect(6,36,228,65,6,TFT_DARKGREY);
  canvas.drawRoundRect(6,105,228,65,6,TFT_DARKGREY);
  drawCenteredUtf8(42,"ТЕМПЕРАТУРА",TFT_LIGHTGREY,2);
  drawCenteredUtf8(111,"ВЛАЖНОСТЬ",TFT_LIGHTGREY,2);
  if(sensorValid) {
    snprintf(b,sizeof(b),"%.1f C",curTemp); drawFitCentered(61,b,TFT_WHITE,5);
    snprintf(b,sizeof(b),"%.1f %%",curHum); drawFitCentered(130,b,TFT_CYAN,5);
  } else {
    drawCenteredUtf8(67,"--.- C",TFT_RED,4);
    drawCenteredUtf8(136,"--.- %",TFT_RED,4);
  }
  if(fanTachFault) drawCenteredUtf8(181,"АВАРИЯ ВЕНТИЛЯТОРА",TFT_RED,2);
  else if(!sensorValid) drawCenteredUtf8(181,"ОШИБКА ДАТЧИКА",TFT_RED,2);
  else if(!cfg.enabled) drawCenteredUtf8(181,"СИСТЕМА ВЫКЛ",TFT_YELLOW,2);
  else {
    uint32_t remaining=fridgeStartWait(millis());
    if(!relayFridge && remaining) {
      snprintf(b,sizeof(b),"ХОЛ: ПАУЗА %lu С",(unsigned long)((remaining+999)/1000));
      drawCenteredUtf8(181,b,TFT_YELLOW,2);
    } else drawCenteredUtf8(181,"СИСТЕМА ВКЛ",TFT_GREEN,2);
  }
  const char *labels[]={"ХОЛ","УВЛ","ВЕН"};
  bool states[]={relayFridge,relayHumid,relayFan};
  for(uint8_t i=0;i<3;++i) drawUtf8(22+i*80,203,labels[i],states[i]?TFT_GREEN:TFT_DARKGREY,2);
}
void drawOutputsPage() {
  drawHeader("ВЫХОДЫ РЕЛЕ",TFT_YELLOW);
  const char *names[]={"ХОЛОДИЛЬНИК","УВЛАЖНИТЕЛЬ","ВЕНТИЛЯТОР"};
  bool states[]={relayFridge,relayHumid,relayFan};
  for(uint8_t i=0;i<3;++i) {
    int16_t y=42+i*56;
    canvas.drawRoundRect(6,y,228,48,6,states[i]?TFT_GREEN:TFT_DARKGREY);
    drawUtf8(13,y+5,names[i],TFT_WHITE,2);
    char b[28]; const uint8_t inputs[]={4,2,1};
    snprintf(b,sizeof(b),"IN%u",(unsigned)inputs[i]);
    drawUtf8(13,y+23,b,TFT_LIGHTGREY,2);
    drawUtf8(178,y+23,states[i]?"ВКЛ":"ВЫКЛ",states[i]?TFT_GREEN:TFT_RED,2);
  }
}
void drawFanPage() {
  drawHeader("ВЕНТИЛЯТОР",TFT_MAGENTA);
  char b[64];
  snprintf(b,sizeof(b),"%.0f",fanMeasuredRPM); drawFitCentered(47,b,TFT_WHITE,6);
  drawCenteredUtf8(98,"ОБ/МИН",TFT_LIGHTGREY,2);
  snprintf(b,sizeof(b),"ЦЕЛЬ: %u ОБ/МИН",(unsigned)cfg.fanTargetRPM);
  drawCenteredUtf8(124,b,TFT_YELLOW,2);
  if(!pwmReady) drawCenteredUtf8(164,"ОШИБКА ШИМ",TFT_RED,2);
  else if(fanTachFault) {
    drawCenteredUtf8(158,"СБОЙ ТАХОМЕТРА",TFT_RED,2);
    drawCenteredUtf8(187,"СБРОС В МЕНЮ / ACK",TFT_YELLOW,2);
  } else {
    const char *state=(!cfg.enabled || !sensorValid)?"ВЫКЛЮЧЕН":
      (fanCycleOn?"ЦИКЛ: РАБОТА":"ЦИКЛ: ПАУЗА");
    drawCenteredUtf8(158,state,TFT_GREEN,2);
    snprintf(b,sizeof(b),"ШИМ: %.0f %%",fanPWMDuty*100.0f/255.0f);
    drawCenteredUtf8(187,b,TFT_LIGHTGREY,2);
  }
}
void drawTargetsPage() {
  drawHeader("УСТАВКИ КЛИМАТА",TFT_ORANGE);
  char b[48];
  canvas.drawRoundRect(8,39,224,36,5,TFT_DARKGREY);
  drawUtf8(17,49,"ТЕМПЕРАТУРА",TFT_LIGHTGREY,2);
  snprintf(b,sizeof(b),"%.1f C",cfg.tempTarget);
  drawUtf8(151,49,b,TFT_WHITE,2);

  canvas.drawRoundRect(8,81,224,36,5,TFT_DARKGREY);
  drawUtf8(17,91,"ВЛАЖНОСТЬ",TFT_LIGHTGREY,2);
  snprintf(b,sizeof(b),"%.1f %%",cfg.humTarget);
  drawUtf8(151,91,b,TFT_CYAN,2);

  canvas.drawRoundRect(8,123,224,36,5,TFT_DARKGREY);
  drawUtf8(17,133,"ГИСТЕРЕЗИС Т",TFT_LIGHTGREY,2);
  snprintf(b,sizeof(b),"%.1f C",cfg.tempHyst);
  drawUtf8(151,133,b,TFT_WHITE,2);

  canvas.drawRoundRect(8,165,224,36,5,TFT_DARKGREY);
  drawUtf8(17,175,"ГИСТЕРЕЗИС В",TFT_LIGHTGREY,2);
  snprintf(b,sizeof(b),"%.1f %%",cfg.humHyst);
  drawUtf8(151,175,b,TFT_CYAN,2);
}
void drawEventsPage() {
  drawHeader("ЖУРНАЛ СОБЫТИЙ",TFT_CYAN);
  // Последние 3 записи крупно. Все 40, без обрезки: команда events.
  uint8_t count=eventCount<3?eventCount:3;
  for(uint8_t i=0;i<count;++i) {
    uint8_t idx=(eventHead+EVENT_CAPACITY-count+i)%EVENT_CAPACITY;
    int16_t y=38+i*59;
    char b[32]; snprintf(b,sizeof(b),"%lu С",(unsigned long)(events[idx].at/1000));
    drawUtf8(8,y,b,TFT_YELLOW,2);
    drawWrapped(8,y+18,events[idx].text,TFT_LIGHTGREY,2);
    if(i+1<count) canvas.drawFastHLine(8,y+56,224,TFT_DARKGREY);
  }
  if(!count) drawCenteredUtf8(96,"ЖУРНАЛ ПУСТ",TFT_DARKGREY,2);
}
void drawMenu() {
  drawHeader("НАСТРОЙКИ",TFT_YELLOW);
  for(uint8_t row=0;row<MENU_VISIBLE;++row) {
    uint8_t index=menuScroll+row;
    if(index>=MENU_COUNT) continue;
    int16_t y=38+row*59;
    uint16_t bg=index==menuIndex?TFT_DARKGREY:TFT_BLACK;
    canvas.fillRoundRect(4,y,232,55,5,bg);
    if(index==menuIndex) canvas.drawRoundRect(4,y,232,55,5,TFT_CYAN);
    drawUtf8(10,y+8,menuLabel(index),TFT_WHITE,2,bg);
    char b[64]; formatSetting(index,cfg,b,sizeof(b));
    drawUtf8(10,y+31,b,index==menuIndex?TFT_YELLOW:TFT_LIGHTGREY,2,bg);
  }
}
void drawEdit() {
  drawHeader("РЕДАКТИРОВАНИЕ",TFT_YELLOW);
  drawCenteredUtf8(45,menuLabel(menuIndex),TFT_WHITE,2);
  char b[64]; formatSetting(menuIndex,editCfg,b,sizeof(b));
  drawFitCentered(80,b,TFT_CYAN,4);
  drawCenteredUtf8(129,"ЕЩЁ НЕ ПРИМЕНЕНО",TFT_YELLOW,2);
  drawCenteredUtf8(155,"УДЕРЖ.: ОТМЕНА",TFT_LIGHTGREY,2);
  drawCenteredUtf8(184,"СОХРАНИТЬ: КНОПКА",TFT_LIGHTGREY,2);
}
void initDisplay() {
  tft.init(); tft.setRotation(0); tft.setTextWrap(false); tft.setTextSize(1);
  canvas.setColorDepth(16);
  displayBytesPerPixel=2;
  if(!canvas.createSprite(SCREEN_W,SCREEN_H)) {
    // Вариант для фрагментированной RAM без зависимости от PSRAM.
    canvas.setColorDepth(8); displayBytesPerPixel=1;
    if(!canvas.createSprite(SCREEN_W,SCREEN_H)) {
      displayReady=false;
      Serial.println("TFT: нет памяти буфера; управление работает, используйте Serial");
      return;
    }
  }
  canvas.setTextWrap(false); canvas.setTextFont(1); canvas.setTextSize(1);
  canvas.fillSprite(TFT_BLACK);
  displayReady=true; displayCacheValid=false; displayPendingTiles=0;
  lastPanelRefresh=millis();
  lastDisplayTransferAt=lastPanelRefresh;
  uiDirty=true;
}

uint32_t hashDisplayTile(uint8_t tile) {
  const uint8_t *pixels=(const uint8_t*)canvas.getPointer();
  size_t size=(size_t)SCREEN_W*DISPLAY_TILE_H*displayBytesPerPixel;
  pixels+=(size_t)tile*size;
  uint32_t hash=2166136261UL;
  for(size_t i=0;i<size;++i) hash=(hash^pixels[i])*16777619UL;
  return hash;
}
void transferDisplayTile() {
  if(!displayPendingTiles) return;
  uint8_t tile=0;
  while(!(displayPendingTiles&(1U<<tile))) ++tile;
  int16_t y=tile*DISPLAY_TILE_H;
  // Ровно одна полоса за вызов. TFT_eSPI закрывает транзакцию внутри pushSprite.
  canvas.pushSprite(0,y,0,y,SCREEN_W,DISPLAY_TILE_H);
  displayPendingTiles&=~(1U<<tile);
  lastDisplayTransferAt=millis();
}

void recoverDisplay() {
  if(!displayReady) return;

#if (TFT_RST >= 0)
  pinMode(TFT_RST,OUTPUT);
  digitalWrite(TFT_RST,LOW); delay(20);
  digitalWrite(TFT_RST,HIGH); delay(150);
#else
  tft.writecommand(0x01); // SWRESET
  delay(150);
#endif

  tft.init();
  tft.setRotation(0);
  tft.setTextWrap(false);
  tft.setTextSize(1);

  canvas.fillSprite(TFT_BLACK);
  displayCacheValid=false;
  displayPendingTiles=0;
  lastPanelRefresh=millis();
  lastDisplayTransferAt=lastPanelRefresh;
  uiDirty=true;
}

void drawDisplay() {
  if(!displayReady) return;
  uint32_t now=millis();

  // Периодическое безопасное восстановление контроллера (без сырых регистров ST7789).
  if((uint32_t)(now-lastPanelRefresh)>=PANEL_REFRESH_MS) {
    recoverDisplay();
    return;
  }

  if(displayPendingTiles) {
    // Если передача полос неожиданно застыла — перезапуск контроллера.
    if((uint32_t)(now-lastDisplayTransferAt)>=DISPLAY_STALL_MS) {
      recoverDisplay();
      return;
    }
    transferDisplayTile();
    return;
  }

  if(!uiDirty && (uint32_t)(now-lastDraw)<DISPLAY_PERIOD_MS) return;
  if((uint32_t)(now-lastDraw)<100) return;

  // Полная очистка только буфера RAM. TFT никогда не гасится перед текстом.
  canvas.fillSprite(TFT_BLACK);
  if(uiMode==UI_MENU) drawMenu();
  else if(uiMode==UI_EDIT) drawEdit();
  else switch(currentPage) {
    case 0: drawHomePage(); break;
    case 1: drawOutputsPage(); break;
    case 2: drawFanPage(); break;
    case 3: drawTargetsPage(); break;
    default: drawEventsPage(); break;
  }
  drawFooter();

  for(uint8_t tile=0;tile<DISPLAY_TILE_COUNT;++tile) {
    uint32_t hash=hashDisplayTile(tile);
    if(!displayCacheValid || hash!=displayTileHash[tile]) displayPendingTiles|=1U<<tile;
    displayTileHash[tile]=hash;
  }

  displayCacheValid=true;
  lastDraw=now;
  uiDirty=false;
  transferDisplayTile();
}

void cameraSetup() {
  const uint8_t relayPins[]={PIN_RELAY_IN1,PIN_RELAY_IN2,PIN_RELAY_IN3,PIN_RELAY_IN4};
  for(uint8_t i=0;i<4;++i) {digitalWrite(relayPins[i],RELAY_OFF); pinMode(relayPins[i],OUTPUT);}
  bootAt=millis(); refrigeratorChangedAt=bootAt;
  Serial.begin(115200);
  logEvent("Запуск климатической камеры " FW_VERSION);
  loadSettings();
  pinMode(PIN_ENC_A,INPUT_PULLUP); pinMode(PIN_ENC_B,INPUT_PULLUP); pinMode(PIN_ENC_SW,INPUT_PULLUP);
  encoderLastState=(digitalRead(PIN_ENC_A)<<1)|digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A),onEncoderChange,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B),onEncoderChange,CHANGE);
  setupFanPWM();
  sensorReady=sht.begin(); sensorFailCount=0;
  sensorMeasurementPending=false;
  logEvent("SHT30: %s",sensorReady?"обнаружен":"не обнаружен");
  lastSensorRetry=millis();
  initDisplay();
  if(tft.width()!=SCREEN_W || tft.height()!=SCREEN_H) logEvent("Внимание: размер TFT не 240x240");
  fanCycleChangedAt=millis(); humidCycleChangedAt=fanCycleChangedAt; lastFanControl=millis(); lastUiActivity=millis();
  lastSensorRead=millis()-SENSOR_PERIOD_MS;
  logEvent("Интерфейс готов. Автозапуск включён");
}
void cameraLoop() {
  handleSerial(); handleUI(); updateSensor(); updateClimateLogic(); updateFanControl(); drawDisplay(); yield();
}
