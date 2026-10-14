#include "NetworkServer.h"
#include "ClimateCamera.h"
#include "FirmwareVersion.h"
#include "NetworkConfig.h"
#include "WebUI.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <esp_system.h>
#include <esp_ota_ops.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

static WebServer server(CC_HTTP_PORT);
static TaskHandle_t networkHandle=nullptr;
static char csrfToken[33]={0};
static bool restartPending=false;
static uint32_t restartAt=0;
struct OtaSession {
  bool authorized=false, maintenance=false, begun=false;
  bool failed=false, finished=false, headerChecked=false;
  size_t bytes=0;
  uint16_t errorCode=400;
  uint32_t lastChunkMs=0;
  String message;
};
static OtaSession ota;
static bool authorized(bool mutation=false) {
  if(!server.authenticate(CC_ADMIN_USER,CC_ADMIN_PASS)) {
    server.requestAuthentication(BASIC_AUTH,"ESP32 Climate Camera");return false;
  }
  if(mutation && server.header("X-Climate-Token")!=csrfToken) {
    server.send(403,"application/json; charset=utf-8","{\"ok\":false,\"message\":\"Missing or invalid X-Climate-Token\"}");return false;
  }
  return true;
}
static void noCache() {server.sendHeader("Cache-Control","no-store");}
static void sendMessage(uint16_t code,bool ok,const char *message) {
  // Сообщения — внутренние фиксированные строки без пользовательского текста.
  String body=ok?"{\"ok\":true,\"message\":\"":"{\"ok\":false,\"message\":\"";
  for(const char *p=message;*p;++p) {
    if(*p=='\\' || *p=='\"') body+='\\';
    if((uint8_t)*p>=0x20) body+=*p;
  }
  body+="\"}";noCache();server.send(code,"application/json; charset=utf-8",body);
}
static void appendJsonString(String &out,const char *value) {
  out+='"';
  const char hex[]="0123456789abcdef";
  for(const unsigned char *p=(const unsigned char *)value;*p;++p) {
    if(*p=='"' || *p=='\\') {out+='\\'; out+=(char)*p;}
    else if(*p<0x20) {out+="\\u00";out+=hex[*p>>4];out+=hex[*p&15];}
    else out+=(char)*p;
  }
  out+='"';
}
static void handleEvents() {
  if(!authorized())return;
  static CameraEvent records[CAMERA_EVENT_CAPACITY]; // not on the HTTP task stack
  size_t count=cameraReadEvents(records,CAMERA_EVENT_CAPACITY);
  CameraSnapshot s={}; cameraReadSnapshot(s);
  String body; body.reserve(512+count*(CAMERA_EVENT_TEXT_SIZE+80));
  char b[144];
  snprintf(b,sizeof(b),"{\"version\":\"" FW_VERSION "\",\"capacity\":%u,\"count\":%u,\"uptimeSec\":%lu,\"events\":[",
           CAMERA_EVENT_CAPACITY,(unsigned)count,(unsigned long)s.uptimeSec);
  body=b;
  for(size_t i=0;i<count;++i) {
    if(i)body+=',';
    snprintf(b,sizeof(b),"{\"id\":%lu,\"uptimeMs\":%lu,\"uptimeSec\":%lu,\"message\":",
             (unsigned long)records[i].id,(unsigned long)records[i].at,(unsigned long)(records[i].at/1000));
    body+=b;appendJsonString(body,records[i].text);body+='}';
  }
  body+="]}";noCache();server.send(200,"application/json; charset=utf-8",body);
}
static String configJson(const Config &c) {
  char b[640];
  snprintf(b,sizeof(b),
    "{\"enabled\":%s,\"tempTarget\":%.3f,\"tempHyst\":%.3f,\"humTarget\":%.3f,\"humHyst\":%.3f,"
    "\"fanWorkMin\":%u,\"fanPauseMin\":%u,\"fanTargetRPM\":%u,\"tempOffset\":%.3f,"
    "\"humOffset\":%.3f,\"pidKp\":%.4f,\"pidKi\":%.4f}",
    c.enabled?"true":"false",c.tempTarget,c.tempHyst,c.humTarget,c.humHyst,
    (unsigned)c.fanWorkMin,(unsigned)c.fanPauseMin,(unsigned)c.fanTargetRPM,
    c.tempOffset,c.humOffset,c.pidKp,c.pidKi);
  return String(b);
}
static void handleStatus() {
  if(!authorized())return;
  CameraSnapshot s={};
  if(!cameraReadSnapshot(s)) {sendMessage(503,false,"No telemetry yet");return;}
  char t[24]="null",h[24]="null";
  if(s.sensorOk && isfinite(s.temp))snprintf(t,sizeof(t),"%.2f",s.temp);
  if(s.sensorOk && isfinite(s.hum))snprintf(h,sizeof(h),"%.2f",s.hum);
  char b[1000];
  snprintf(b,sizeof(b),
    "{\"version\":\"" FW_VERSION "\",\"temp\":%s,\"hum\":%s,\"rpm\":%.1f,\"pwm\":%.1f,"
    "\"enabled\":%s,\"sensorOk\":%s,\"pwmReady\":%s,\"fanTachFault\":%s,"
    "\"fridge\":%s,\"humid\":%s,\"fan\":%s,\"otaMode\":%s,\"uiEditing\":%s,"
    "\"fridgeWait\":%s,\"fridgeWaitSec\":%lu,\"fridgeRunSec\":%lu,\"uptimeSec\":%lu,",
    t,h,isfinite(s.rpm)?s.rpm:0.0f,isfinite(s.pwm)?s.pwm:0.0f,
    s.config.enabled?"true":"false",s.sensorOk?"true":"false",s.pwmReady?"true":"false",s.fanTachFault?"true":"false",
    s.fridge?"true":"false",s.humid?"true":"false",s.fan?"true":"false",s.otaMode?"true":"false",s.uiEditing?"true":"false",
    s.fridgeWaitMs?"true":"false",(unsigned long)((s.fridgeWaitMs+999)/1000),(unsigned long)(s.fridgeRunMs/1000),(unsigned long)s.uptimeSec);
  String json;json.reserve(1700);json=b;
  json+="\"sensorAgeSec\":";
  if(s.sensorAgeMs==UINT32_MAX)json+="null";
  else {char age[24];snprintf(age,sizeof(age),"%lu",(unsigned long)(s.sensorAgeMs/1000));json+=age;}
  json+=",\"sensorDegraded\":";json+=s.sensorDegraded?"true":"false";
  char failures[16];snprintf(failures,sizeof(failures),"%u",s.sensorFailures);
  json+=",\"sensorFailures\":";json+=failures;
  json+=",\"sensorFailureLimit\":3,\"sensorState\":";
  appendJsonString(json,s.sensorOk?(s.sensorDegraded?"degraded":"ok"):(s.sensorAgeMs==UINT32_MAX?"starting":"failed"));
  json+=",\"sensorErrorCode\":";appendJsonString(json,s.sensorErrorCode);
  json+=",\"sensorError\":";appendJsonString(json,s.sensorError);
  json+=",\"wifiConnected\":";json+=WiFi.status()==WL_CONNECTED?"true":"false";
  json+=",\"staIP\":\"";json+=WiFi.localIP().toString();
  json+="\",\"apIP\":\"";json+=WiFi.softAPIP().toString();
  json+="\",\"token\":\"";json+=csrfToken;json+="\",\"config\":";json+=configJson(s.config);json+="}";
  noCache();server.send(200,"application/json; charset=utf-8",json);
}
static void handleGetConfig() {
  if(!authorized())return;
  CameraSnapshot s={};if(!cameraReadSnapshot(s)){sendMessage(503,false,"No telemetry yet");return;}
  noCache();server.send(200,"application/json; charset=utf-8",configJson(s.config));
}
static bool numberArg(const char *name,double low,double high,bool integer,double &out) {
  String text=server.arg(name);text.trim();
  if(text.length()==0 || text.length()>32)return false;
  char *end=nullptr;errno=0;
  double v=strtod(text.c_str(),&end);
  if(errno==ERANGE || end==text.c_str() || *end!='\0' || !isfinite(v) || v<low || v>high)return false;
  if(integer && floor(v)!=v)return false;
  out=v;return true;
}
static bool knownField(const String &n) {
  static const char *fields[]={"tempTarget","tempHyst","humTarget","humHyst","fanWorkMin","fanPauseMin","fanTargetRPM","tempOffset","humOffset","pidKp","pidKi","save"};
  for(const char *field:fields)if(n==field)return true;
  return false;
}
static void handleSetConfig() {
  if(!authorized(true))return;
  CameraSnapshot state={};if(!cameraReadSnapshot(state)){sendMessage(503,false,"No telemetry yet");return;}
  if(state.otaMode || state.uiEditing){sendMessage(409,false,"OTA or local editing active");return;}
  for(int i=0;i<server.args();++i)if(!knownField(server.argName(i))){sendMessage(400,false,"Unknown config field");return;}
  CameraCommand cmd={};cmd.kind=CameraCommandKind::SetConfig;cmd.config=state.config;cmd.save=false;
  unsigned changed=0;double value=0;
#define READ_FLOAT(name,lo,hi) if(server.hasArg(#name)){if(!numberArg(#name,lo,hi,false,value)){sendMessage(400,false,"Invalid " #name);return;}cmd.config.name=(float)value;++changed;}
#define READ_INT(name,lo,hi) if(server.hasArg(#name)){if(!numberArg(#name,lo,hi,true,value)){sendMessage(400,false,"Invalid " #name);return;}cmd.config.name=(uint16_t)value;++changed;}
  READ_FLOAT(tempTarget,2,30) READ_FLOAT(tempHyst,0.2,5)
  READ_FLOAT(humTarget,30,98) READ_FLOAT(humHyst,1,10)
  READ_INT(fanWorkMin,1,180) READ_INT(fanPauseMin,0,360) READ_INT(fanTargetRPM,300,1500)
  READ_FLOAT(tempOffset,-5,5) READ_FLOAT(humOffset,-15,15)
  READ_FLOAT(pidKp,0,1) READ_FLOAT(pidKi,0,1)
#undef READ_FLOAT
#undef READ_INT
  if(!changed){sendMessage(400,false,"No settings supplied");return;}
  if(server.hasArg("save")) {
    String save=server.arg("save");
    if(save!="0" && save!="1"){sendMessage(400,false,"save must be 0 or 1");return;}
    cmd.save=save=="1";
  }
  CameraReply reply={};cameraSubmitCommand(cmd,reply);
  sendMessage(reply.httpStatus,reply.ok,reply.message);
}
static void handleControl() {
  if(!authorized(true))return;
  if(server.args()!=1 || !server.hasArg("action")){sendMessage(400,false,"Supply action only");return;}
  String action=server.arg("action");CameraCommand cmd={};
  if(action=="on")cmd.kind=CameraCommandKind::SystemOn;
  else if(action=="off")cmd.kind=CameraCommandKind::SystemOff;
  else if(action=="ack")cmd.kind=CameraCommandKind::Acknowledge;
  else if(action=="save")cmd.kind=CameraCommandKind::Save;
  else {sendMessage(400,false,"Unknown action");return;}
  CameraReply reply={};cameraSubmitCommand(cmd,reply);sendMessage(reply.httpStatus,reply.ok,reply.message);
}
static void failOta(uint16_t code,const char *message) {
  if(!ota.failed){ota.errorCode=code;ota.message=message;}
  ota.failed=true;
  if(ota.begun){Update.abort();ota.begun=false;}
}
static void releaseOta() {
  if(!ota.maintenance)return;
  CameraCommand cmd={};cmd.kind=CameraCommandKind::LeaveOta;CameraReply reply={};
  if(!cameraSubmitCommand(cmd,reply)||!reply.ok)Serial.println("OTA release not acknowledged; outputs remain stopped");
  ota.maintenance=false;
}
static void uploadOta() {
  HTTPUpload &upload=server.upload();
  if(upload.status==UPLOAD_FILE_START) {
    ota=OtaSession();ota.lastChunkMs=millis();
    ota.authorized=server.authenticate(CC_ADMIN_USER,CC_ADMIN_PASS) && server.header("X-Climate-Token")==csrfToken;
    if(!ota.authorized){failOta(403,"Unauthorized upload");return;}
    String filename=upload.filename;filename.toLowerCase();
    if(!filename.endsWith(".bin")){failOta(400,"Expected application .bin");return;}
    if(!esp_ota_get_next_update_partition(nullptr)){failOta(409,"No OTA application partition; flash correct partition table by USB");return;}
    CameraCommand cmd={};cmd.kind=CameraCommandKind::EnterOta;CameraReply reply={};
    if(!cameraSubmitCommand(cmd,reply,3000)||!reply.ok){failOta(503,"Outputs-off acknowledgement failed; nothing written");return;}
    ota.maintenance=true;
    if(!Update.begin(UPDATE_SIZE_UNKNOWN,U_FLASH)){failOta(500,"Update.begin failed; check partition and free memory");return;}
    ota.begun=true;
  } else if(upload.status==UPLOAD_FILE_WRITE) {
    ota.lastChunkMs=millis();
    if(ota.failed || !ota.authorized || !ota.begun)return;
    if(!ota.headerChecked) {
      if(!upload.currentSize || upload.buf[0]!=0xE9){failOta(400,"Invalid ESP application image header");return;}
      ota.headerChecked=true;
    }
    if(Update.write(upload.buf,upload.currentSize)!=upload.currentSize){failOta(500,"Flash write failed or application exceeds OTA slot");return;}
    ota.bytes+=upload.currentSize;
  } else if(upload.status==UPLOAD_FILE_END) {
    ota.lastChunkMs=millis();
    if(ota.failed || !ota.authorized || !ota.begun)return;
    if(!ota.headerChecked || ota.bytes==0 || ota.bytes!=upload.totalSize){failOta(400,"Empty or incomplete application image");return;}
    if(!Update.end(true)){failOta(400,"Image validation failed; old application retained");return;}
    ota.begun=false;ota.finished=true;
  } else if(upload.status==UPLOAD_FILE_ABORTED) {
    failOta(400,"Upload aborted; system remains off");releaseOta();
  }
}
static void finishOtaRequest() {
  if(!authorized(true)) {releaseOta();return;}
  if(ota.finished && !ota.failed) {
    sendMessage(200,true,"OTA successful. Rebooting; compressor startup delay will apply.");
    restartPending=true;restartAt=millis()+700;
  } else {
    if(!ota.failed)failOta(400,"No completed application upload");
    releaseOta();sendMessage(ota.errorCode,false,ota.message.c_str());
  }
}
static void setupRoutes() {
  const char *headers[]={"X-Climate-Token"};server.collectHeaders(headers,1);
  server.on("/",HTTP_GET,[](){if(!authorized())return;noCache();server.send_P(200,"text/html; charset=utf-8",INDEX_HTML);});
  server.on("/api/status",HTTP_GET,handleStatus);
  server.on("/api/events",HTTP_GET,handleEvents);
  server.on("/api/config",HTTP_GET,handleGetConfig);
  server.on("/api/config",HTTP_POST,handleSetConfig);
  server.on("/api/control",HTTP_POST,handleControl);
  server.on("/update",HTTP_GET,[](){if(!authorized())return;noCache();server.send_P(200,"text/html; charset=utf-8",UPDATE_HTML);});
  server.on("/update",HTTP_POST,finishOtaRequest,uploadOta);
  server.onNotFound([](){if(!authorized())return;sendMessage(404,false,"Not found");});
  server.begin();
}
static void restartMdns() {
  MDNS.end();
  if(MDNS.begin(CC_HOSTNAME))MDNS.addService("http","tcp",CC_HTTP_PORT);
}
static void networkTask(void *) {
  for(unsigned i=0;i<4;++i)snprintf(csrfToken+8*i,9,"%08lx",(unsigned long)esp_random());
  WiFi.persistent(false);WiFi.mode(WIFI_AP_STA);WiFi.setHostname(CC_HOSTNAME);
  WiFi.setAutoReconnect(true);WiFi.setSleep(false);
  WiFi.softAPConfig(IPAddress(192,168,4,1),IPAddress(192,168,4,1),IPAddress(255,255,255,0));
  if(!WiFi.softAP(CC_AP_SSID,CC_AP_PASS))Serial.println("SoftAP start failed: check password (8..63 characters)");
  if(strlen(CC_WIFI_SSID))WiFi.begin(CC_WIFI_SSID,CC_WIFI_PASS);
  setupRoutes();restartMdns();
  Serial.printf("Web UI v%s: AP http://%s/ ; mDNS http://%s.local/\n",FW_VERSION,WiFi.softAPIP().toString().c_str(),CC_HOSTNAME);
  bool wasConnected=false;uint32_t lastReconnect=millis();
  for(;;) {
    server.handleClient();
    uint32_t now=millis();bool connected=WiFi.status()==WL_CONNECTED;
    if(connected!=wasConnected) {
      wasConnected=connected;restartMdns();
      if(connected)Serial.printf("STA IP: %s\n",WiFi.localIP().toString().c_str());
    }
    if(strlen(CC_WIFI_SSID) && !connected && (uint32_t)(now-lastReconnect)>=30000) {
      lastReconnect=now;WiFi.begin(CC_WIFI_SSID,CC_WIFI_PASS);
    }
    if(ota.maintenance && !ota.finished && (uint32_t)(now-ota.lastChunkMs)>15000) {
      failOta(408,"Upload timed out; system remains off");releaseOta();
    }
    if(restartPending && (int32_t)(now-restartAt)>=0)ESP.restart();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
bool startNetworkTask() {
  if(networkHandle)return true;
  return xTaskCreatePinnedToCore(networkTask,"ClimateNet",12288,nullptr,1,&networkHandle,0)==pdPASS;
}
