#include <Arduino.h>
#include "SPI.h"
#include "SPIFFS.h"
#include <WiFi.h>
#include "WiFiGeneric.h"
#include "math.h"
#include <AsyncTCP.h>
#include <VL53L0X.h>
#include <Wire.h>

#ifdef USE_ETH_INSTEAD_WIFI
#include <ETH.h>
#endif

#ifdef MQTT_ENABLE
#include <MQTT.h>
#endif

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.
//#define LONG_RANGE

// now set in platformio.ini
//#define DEBUG_PRINT 1    // SET TO 0 OUT TO REMOVE TRACES

#ifdef DEBUG_PRINT
#pragma message("Info : DEBUG_PRINT=1")
#define debug_begin(...) Serial.begin(__VA_ARGS__);
#define debug_print(...) Serial.print(__VA_ARGS__);
#define debug_write(...) Serial.write(__VA_ARGS__);
#define debug_println(...) Serial.println(__VA_ARGS__);
#define debug_printf(...) Serial.printf(__VA_ARGS__);
#else
#define debug_begin(...)
#define debug_print(...)
#define debug_printf(...)
#define debug_write(...)
#define debug_println(...)
#endif

#ifndef WITHOUT_TEMP
#include "driver/temp_sensor.h"
#endif

#include <ESPAsyncWebServer.h>
#include "ESP32Time.h"

// for EEPROM Emulation
#include <Preferences.h>

#include "FileVarStore.h"
#include "AsyncWebLog.h"
#include "AsyncWebOTA.h"

#ifdef DEF_S2_MINI
#pragma message(": Info for CuteCom: Set DTR for Serial-communication")
#endif


// ######################### Global Variables ##################################

ESP32Time rtc(0);

AsyncWebServer server(80);
#ifdef MQTT_ENABLE
WiFiClient client;
#endif
const char* PARAM_MESSAGE = "message";

static uint16_t exampleValue = 6;
static uint16_t SYS_RestartCount = 0;
static uint16_t SYS_TimeoutCount = 0;
static String SYS_Version = "V 1.0.0";
static String SYS_CompileTime =  __DATE__;
static String SYS_IP = "0.0.0.0";

VL53L0X sensor;
uint16_t distance = 0;

#ifdef MQTT_ENABLE
#define MQTT_PAYLOAD_SIZE 256 // maximum MQTT message size
const char MQTT_CLIENTID[] = "SenseDistance";
#endif


// ######################### LED ###############################################

void initLED()
{
#ifndef WITHOUT_LED
  pinMode(LED_GPIO, OUTPUT);
  digitalWrite(LED_GPIO, HIGH);
#endif
}

// i = HIGH / LOW
void setLED(uint8_t i)
{
  debug_printf("LED light %d\r\n", i);
#ifndef WITHOUT_LED
  digitalWrite(LED_GPIO, i);
#endif
}


// ######################### FileVarStore ######################################

class SenseDistance_FileVarStore : public FileVarStore
{
 public:
  // Device-Parameter
  String varDEVICE_s_Name = "SenseDistance";
  uint16_t varDEVICE_i_Interval = 30000; // in milliseconds
  uint16_t varDEVICE_i_MeasurementTimingBudget = 33; // in ms
  uint16_t varDEVICE_i_LongRange=0; // 0=normal, 1=long range
  uint16_t varDEVICE_i_RestartAfterFailedMeasurements = 10;

  // Wifi-Parameter
  String varWIFI_s_Mode    = "STA"; // STA=client connect with Router,  AP=Access-Point-Mode (needs no router)
  String varWIFI_s_Password= "mypassword";
  String varWIFI_s_SSID    = "myssid";

#ifdef MQTT_ENABLE
  uint16_t varMQTT_i_PORT = 1883;
  String varMQTT_s_HOST = "192.168.1.2";
  String varMQTT_s_USER = "";
  String varMQTT_s_PASS = "";
  String varMQTT_s_TOPIC_OUT   = "sensedistance/out/";
  String varMQTT_s_PAYLOAD_OUT ="{\"distance\":%d}";
#endif

 protected:
  void GetVariables()
  {
    varDEVICE_s_Name     = GetVarString(GETVARNAME(varDEVICE_s_Name));
    varWIFI_s_Mode       = GetVarString(GETVARNAME(varWIFI_s_Mode)); //STA or AP
    varWIFI_s_Password   = GetVarString(GETVARNAME(varWIFI_s_Password));
    varWIFI_s_SSID       = GetVarString(GETVARNAME(varWIFI_s_SSID));
    varDEVICE_i_Interval = GetVarInt(GETVARNAME(varDEVICE_i_Interval),30000);
    varDEVICE_i_MeasurementTimingBudget = GetVarInt(GETVARNAME(varDEVICE_i_MeasurementTimingBudget),33);
    varDEVICE_i_LongRange = GetVarInt(GETVARNAME(varDEVICE_i_LongRange),0);
    varDEVICE_i_RestartAfterFailedMeasurements = GetVarInt(GETVARNAME(varDEVICE_i_RestartAfterFailedMeasurements),10);

#ifdef MQTT_ENABLE
    varMQTT_i_PORT       = GetVarInt(GETVARNAME(varMQTT_i_PORT),1883);
    varMQTT_s_HOST       = GetVarString(GETVARNAME(varMQTT_s_HOST));
    varMQTT_s_USER       = GetVarString(GETVARNAME(varMQTT_s_USER));
    varMQTT_s_PASS       = GetVarString(GETVARNAME(varMQTT_s_PASS));
    varMQTT_s_TOPIC_OUT  = GetVarString(GETVARNAME(varMQTT_s_TOPIC_OUT));
    varMQTT_s_PAYLOAD_OUT= GetVarString(GETVARNAME(varMQTT_s_PAYLOAD_OUT));
#endif
  }
};
SenseDistance_FileVarStore varStore;

void initFileVarStore()
{
  varStore.Load();
}


// ######################### MQTT ##############################################

#ifdef MQTT_ENABLE
// Generate MQTT client instance
// N.B.: Default message buffer size is too small!
MQTTClient mqttclient(MQTT_PAYLOAD_SIZE);

// Setup MQTT-client
void mqtt_setup()
{
  debug_println("* MQTT: connecting... ");
  mqttclient.begin(varStore.varMQTT_s_HOST.c_str(), varStore.varMQTT_i_PORT, client);

  while (!mqttclient.connect(MQTT_CLIENTID, varStore.varMQTT_s_USER.c_str(), varStore.varMQTT_s_PASS.c_str()))
  {
    Serial.print(".");
    delay(1000);
  }
  debug_println("* MQTT: connected!");
}

// MQTT run in loop()
inline void mqtt_loop()
{
  mqttclient.loop();
  // fixes some issues with WiFi stability
  delay(10);

  if (!mqttclient.connected())
  {
    mqtt_setup();
  }
  char str[80] = {};
  sprintf(str, varStore.varMQTT_s_PAYLOAD_OUT.c_str(), distance);
  debug_printf("* MQTT Topic_out:%s Payload:%s\r\n", varStore.varMQTT_s_TOPIC_OUT, str);
  mqttclient.publish(varStore.varMQTT_s_TOPIC_OUT, str);
}
#endif // MQTT


// ######################### Ethernet ##########################################

#ifdef USE_ETH_INSTEAD_WIFI
void initEthernet()
{
  debug_print("Starting ETH interface...");
  ETH.begin();
  delay(200);
  ETH.setHostname(varStore.varDEVICE_s_Name.c_str());

  debug_print("ETH MAC: ");
  debug_print(ETH.macAddress());
  debug_print("IP Address: ");
  debug_print(ETH.localIP());
  SYS_IP = ETH.localIP().toString();
  return;
}

void handleEthernetConnection()
{
}
#endif // USE_ETH_INSTEAD_WIFI


// ######################### Wifi ##############################################

void initWifi()
{
  // Test with AP
  //varStore.varWIFI_s_Mode="AP";
  // API Info: https://docs.espressif.com/projects/esp-idf/en/v4.4.6/esp32/api-reference/network/esp_wifi.html

  if (varStore.varWIFI_s_Mode == "AP")
  {
    delay(100);
    debug_println("INFO-WIFI:AP-Mode");
    WiFi.softAP(varStore.varDEVICE_s_Name.c_str());
    debug_print("IP Address: ");
    SYS_IP = WiFi.softAPIP().toString();
    debug_println(SYS_IP);
  }
  else
  {
    debug_printf("INFO-WIFI:STA-Mode\r\n");
    WiFi.mode(WIFI_STA);

    WiFi.setHostname(varStore.varDEVICE_s_Name.c_str());
    WiFi.begin(varStore.varWIFI_s_SSID.c_str(), varStore.varWIFI_s_Password.c_str());
    int i = 0;
    delay(500);
    debug_printf("SSID:%s\r\n", varStore.varWIFI_s_SSID);
    ///debug_printf("Passwort:%s\r\n", varStore.varWIFI_s_Password);
    while ((WiFi.waitForConnectResult() != WL_CONNECTED) && (i < 5))
    {
      debug_printf(".");
      setLED(i%2);
      i++;
      delay(300);
    }
    delay(300);
    if (WiFi.waitForConnectResult() == WL_CONNECTED)
    {
      debug_printf("Get WiFi-Power:%d\r\n",WiFi.getTxPower())
        debug_printf("Get WiFi-RSSI:%d\r\n",WiFi.RSSI());

      debug_print("IP Address: ");
      SYS_IP = WiFi.localIP().toString();
      debug_println(SYS_IP);
      return;
    }
    else
    {
      ESP.restart();
    }
  }

  return;
}

void handleWifiConnection()
{
  // Test if wifi is lost from router
  if ((varStore.varWIFI_s_Mode == "STA") && (WiFi.status() != WL_CONNECTED))
  {
    debug_println("Reconnecting to WiFi...");
    delay(100);
    if (!WiFi.reconnect())
    {
      delay(200);
      AsyncWebLog.println("WiFi lost. Restarting...");
      ESP.restart();
    }
  }
}


// ######################### WEBSERVER #########################################

static String readString(File s)
{
  String ret;
  int c = s.read();
  while (c >= 0)
  {
    ret += (char)c;
    c = s.read();
  }
  return ret;
}

// Replace placeholder "%<variable>%" in HTML-Code
String setHtmlVar(const String& var)
{
  debug_print("func:setHtmlVar: ");
  debug_println(var);

  if (var == "CONFIG") // read config.txt
  {
    if (!SPIFFS.exists("/config.txt"))
    {
      return String(F("Error: File 'config.txt' not found!"));
    }
    // read "config.txt"
    fs::File configfile = SPIFFS.open("/config.txt", "r");
    String sConfig;
    if (configfile)
    {
      sConfig = readString(configfile);
      configfile.close();
    }
    else
    { // no "config.txt"
      sConfig = "";
    }
    return sConfig;
  }
  else if (var== "DEVICEID")
  {
    return varStore.varDEVICE_s_Name;
  }

  if (var == "INFO")
  {
#ifndef WITHOUT_TEMP
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor.dac_offset = TSENS_DAC_L2;  // TSENS_DAC_L2 is default; L4(-40°C ~ 20°C), L2(-10°C ~ 80°C), L1(20°C ~ 100°C), L0(50°C ~ 125°C)
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
    float temp_celsius = 0;
    temp_sensor_read_celsius(&temp_celsius);
    String temp = String(temp_celsius);
#else
    String temp = "(unknown)";
#endif // WITHOUT_TEMP

    return "Version    :"   + SYS_Version +
      "\nBuild      :" + SYS_CompileTime +
      "\nTemp(C)    :" + temp +
      "\nIP-Addr    :" + SYS_IP +
      "\nTimeout-Cnt:" + SYS_TimeoutCount +
      "\nRestart-Cnt:" + String(SYS_RestartCount) +
      "\nRSSI       :" + String(WiFi.RSSI()) +
      "\nMeasure-Timing: " + String(varStore.varDEVICE_i_MeasurementTimingBudget) +
      "\nLongRange: " + String(varStore.varDEVICE_i_LongRange);
  }
  else if (var == "ExampleValue")
  {
    return String(exampleValue);
  }

  return String();
}

void notFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Not found");
}

// for "/" and "/index" handle post
void Handle_Index_Post(AsyncWebServerRequest *request)
{
  debug_println("Argument: " + request->argName(0));
  debug_println("Value: ");

  AsyncWebLog.println("deliver index page");

  request->send(SPIFFS, "/index.html", String(), false, setHtmlVar);
}

void initWebServer()
{
  debug_print("Init web server...\n");

  // Route for root / web page
  server.on("/",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/index.html", String(), false, setHtmlVar);
  });

  // Route for root /index web page
  server.on("/index.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/index.html", String(), false, setHtmlVar);
  });

  // Route for setup web page
  server.on("/setup.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/setup.html", String(), false, setHtmlVar);
  });

  // Route for config web page
  server.on("/config.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/config.html", String(), false, setHtmlVar);
  });

  // > Version V1.2
  // Route for Info-page
  server.on("/info.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/info.html", String(), false, setHtmlVar);
  });

  // Route for style-sheet
  server.on("/style.css",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/style.css", String(), false);
  });

  // fetch GET
  server.on("/fetch", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    // return actual values
    // REMARK: if you change Imax it needs one more GET to return the actual value of Imax
    String s = String(distance);
    request->send(200, "text/plain", s);
    //debug_println("server.on /fetch: "+ s);
  });

  // config.txt GET
  server.on("/config.txt", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/config.txt", "text/html", false);
  });

  // config.txt GET
  server.on("/reboot.html", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(404, "text/plain", "RESTART !");
    ESP.restart();
  });

  //.. some code for the navigation icons
  server.on("/home.png",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/home.png", String(), false);
  });

  server.on("/file-list.png",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/file-list.png", String(), false);
  });

  server.on("/settings.png",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/settings.png", String(), false);
  });

  // ...a lot of code only for icons and favicons ;-))
  server.on("/manifest.json",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/manifest.json", String(), false);
  });

  server.on("/favicon.ico",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/favicon.ico", String(), false);
  });

  server.on("/apple-touch-icon.png",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/apple-touch-icon.png", String(), false);
  });

  server.on("/android-chrome-192x192.png",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/android-chrome-192x192.png", String(), false);
  });

  server.on("/android-chrome-384x384.png",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/android-chrome-384x384.png", String(), false);
  });

  // ------------ POSTs --------------------------------------------------------------------------
  // root (/) POST
  server.on("/",          HTTP_POST, [](AsyncWebServerRequest *request)
  {
     Handle_Index_Post(request);
  });

  // index.html POST
  server.on("/index.html",          HTTP_POST, [](AsyncWebServerRequest *request)
  {
     Handle_Index_Post(request);
  });

  // config.html POST
  server.on("/config.html",          HTTP_POST, [](AsyncWebServerRequest *request)
  {
    //debug_println("Argument: " + request->argName(0));
    //debug_println("Value: ");
    uint8_t i = 0;
    String s  = request->arg(i);
    debug_println(s);
    if (request->argName(0) == "saveconfig")
    {
      varStore.Save(s);
      varStore.Load();
    }
    //debug_println("Request /index3.html");
    request->send(SPIFFS, "/config.html", String(), false, setHtmlVar);
  });

  server.onNotFound(notFound);

  AsyncWebLog.begin(&server);
  AsyncWebOTA.begin(&server);
  server.begin();
}


// ######################### SPIFFS ############################################

void initSPIFFS()
{
  if (!SPIFFS.begin())
  {
    debug_println("*** ERROR: SPIFFS Mount failed");
  }
  else
  {
    debug_println("* INFO: SPIFFS Mount succesfull");
  }
}


// ######################### Setup #############################################

void setup()
{
  delay(800);

  debug_println("Using GPIO "+ String(I2C_SDA_GPIO) + " (SDA) and " + String(I2C_SCL_GPIO) + " (SCL) for I2C");

  Serial.begin(9600);
  Wire.begin(I2C_SDA_GPIO, I2C_SCL_GPIO);

  debug_println("Measure distance each " + String(varStore.varDEVICE_i_Interval) + "ms");
  debug_println("Restart after " + String(varStore.varDEVICE_i_RestartAfterFailedMeasurements) + " failed measurements");

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    debug_println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  // should it use LongRange mode?
  if (varStore.varDEVICE_i_LongRange >= 1)
  {
    debug_println("Using Long Range mode");
    // lower the return signal rate limit (default is 0.25 MCPS)
    sensor.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  }

  // Set the measurement timing budget in microseconds
  debug_println("Setting measurement timing budget to " + String(varStore.varDEVICE_i_MeasurementTimingBudget) + "ms");
  sensor.setMeasurementTimingBudget(varStore.varDEVICE_i_MeasurementTimingBudget * 1000);

  delay(800);
  debug_begin(115200);
  delay(800);
  debug_println("Setup-Start");
  initSPIFFS();
#ifndef WITHOUT_LED
  initLED();
#endif
  initFileVarStore();
  delay(200);
#ifdef USE_ETH_INSTEAD_WIFI
  initEthernet();
#else
  initWifi();
#endif
  delay(200);
#ifdef MQTT_ENABLE
  mqtt_setup();
#endif
  initWebServer();
  delay(400);
  rtc.setTime(0);
}

// ######################### MAIN LOOP #########################################

static unsigned long now = 0;
static unsigned long last = 0;
static unsigned long tmp_poll_time_ms = 0;
static unsigned long previousMillis = 0;
void loop()
{
  now = millis();
  setLED(1);

  if (now - previousMillis >= varStore.varDEVICE_i_Interval) {
    // Update the last run time
    previousMillis = now;

    distance = sensor.readRangeSingleMillimeters();
    if (sensor.timeoutOccurred()) {
      AsyncWebLog.println("Sensor timeout.");
      debug_print("Sensor timeout.");
      last++;
    } else {
      last = 0;
      AsyncWebLog.println("Distance: " + String(distance) + "mm" + "\r\n");
    }

    if (last >= varStore.varDEVICE_i_RestartAfterFailedMeasurements) {
      AsyncWebLog.println("Restarting due to too many failed measurements");
      debug_println("Restarting due to too many failed measurements");
      ESP.restart();
    }
  }

#ifdef USE_ETH_INSTEAD_WIFI
  handleEthernetConnection();
#else
  handleWifiConnection();
#endif

#ifdef MQTT_ENABLE
  mqtt_loop();
#endif
}
