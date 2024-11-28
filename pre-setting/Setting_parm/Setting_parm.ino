/*
   EEPROM Write

   Stores random values into the EEPROM.
   These values will stay in the EEPROM when the board is
   turned off and may be retrieved later by another sketch.
*/
#include <Arduino.h>
#include "config.h"
#include "Wire.h"
#include <MCP3424.h>
#include <ESP32Servo.h>
#include "TypeK.h"
#include "DFRobot_AHT20.h"
#include "SparkFun_External_EEPROM.h"  // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

#include <WiFiClient.h>


// Need this for the lower level access to set them up.
uint8_t address = 0x68;
int i, j;
double temp_cal_tmp[5];
double temp_;
long Voltage;  // Array used to store results
#define LOCATION_SETTINGS 0
double BT_TEMP;
double ET_TEMP;
double AMB_TEMP;
byte tries;
char ap_name[16];
uint8_t macAddr[6];
#define R0 100
#define Rref 1000

const int HEAT_OUT_PIN = PWM_HEAT;  // GPIO26
const int FAN_OUT_PIN = PWM_FAN;
const int frequency = PWM_FREQ;
const byte resolution = PWM_RESOLUTION;  // pwm -0-1023

MCP3424 ADC_MCP3424(address);  // Declaration of MCP3424 A2=0 A1=1 A0=0
DFRobot_AHT20 aht20;
ESP32PWM pwm_heat;
ESP32PWM pwm_fan;
TypeK temp_K_cal;
ExternalEEPROM I2C_EEPROM;
AsyncWebServer server(80);

pid_setting_t pid_parm = {
  .pid_CT = 1.0,      // double pid_CT;
  .p = 1.03,           // double p ;
  .i = 0.07,          // double i ;
  .d = 9.71,           // double d ;
  .BT_tempfix = 0.0,  // double BT_tempfix;
  .ET_tempfix = 0.0   // double ET_tempfix;
};

unsigned long ota_progress_millis = 0;

void onOTAStart() {
  // Log when OTA has started
  // Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    // Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    // Serial.println("OTA update finished successfully!");
  } else {
    // Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}


String IpAddressToString(const IPAddress &ipAddress) {
  return String(ipAddress[0]) + String(".") + String(ipAddress[1]) + String(".") + String(ipAddress[2]) + String(".") + String(ipAddress[3]);
}

String processor(const String &var) {
  if (var == "version") {
    return VERSION;
  } else if (var == "pid_CT") {
    return String(pid_parm.pid_CT);
  } else if (var == "pid_P") {
    return String(pid_parm.p);
  } else if (var == "pid_I") {
    return String(pid_parm.i);
  } else if (var == "pid_D") {
    return String(pid_parm.d);
  } else if (var == "bt_fix") {
    return String(pid_parm.BT_tempfix);
  }
  else if (var == "et_fix") {
    return String(pid_parm.BT_tempfix);
  }
  return String();
}

void loadUserSettings();

void setup() {

  // ESP32PWM::allocateTimer(0);
  // ESP32PWM::allocateTimer(1);
  // ESP32PWM::allocateTimer(2);
  // ESP32PWM::allocateTimer(3);


  //  Init pwm output

  pwm_heat.attachPin(HEAT_OUT_PIN, frequency, resolution);  // 1KHz 8 bit
  pwm_heat.writeScaled(0.0);
  pwm_fan.attachPin(FAN_OUT_PIN, frequency, resolution);  // 1KHz 8 bit
  pwm_fan.writeScaled(0.4);

  Serial.begin(BAUDRATE);
  Serial.println("start...\n");
  Serial.println("INIT PWM...\n");

  vTaskDelay(30000);
  Serial.println("INIT AHT20 and EEPROM...\n");
  // Prepare working .....
  Wire.begin();
  I2C_EEPROM.setMemoryType(64);

  ADC_MCP3424.NewConversion();  // New conversion is initiated
  aht20.begin();

  // 初始化网络服务
  WiFi.macAddress(macAddr);
  // WiFi.mode(WIFI_AP);
  sprintf(ap_name, "MATCHBOX_setup");
  while (WiFi.status() != WL_CONNECTED) {

    delay(1000);
#if defined(DEBUG_MODE)
    Serial.println("wifi not ready");
#endif
    if (tries++ > 2) {
      // init wifi
      WiFi.mode(WIFI_AP);
      WiFi.softAP(ap_name, "matchbox8888");  // defualt IP address :192.168.4.1 password min 8 digis
      break;
    }
  }
// show AP's IP
#if defined(DEBUG_MODE)
  Serial.printf("IP:");
  if (WiFi.getMode() == 2)  // 1:STA mode 2:AP mode
  {
    Serial.println(IpAddressToString(WiFi.softAPIP()));
    local_IP = IpAddressToString(WiFi.softAPIP());
  } else {
    Serial.println(IpAddressToString(WiFi.localIP()));
    local_IP = IpAddressToString(WiFi.localIP());
  }
#endif

  // Start ElegantOTA

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", index_html, processor);
  });

  // server.on("/", handle_root);
  ElegantOTA.begin(&server);  // Start ElegantOTA
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  server.begin();

  Serial.println("Pharse I:Sensor init\n");
  if (aht20.startMeasurementReady(/* crcEn = */ true)) {
    AMB_TEMP = aht20.getTemperature_C();
  }
  Serial.println("Pharse I:OK\n");
  Serial.println("Pharse II:cal temp fix data in 3s ....\n");
  vTaskDelay(3000);
  Serial.println("Temp raw Reading ....\n");
  vTaskDelay(200);
  ADC_MCP3424.Configuration(1, 16, 1, 1);  // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
  Voltage = ADC_MCP3424.Measure();         // Measure is stocked in array Voltage, note that the library will wait for a completed conversion that takes around 200 ms@18bits
  BT_TEMP = pid_parm.BT_tempfix + (((Voltage / 1000 * Rref) / ((3.3 * 1000) - Voltage / 1000) - R0) / (R0 * 0.0039583));
  //BT_TEMP = temp_K_cal.Temp_C(Voltage * 0.001, aht20.getTemperature_C());

  Serial.printf("Temp raw:: AMB_TEMP:%4.2f;BT:%4.2f\n", AMB_TEMP, BT_TEMP);

   pid_parm.BT_tempfix = AMB_TEMP - BT_TEMP;

  //pid_parm.BT_tempfix = 0.0;

  vTaskDelay(200);
  ADC_MCP3424.Configuration(2, 16, 1, 1);  // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
  Voltage = ADC_MCP3424.Measure();         // Measure is stocked in array Voltage, note that the library will wait for a completed conversion that takes around 200 ms@18bits
  //ET_TEMP = pid_parm.ET_tempfix + (((Voltage / 1000 * Rref) / ((3.3 * 1000) - Voltage / 1000) - R0) / (R0 * 0.0039083));
  ET_TEMP = temp_K_cal.Temp_C(Voltage * 0.001, aht20.getTemperature_C());

  Serial.printf("Temp raw:: AMB_TEMP:%4.2f;BT:%4.2f\n", AMB_TEMP, ET_TEMP);

   pid_parm.ET_tempfix = AMB_TEMP - ET_TEMP;
//pid_parm.ET_tempfix = 0.0;


  Serial.printf("Temp fix::BT fix:%4.2f\n", pid_parm.BT_tempfix);
  Serial.println("Pharse II:Done\n");

  Serial.println("Pharse III: Write data into EEPROM...\n");

  // part I :init setting
  Serial.println("start EEPROM setting ...");
  if (!I2C_EEPROM.begin()) {
    Serial.println("failed to initialise EEPROM");
    delay(1000000);
  } else {
    Serial.println("Initialed EEPROM,load data will be writen after 3s...");
    delay(3000);
    I2C_EEPROM.put(0, pid_parm);
    I2C_EEPROM.put(64, pid_parm);
    I2C_EEPROM.put(128, pid_parm);
    I2C_EEPROM.put(192, pid_parm);
    Serial.println("EEPROM,load data for check after 3s...");

    delay(3000);
    loadUserSettings();
    Serial.printf("\nEEPROM value check ...\n");
    Serial.printf("pid_CT:%4.2f\n", pid_parm.pid_CT);
    Serial.printf("PID kp:%4.2f\n", pid_parm.p);
    Serial.printf("PID ki:%4.2f\n", pid_parm.i);
    Serial.printf("PID kd:%4.2f\n", pid_parm.d);
    Serial.printf("BT fix:%4.2f\n", pid_parm.BT_tempfix);
    Serial.printf("ET fix:%4.2f\n", pid_parm.ET_tempfix);
  }
}

void loop() {
  ElegantOTA.loop();
}

// Load the current settings from EEPROM into the settings struct
void loadUserSettings() {

  // Check to see if EEPROM is blank. If the first four spots are zeros then we can assume the EEPROM is blank.
  uint32_t testRead = 0;
  if (I2C_EEPROM.get(LOCATION_SETTINGS, testRead) == 0)  // EEPROM address to read, thing to read into
  {
    // At power on, settings are set to defaults within the struct.
    // So go record the struct as it currently exists so that defaults are set.
    I2C_EEPROM.put(LOCATION_SETTINGS, pid_parm);
    Serial.println("Default settings applied");
  } else {
    // Read current settings
    I2C_EEPROM.get(LOCATION_SETTINGS, pid_parm);
  }
}
