/*
   EEPROM Write

   Stores random values into the EEPROM.
   These values will stay in the EEPROM when the board is
   turned off and may be retrieved later by another sketch.
*/

#include <Arduino.h>
#include <ESP32Servo.h>
#include "SparkFun_External_EEPROM.h" // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM
#include <WiFiClient.h>
#include <AsyncTCP.h>
#include <WiFi.h>
#include "config.h"

#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <WString.h>
#include <MycilaWebSerial.h>
#include <ElegantOTA.h>

ExternalEEPROM I2C_EEPROM;
ESP32PWM pwm_heat;
ESP32PWM pwm_fan;
String local_IP;
AsyncWebServer server(80);

// static uint32_t last = millis();

pid_setting_t pid_parm = {
    .pid_CT = 1.5,     // uint16_t pid_CT;
    .p = 2.0,          // double p ;
    .i = 0.12,         // double i ;
    .d = 5.0,          // double d ;
    .BT_tempfix = 0.0, // uint16_t BT_tempfix;
    .ET_tempfix = 0.0  // uint16_t ET_tempfix;
};

const uint32_t frequency = PWM_FREQ;
const byte resolution = PWM_RESOLUTION;
const byte pwm_fan_out = PWM_FAN;
const byte pwm_heat_out = PWM_HEAT;
byte tries;
char ap_name[16];
uint8_t macAddr[6];

unsigned long ota_progress_millis = 0;

void onOTAStart()
{
    // Log when OTA has started
    // Serial.println("OTA update started!");
    // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final)
{
    // Log every 1 second
    if (millis() - ota_progress_millis > 1000)
    {
        ota_progress_millis = millis();
        // Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
    }
}

void onOTAEnd(bool success)
{
    // Log when OTA has finished
    if (success)
    {
        // Serial.println("OTA update finished successfully!");
    }
    else
    {
        // Serial.println("There was an error during OTA update!");
    }
    // <Add your own code here>
}



String processor(const String &var)
{
    if (var == "version")
    {
        return VERSION;
    }
    return String();
}



String IpAddressToString(const IPAddress &ipAddress)
{
    return String(ipAddress[0]) + String(".") +
           String(ipAddress[1]) + String(".") +
           String(ipAddress[2]) + String(".") +
           String(ipAddress[3]);
}

void setup()
{

    // Prepare working .....
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);

    pwm_fan.attachPin(pwm_fan_out, frequency, resolution); // 1KHz 8 bit
    pwm_fan.write(600);

    pwm_heat.attachPin(pwm_heat_out, frequency, resolution); // 1KHz 8 bit
    pwm_heat.write(1);

    vTaskDelay(3000);

    Serial.begin(BAUDRATE);
    Serial.println("start...\n");
    Serial.println("INIT EEPROM...\n");
    Wire.begin();
    I2C_EEPROM.setMemoryType(64);
    if (!I2C_EEPROM.begin())
    {
        Serial.println("failed to initialise EEPROM");
        delay(1000);
    }
    else
    {
        I2C_EEPROM.get(0, pid_parm);
    }

    // 初始化网络服务
    WiFi.macAddress(macAddr);
    // WiFi.mode(WIFI_AP);
    sprintf(ap_name, "MATCHBOX_%02X%02X%02X", macAddr[3], macAddr[4], macAddr[5]);
    while (WiFi.status() != WL_CONNECTED)
    {

        delay(1000);
#if defined(DEBUG_MODE)
        Serial.println("wifi not ready");
#endif
        if (tries++ > 1)
        {
            // init wifi
            WiFi.mode(WIFI_AP);
            WiFi.softAP(ap_name, "matchbox8888"); // defualt IP address :192.168.4.1 password min 8 digis
            break;
        }
    }
// show AP's IP
    Serial.printf("IP:");
    if (WiFi.getMode() == 2) // 1:STA mode 2:AP mode
    {
        Serial.println(IpAddressToString(WiFi.softAPIP()));
        local_IP = IpAddressToString(WiFi.softAPIP());
    }
    else
    {
        Serial.println(IpAddressToString(WiFi.localIP()));
        local_IP = IpAddressToString(WiFi.localIP());
    }

  WebSerial.onMessage([](const String& msg) { Serial.println(msg); });
  WebSerial.begin(&server);
  WebSerial.setBuffer(100);

  server.onNotFound([](AsyncWebServerRequest* request) { request->redirect("/webserial"); });



  ElegantOTA.begin(&server);    // Start ElegantOTA
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", index_html,processor);
  });

  server.begin();

    Serial.println("start EEPROM setting ...");

        Serial.println("READ EEPROM PID PHRASE 1 ,after 3s...");
        delay(3000);
        I2C_EEPROM.get(0, pid_parm);
        Serial.printf("\nEEPROM value check ...\n");
        Serial.printf("\npid_CT:%4.2f", pid_parm.pid_CT);
        Serial.printf("\nPID kp:%4.2f", pid_parm.p);
        Serial.printf("\nPID ki:%4.2f", pid_parm.i);
        Serial.printf("\nPID kd:%4.2f", pid_parm.d);
        Serial.printf("\nBT fix:%4.2f", pid_parm.BT_tempfix);
        Serial.printf("\nET fix:%4.2f\n", pid_parm.ET_tempfix);
        Serial.println("READ EEPROM PID PHRASE 2 ,after 3s...");
        delay(3000);
        I2C_EEPROM.get(128, pid_parm);
        Serial.printf("\nEEPROM value check ...\n");
        Serial.printf("\npid_CT:%4.2f", pid_parm.pid_CT);
        Serial.printf("\nPID kp:%4.2f", pid_parm.p);
        Serial.printf("\nPID ki:%4.2f", pid_parm.i);
        Serial.printf("\nPID kd:%4.2f", pid_parm.d);
        Serial.printf("\nBT fix:%4.2f", pid_parm.BT_tempfix);
        Serial.printf("\nET fix:%4.2f\n", pid_parm.ET_tempfix);

        Serial.println("READ EEPROM PID PHRASE 3 ,after 3s...");
        delay(3000);
        I2C_EEPROM.get(256, pid_parm);
        Serial.printf("\nEEPROM value check ...\n");
        Serial.printf("\npid_CT:%4.2f", pid_parm.pid_CT);
        Serial.printf("\nPID kp:%4.2f", pid_parm.p);
        Serial.printf("\nPID ki:%4.2f", pid_parm.i);
        Serial.printf("\nPID kd:%4.2f", pid_parm.d);
        Serial.printf("\nBT fix:%4.2f", pid_parm.BT_tempfix);
        Serial.printf("\nET fix:%4.2f\n", pid_parm.ET_tempfix);

}

void loop()
{
    ElegantOTA.loop();

    

    // part I :init setting

    WebSerial.println("start EEPROM setting ...");

        WebSerial.println("READ EEPROM PID PHRASE 1 ,after 3s...");
        delay(3000);
        I2C_EEPROM.get(0, pid_parm);
        WebSerial.printf("\nEEPROM value check ...\n");
        WebSerial.printf("\npid_CT:%4.2f", pid_parm.pid_CT);
        WebSerial.printf("\nPID kp:%4.2f", pid_parm.p);
        WebSerial.printf("\nPID ki:%4.2f", pid_parm.i);
        WebSerial.printf("\nPID kd:%4.2f", pid_parm.d);
        WebSerial.printf("\nBT fix:%4.2f", pid_parm.BT_tempfix);
        WebSerial.printf("\nET fix:%4.2f\n", pid_parm.ET_tempfix);
        WebSerial.println("READ EEPROM PID PHRASE 2 ,after 3s...");
        delay(3000);
        I2C_EEPROM.get(128, pid_parm);
        WebSerial.printf("\nEEPROM value check ...\n");
        WebSerial.printf("\npid_CT:%4.2f", pid_parm.pid_CT);
        WebSerial.printf("\nPID kp:%4.2f", pid_parm.p);
        WebSerial.printf("\nPID ki:%4.2f", pid_parm.i);
        WebSerial.printf("\nPID kd:%4.2f", pid_parm.d);
        WebSerial.printf("\nBT fix:%4.2f", pid_parm.BT_tempfix);
        WebSerial.printf("\nET fix:%4.2f\n", pid_parm.ET_tempfix);

        WebSerial.println("READ EEPROM PID PHRASE 3 ,after 3s...");
        delay(3000);
        I2C_EEPROM.get(256, pid_parm);
        WebSerial.printf("\nEEPROM value check ...\n");
        WebSerial.printf("\npid_CT:%4.2f", pid_parm.pid_CT);
        WebSerial.printf("\nPID kp:%4.2f", pid_parm.p);
        WebSerial.printf("\nPID ki:%4.2f", pid_parm.i);
        WebSerial.printf("\nPID kd:%4.2f", pid_parm.d);
        WebSerial.printf("\nBT fix:%4.2f", pid_parm.BT_tempfix);
        WebSerial.printf("\nET fix:%4.2f\n", pid_parm.ET_tempfix);






}
