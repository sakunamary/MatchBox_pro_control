#include <Arduino.h>
#include <config.h>
#include <HardwareSerial.h

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#include <TASK_read_temp.h>
#include <TASK_BLE_Serial.h>
#include <TASK_HMI_Serial.h>
#include <TASK_modbus_control.h>

// For ESP32-C3
HardwareSerial Serial_HMI(0);

String local_IP;

extern bool loopTaskWDTEnabled;
extern TaskHandle_t loopTaskHandle;

AsyncWebServer server(80);

char ap_name[30];
uint8_t macAddr[6];
AsyncWebServer server(80);

void task_send_Hreg(void *pvParameters);
void task_get_data(void *pvParameters);

void Task_modbus_control(void *pvParameters);
void Task_Thermo_get_data(void *pvParameters);

void setup()
{
    loopTaskWDTEnabled = true;
    xGetDataMutex = xSemaphoreCreateMutex();
    xSerialReadBufferMutex = xSemaphoreCreateMutex();
    pinMode(HEAT_OUT_PIN, OUTPUT);
    pinMode(FAN_OUT_PIN, OUTPUT);

    Serial.begin(BAUDRATE);
    Serial_HMI(0).begin(HMI_BAUDRATE, SERIAL_8N1, -1, -1);

    thermo_BT.begin(MAX318652WIRE); // set to 2WIRE or 4WIRE as necessary
    thermo_ET.begin(MAX318652WIRE); // set to 2WIRE or 4WIRE as necessary

#if defined(DEBUG_MODE)
    Serial.printf("\nHOT AIR ROASTER STARTING...\n");
#endif

    // Init BLE Serial
    // Disable watchdog timers
    disableCore0WDT();
    disableLoopWDT();
    esp_task_wdt_delete(NULL);

    SerialBLE.begin(ap_name, true, 8); // FOR ESP32C3 SuperMini board
    SerialBLE.setTimeout(10);
#if defined(DEBUG_MODE)
    Serial.printf("\nSerial_BT setup OK\n");
#endif

    // 初始化网络服务

    WiFi.macAddress(macAddr);
    WiFi.mode(WIFI_AP);
    sprintf(ap_name, "ROASTER_%02X%02X%02X", macAddr[3], macAddr[4], macAddr[5]);
    WiFi.softAP(ap_name, "12345678"); // defualt IP address :192.168.4.1 password min 8 digis

#if defined(DEBUG_MODE)
    Serial.printf("\nStart Task...\n");
#endif

    /*---------- Task Definition ---------------------*/
    // Setup tasks to run independently.
    xTaskCreate(
        Task_Thermo_get_data, "Thermo_get_data" // 获取HB数据
        ,
        1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK1:Task_Thermo_get_data...\n");
#endif

    xTaskCreate(
        Task_modbus_control, "modbus_control" // 获取HB数据
        ,
        1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK2:Task_modbus_control...\n");
#endif

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", "HOT AIR ROASTER Version:1.0.0"); });

    AsyncElegantOTA.begin(&server); // Start AsyncElegantOTA
    server.begin();

#if defined(DEBUG_MODE)
    Serial.println("HTTP server started");
#endif

    // Init pwm fan  output
    pwm_fan.pause();
    pwm_fan.write(PWM_FAN, 0, PWM_FREQ, PWM_RESOLUTION);
    pwm_fan.resume();
#if defined(DEBUG_MODE)
    pwm_fan.printDebug();

    Serial.println("PWM FAN started");

#endif

    // Init pwm heat  output
    pwm_heat.pause();
    pwm_heat.write(PWM_HEAT, 0, PWM_FREQ, PWM_RESOLUTION);
    pwm_heat.resume();
#if defined(DEBUG_MODE)
    pwm_heat.printDebug();

    Serial.println("PWM FAN started");

#endif

// Init Modbus-TCP
#if defined(DEBUG_MODE)
    Serial.printf("\nStart Modbus-TCP   service...\n");
#endif
    mb.server(502); // Start Modbus IP //default port :502
    mb.addHreg(BT_HREG);
    mb.addHreg(ET_HREG);
    mb.addHreg(HEAT_HREG);
    mb.addHreg(FAN_HREG);

    mb.Hreg(BT_HREG, 0);   // 初始化赋值
    mb.Hreg(ET_HREG, 0);   // 初始化赋值
    mb.Hreg(HEAT_HREG, 0); // 初始化赋值
    mb.Hreg(FAN_HREG, 0);  // 初始化赋值
}

void loop()
{
    mb.task();
}