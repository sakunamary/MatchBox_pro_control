#include <Arduino.h>
#include "config.h"
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <StringTokenizer.h>

#include "HD44780_LCD_PCF8574.h"
#include <WiFiClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

#include "SparkFun_External_EEPROM.h" // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM
#include <PID_v1.h>
#include "pidautotuner.h"
#include <TASK_read_temp.h>
#include <TASK_BLE_Serial.h>
#include "TASK_LCD.h"

AsyncWebServer server(80);

String local_IP;
ExternalEEPROM I2C_EEPROM;
ESP32PWM pwm_heat;
ESP32PWM pwm_fan;
extern filterRC BT_TEMP_ft;
extern filterRC ET_TEMP_ft;
extern HD44780LCD LCD;
PIDAutotuner tuner = PIDAutotuner();

PID Heat_pid_controller(&BT_TEMP, &PID_output, &pid_sv, pid_parm.p, pid_parm.i, pid_parm.d, DIRECT);

extern bool loopTaskWDTEnabled;
extern TaskHandle_t loopTaskHandle;
// uint8_t HMI_HAND[HMI_BUFFER_SIZE];
int levelOT1 = 0;
int levelIO3 = 30;
bool pid_status = false;
bool PID_TUNNING = false;
bool first = true;
byte tries;
double PID_output = 0;
double pid_sv;

double pid_out_max = PID_MAX_OUT; // 取值范围 （0-100）
double pid_out_min = PID_MIN_OUT; // 取值范围 （0-100）

const uint32_t frequency = PWM_FREQ;
const byte resolution = PWM_RESOLUTION;
const byte pwm_fan_out = PWM_FAN;
const byte pwm_heat_out = PWM_HEAT;

char ap_name[16];
uint8_t macAddr[6];

pid_setting_t pid_parm = {
    .pid_CT = 1.5,     // uint16_t pid_CT;
    .p = 2.0,          // double p ;
    .i = 0.12,         // double i ;
    .d = 5.0,          // double d ;
    .BT_tempfix = 0.0, // uint16_t BT_tempfix;
    .ET_tempfix = 0.0  // uint16_t ET_tempfix;
};

unsigned long ota_progress_millis = 0;

void onOTAStart()
{
    // Log when OTA has started
    // Serial.println("OTA update started!");
    // <Add your own code here>

    vTaskDelete(xTask_PID_autotune);
    vTaskDelete(xTASK_BLE_CMD_handle);
    vTaskDelete(xTASK_data_to_BLE);
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
    // if (success)
    // {
    //     // Serial.println("OTA update finished successfully!");
    // }
    // else
    // {
    //     Serial.println("There was an error during OTA update!");
    // }
    // <Add your own code here>
}

String IpAddressToString(const IPAddress &ipAddress)
{
    return String(ipAddress[0]) + String(".") +
           String(ipAddress[1]) + String(".") +
           String(ipAddress[2]) + String(".") +
           String(ipAddress[3]);
}

String processor(const String &var)
{
    if (var == "version")
    {
        return VERSION;
    }
    else if (var == "pid_CT")
    {
        return String(pid_parm.pid_CT);
    }
    else if (var == "pid_P")
    {
        return String(pid_parm.p);
    }
    else if (var == "pid_I")
    {
        return String(pid_parm.i);
    }
    else if (var == "pid_D")
    {
        return String(pid_parm.d);
    } else if (var == "bt_fix") {
    return String(pid_parm.BT_tempfix);
    }
    else if (var == "et_fix") {
    return String(pid_parm.ET_tempfix);
    }
    return String();
}

void setup()
{

    xThermoDataMutex = xSemaphoreCreateMutex();

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    pwm_fan.attachPin(pwm_fan_out, frequency, resolution); // 1KHz 8 bit
    pwm_fan.write(600);
    pwm_heat.attachPin(pwm_heat_out, 1000, resolution); // 1KHz 8 bit
    pwm_heat.write(1);

    LCD.PCF8574_LCDInit(LCD.LCDCursorTypeOff);
    LCD.PCF8574_LCDClearScreen();
    LCD.PCF8574_LCDBackLightSet(true);

    BT_TEMP_ft.init(BT_FILTER);
    ET_TEMP_ft.init(ET_FILTER);
    AMB_ft.init(AMB_FILTER);
    fRise.init(RISE_FILTER); // digital filtering for RoR calculation
    fRoR.init(ROR_FILTER);   // post-filtering on RoR values
    first = true;

    Serial.begin(HMI_BAUDRATE);

#if defined(DEBUG_MODE)
    Serial.printf("\nStart PWM...");
#endif

#if defined(DEBUG_MODE)

    Serial.printf("\nStart Task...");
#endif
    Wire.begin();
#if defined(TC_TYPE_K)
    aht20.begin();
#endif
    MCP.NewConversion(); // New conversion is initiated
    I2C_EEPROM.setMemoryType(64);
#if defined(DEBUG_MODE)
    Serial.println("start Reading EEPROM setting ...");
#endif
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

    // init wifi
    Serial.println("WiFi.mode(AP):");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_name, "matchbox8888"); // defualt IP address :192.168.4.1 password min 8 digis

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

    // Create the BLE Device
    BLEDevice::init(ap_name);
    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);
    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new MyCallbacks());
    // Start the service
    pService->start();
    // Start advertising
    pServer->getAdvertising()->start();

    /*---------- Task Definition ---------------------*/
    // Setup tasks to run independently.

    xTaskCreate(
        Task_Thermo_get_data, "Task_Thermo_get_data" // 获取HB数据
        ,
        1024 * 10 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 4 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        &xTask_Thermo_get_data // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK1:Task_Thermo_get_data...");
#endif
    //     // vTaskSuspend(xTask_Thermo_get_data);

    xTaskCreate(
        TASK_DATA_to_BLE, "TASK_DATA_to_BLE" // 获取HB数据
        ,
        1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        &xTASK_data_to_BLE // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK6:TASK_DATA_to_BLE...");
#endif

    xTaskCreate(
        TASK_BLE_CMD_handle, "TASK_BLE_CMD_handle" // 获取HB数据
        ,
        1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        &xTASK_BLE_CMD_handle // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK8:TASK_BLE_CMD_handle...\n");
#endif

    xTaskCreate(
        Task_PID_autotune, "PID autotune" //
        ,
        1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        &xTask_PID_autotune // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
    vTaskSuspend(xTask_PID_autotune); //
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK=8:PID autotune OK\n");
#endif

    xTaskCreate(
        TASK_LCD, "TASK_LCD" // 获取HB数据
        ,
        1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK2:TASK_LCD...");
#endif

    // // init PID
    Heat_pid_controller.SetMode(MANUAL);
    Heat_pid_controller.SetOutputLimits(PID_MIN_OUT, PID_MAX_OUT);
    Heat_pid_controller.SetSampleTime(int(pid_parm.pid_CT * 1000));

    // INIT PID AUTOTUNE

    tuner.setTargetInputValue(PID_TUNE_SV_1);
    tuner.setLoopInterval(pid_parm.pid_CT * uS_TO_S_FACTOR);
    tuner.setOutputRange(round(PID_MIN_OUT * 255 / 100), round(PID_MAX_OUT * 255 / 100));
    tuner.setZNMode(PIDAutotuner::ZNModeNoOvershoot);

    // INIT OTA service
    // server.on("/", handle_root);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/html", index_html, processor); });
    ElegantOTA.begin(&server); // Start ElegantOTA
    // ElegantOTA callbacks
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onProgress(onOTAProgress);
    ElegantOTA.onEnd(onOTAEnd);

    server.begin();
    Serial.println("HTTP server started");

#if defined(DEBUG_MODE)
    Serial.printf("\nEEPROM value check ...\n");
    Serial.printf("pid_CT:%4.2f\n", pid_parm.pid_CT);
    Serial.printf("PID kp:%4.2f\n", pid_parm.p);
    Serial.printf("PID ki:%4.2f\n", pid_parm.i);
    Serial.printf("PID kd:%4.2f\n", pid_parm.d);
    Serial.printf("BT fix:%4.2f\n", pid_parm.BT_tempfix);
    Serial.printf("ET fix:%4.2f\n", pid_parm.ET_tempfix);
#endif
    // init watch dog
    esp_task_wdt_init(TWDT_TIMEOUT_S, false);
    // esp_task_wdt_add(&xTask_Thermo_get_data);
}

void loop()
{
    // server.handleClient();
    ElegantOTA.loop();
    // disconnecting
    if (!deviceConnected && oldDeviceConnected)
    {
        delay(500);                  // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
#if defined(DEBUG_MODE)
        Serial.println("start advertising");
#endif
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected)
    {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}