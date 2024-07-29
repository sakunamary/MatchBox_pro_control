#include <Arduino.h>
#include "config.h"

#include <WiFi.h>
#include <pwmWrite.h>
#include <StringTokenizer.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>
// #include <cmndreader.h>
// #include <pidautotuner.h>
#include "SparkFun_External_EEPROM.h" // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM
#include "ArduPID.h"
#include <TASK_read_temp.h>
#include <TASK_BLE_Serial.h>
// #include <TASK_modbus_handle.h>
#include <TASK_HMI_Serial.h>

WebServer server(80);
String local_IP;
ExternalEEPROM I2C_EEPROM;
Pwm pwm = Pwm();
ArduPID Heat_pid_controller;

extern bool loopTaskWDTEnabled;
extern TaskHandle_t loopTaskHandle;

int levelOT1 = 0;
int levelIO3 = 30;
bool pid_status = false;

double PID_output = 0;
double pid_sv;
double pid_tune_output;

const uint32_t frequency = PWM_FREQ;
const byte resolution = PWM_RESOLUTION;
const byte pwm_fan_out = PWM_FAN;
const byte pwm_heat_out = PWM_HEAT;

char ap_name[16];
uint8_t macAddr[6];
byte tries;

pid_setting_t pid_parm = {
    .pid_CT = 2,       // uint16_t pid_CT;
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
    Serial.println("OTA update started!");
    // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final)
{
    // Log every 1 second
    if (millis() - ota_progress_millis > 1000)
    {
        ota_progress_millis = millis();
        Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
    }
}

void onOTAEnd(bool success)
{
    // Log when OTA has finished
    if (success)
    {
        Serial.println("OTA update finished successfully!");
    }
    else
    {
        Serial.println("There was an error during OTA update!");
    }
    // <Add your own code here>
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

    loopTaskWDTEnabled = true;
    xThermoDataMutex = xSemaphoreCreateMutex();
    xSerialReadBufferMutex = xSemaphoreCreateMutex();

    Serial.begin(BAUDRATE);
    // Serial_HMI.setBuffer();
    Serial_HMI.begin(BAUDRATE, SERIAL_8N1, RXD_HMI, TXD_HMI);

#if defined(DEBUG_MODE)

    // start Serial

    Serial.printf("\nStart Task...");
#endif
    aht20.begin();
    MCP.NewConversion(); // New conversion is initiated

    pwm.pause();
    pwm.write(pwm_fan_out, 800, frequency, resolution);
    pwm.write(pwm_heat_out, 0, frequency, resolution);
    pwm.resume();
    // pwm.printDebug();

    // #if defined(DEBUG_MODE)
    //     Serial.printf("\nStart PWM...");
    // #endif

    // 初始化网络服务
    WiFi.macAddress(macAddr);
    // WiFi.mode(WIFI_AP);
    sprintf(ap_name, "MATCHBOX_%02X%02X%02X", macAddr[3], macAddr[4], macAddr[5]);
#
    while (WiFi.status() != WL_CONNECTED)
    {

        delay(1000);
        Serial.println("wifi not ready");

        if (tries++ > 2)
        {
            // init wifi
            Serial.println("WiFi.mode(AP):");
            WiFi.mode(WIFI_AP);
            WiFi.softAP(ap_name, "88888888"); // defualt IP address :192.168.4.1 password min 8 digis
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
        NULL // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK1:Task_Thermo_get_data...");
#endif

    //     xTaskCreate(
    //         Task_modbus_handle, "modbus_handle" // 获取HB数据
    //         ,
    //         1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
    //         ,
    //         NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    //         ,
    //         NULL // Running Core decided by FreeRTOS,let core0 run wifi and BT
    //     );
    // #if defined(DEBUG_MODE)
    //     Serial.printf("\nTASK2:Task_modbus_handle...");
    // #endif

    //     xTaskCreate(
    //         TASK_data_to_HMI, "TASK_data_to_HMI" // 获取HB数据
    //         ,
    //         1024 * 2 // This stack size can be checked & adjusted by reading the Stack Highwater
    //         ,
    //         NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    //         ,
    //         &xTASK_data_to_HMI // Running Core decided by FreeRTOS,let core0 run wifi and BT
    //     );
    // #if defined(DEBUG_MODE)
    //     Serial.printf("\nTASK3:TASK_data_to_HMI...\n");
    // #endif

    //     xTaskCreate(
    //         TASK_CMD_FROM_HMI, "TASK_CMD_FROM_HMI" // 获取HB数据
    //         ,
    //         1024 * 2 // This stack size can be checked & adjusted by reading the Stack Highwater
    //         ,
    //         NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    //         ,
    //         &xTASK_CMD_HMI // Running Core decided by FreeRTOS,let core0 run wifi and BT
    //     );
    // #if defined(DEBUG_MODE)
    //     Serial.printf("\nTASK4:TASK_CMD_FROM_HMI...\n");
    // #endif

    //     xTaskCreate(
    //         TASK_HMI_CMD_handle, "TASK_HMI_CMD_handle" // 获取HB数据
    //         ,
    //         1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
    //         ,
    //         NULL, 1 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    //         ,
    //         &xTASK_HMI_CMD_handle // Running Core decided by FreeRTOS,let core0 run wifi and BT
    //     );
    // #if defined(DEBUG_MODE)
    //     Serial.printf("\nTASK5:TASK_HMI_CMD_handle...\n");
    // #endif

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
        NULL, 1 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        &xTASK_BLE_CMD_handle // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK8:TASK_BLE_CMD_handle...\n");
#endif

    // Init Modbus-TCP
    // #if defined(DEBUG_MODE)
    //     Serial.printf("\nStart Modbus-TCP   service...");
    // #endif
    //     mb.server(502); // Start Modbus IP //default port :502
    //     mb.addHreg(BT_HREG);
    //     mb.addHreg(ET_HREG);
    //     mb.addHreg(HEAT_HREG);
    //     mb.addHreg(FAN_HREG);
    //     mb.addHreg(AMB_RH_HREG);
    //     mb.addHreg(AMB_TEMP_HREG);

    //     mb.addHreg(PID_STATUS_HREG);
    //     mb.addHreg(PID_SV_HREG);

    //     mb.Hreg(BT_HREG, 0);         // 初始化赋值
    //     mb.Hreg(ET_HREG, 0);         // 初始化赋值
    //     mb.Hreg(HEAT_HREG, 0);       // 初始化赋值
    //     mb.Hreg(FAN_HREG, 10);       // 初始化赋值
    //     mb.Hreg(PID_STATUS_HREG, 0); // 初始化赋值
    //     mb.Hreg(PID_SV_HREG, 0);     // 初始化赋值

    //     mb.Hreg(AMB_RH_HREG, 0);   // 初始化赋值
    //     mb.Hreg(AMB_TEMP_HREG, 0); // 初始化赋值

    // init PID
    Heat_pid_controller.begin(&BT_TEMP, &PID_output, &pid_sv, pid_parm.p, pid_parm.i, pid_parm.d);
    Heat_pid_controller.setSampleTime(pid_parm.pid_CT * 1000); // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
    Heat_pid_controller.setOutputLimits(round(PID_MIN_OUT * 255 / 100), round(PID_MAX_OUT * 255 / 100));
    Heat_pid_controller.setBias(255.0 / 2.0);
    Heat_pid_controller.setWindUpLimits(2, 2); // Groth bounds for the integral term to prevent integral wind-up
    Heat_pid_controller.start();

    // INIT PID AUTOTUNE

    // tuner.setTargetInputValue(180.0);
    // tuner.setLoopInterval(pid_parm.pid_CT * uS_TO_S_FACTOR);
    // tuner.setOutputRange(round(PID_MIN_OUT * 255 / 100), round(PID_MAX_OUT * 255 / 100));
    // tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);

    server.on("/", []()
              { server.send(200, "text/plain", "Hi! This is ElegantOTA Demo."); });

    ElegantOTA.begin(&server); // Start ElegantOTA
    // ElegantOTA callbacks
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onProgress(onOTAProgress);
    ElegantOTA.onEnd(onOTAEnd);

    server.begin();
    Serial.println("HTTP server started");
}

void loop()
{
    server.handleClient();
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