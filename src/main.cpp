#include <Arduino.h>
#include <config.h>

#include <WiFi.h>
#include <ESP32Servo.h>
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
//  #include <TASK_HMI_Serial.h>

String local_IP;
ExternalEEPROM I2C_EEPROM;
ESP32PWM pwm_heat;
ESP32PWM pwm_fan;

WebServer server(80);
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
byte tries;
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

// Handle root url (/)
void handle_root()
{
    char index_html[2048];
    String ver = VERSION;
    snprintf(index_html, 2048,
             "<html>\
<head>\
<title>MATCH BOX SETUP</title>\
    </head> \
    <body>\
        <main>\
        <h1 align='center'>BLE version:%s</h1>\
        <div align='center'><a href='/update' target='_blank'>FIRMWARE UPDATE</a>\
        </main>\
        </div>\
    </body>\
</html>\
",
             ver);
    server.send(200, "text/html", index_html);
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
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    // ESP32PWM::allocateTimer(2);
    // ESP32PWM::allocateTimer(3);

    // read pid data from EEPROM
#if defined(DEBUG_MODE)
    Serial.begin(BAUDRATE);

    // start Serial

    Serial.printf("\nStart Task...");
#endif
    aht20.begin();
    MCP.NewConversion(); // New conversion is initiated

#if defined(DEBUG_MODE)
    Serial.printf("\nStart PWM...");
#endif
    pwm_heat.attachPin(pwm_heat_out, frequency, resolution); // 1KHz 8 bit
    pwm_fan.attachPin(pwm_fan_out, frequency, resolution);   // 1KHz 8 bit
    pwm_heat.writeScaled(0.0);
    pwm_fan.writeScaled(0.3);
    // 初始化网络服务
    WiFi.macAddress(macAddr);
    // WiFi.mode(WIFI_AP);
    sprintf(ap_name, "MATCHBOX_%02X%02X%02X", macAddr[3], macAddr[4], macAddr[5]);
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

    // init PID
    Heat_pid_controller.begin(&BT_TEMP, &PID_output, &pid_sv, pid_parm.p, pid_parm.i, pid_parm.d);
    Heat_pid_controller.setSampleTime(pid_parm.pid_CT * 1000); // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
    Heat_pid_controller.setOutputLimits(round(PID_MIN_OUT * 255 / 100), round(PID_MAX_OUT * 255 / 100));
    Heat_pid_controller.setBias(255.0 / 2.0);
    Heat_pid_controller.setWindUpLimits(2, 2); // Groth bounds for the integral term to prevent integral wind-up
    Heat_pid_controller.start();

    // Start ElegantOTA
    server.on("/", handle_root);
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
    // mb.task();
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