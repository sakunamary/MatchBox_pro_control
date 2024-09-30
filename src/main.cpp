#include <Arduino.h>
#include <config.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <ESP32Servo.h>
#include <StringTokenizer.h>
#include <WiFiClient.h>
#include <WebServer.h>
// #include <ElegantOTA.h>
#include "SparkFun_External_EEPROM.h" // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM
#include <PID_v1.h>
#include <TASK_read_temp.h>
#include "TASK_modbus_control.h"
String local_IP;
ExternalEEPROM I2C_EEPROM;
ESP32PWM pwm_heat;
ESP32PWM pwm_fan;

WebServer server(80);
PID Heat_pid_controller(&BT_TEMP, &PID_output, &pid_sv, pid_parm.p, pid_parm.i, pid_parm.d, DIRECT);

extern bool loopTaskWDTEnabled;
extern TaskHandle_t loopTaskHandle;

byte tries;
char ap_name[16];
uint8_t macAddr[6];

extern const byte pwm_fan_out;
extern const byte pwm_heat_out;

pid_setting_t pid_parm = {
    .pid_CT = 1.5,     // uint16_t pid_CT;
    .p = 2.0,          // double p ;
    .i = 0.12,         // double i ;
    .d = 5.0,          // double d ;
    .BT_tempfix = 0.0, // uint16_t BT_tempfix;
    .ET_tempfix = 0.0  // uint16_t ET_tempfix;
};

// unsigned long ota_progress_millis = 0;

// void onOTAStart()
// {
//     // Log when OTA has started
//     // Serial.println("OTA update started!");
//     // <Add your own code here>
// }

// void onOTAProgress(size_t current, size_t final)
// {
//     // Log every 1 second
//     if (millis() - ota_progress_millis > 1000)
//     {
//         ota_progress_millis = millis();
//         // Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
//     }
// }

// void onOTAEnd(bool success)
// {
//     // Log when OTA has finished
//     if (success)
//     {
//         // Serial.println("OTA update finished successfully!");
//     }
//     else
//     {
//         // Serial.println("There was an error during OTA update!");
//     }
//     // <Add your own code here>
// }

// // Handle root url (/)
// void handle_root()
// {
//     char index_html[2048];
//     String ver = VERSION;
//     snprintf(index_html, 2048,
//              "<html>\
// <head>\
// <title>MATCH BOX MINI SETUP</title>\
//     </head> \
//     <body>\
//         <main>\
//         <h1 align='center'>BLE version:%s</h1>\
//         <div align='center'><a href='/update' target='_blank'>FIRMWARE UPDATE</a>\
//         </main>\
//         </div>\
//     </body>\
// </html>\
// ",
//              ver);
//     server.send(200, "text/html", index_html);
// }

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

    pwm_fan.attachPin(pwm_fan_out, frequency, resolution); // 1KHz 8 bit
    pwm_fan.write(300);

    pwm_heat.attachPin(pwm_heat_out, frequency, resolution); // 1KHz 8 bit
    pwm_heat.write(1);

    Serial.begin(BAUDRATE);
    // read pid data from EEPROM
#if defined(DEBUG_MODE)
    // start Serial
    Serial.printf("\nStart Task...\n");
#endif
    aht20.begin();
    MCP.NewConversion(); // New conversion is initiated
    I2C_EEPROM.setMemoryType(64);

#if defined(DEBUG_MODE)
    Serial.println("\nstart Reading EEPROM setting ...\n");
#endif
    if (!I2C_EEPROM.begin())
    {
        Serial.println("\nfailed to initialise EEPROM\n");
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
#if defined(DEBUG_MODE)
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
#endif

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
    Serial.printf("\nTASK1:Task_Thermo_get_data...\n");
#endif

    xTaskCreate(
        Task_modbus_control, "modbus_control" //
        ,
        1024 * 10 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        &xTask_modbus_control // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK=2:modbus_control OK\n");
#endif

    xTaskCreate(
        Task_PID_PARM_SETTING, "Task_PID_PARM_SETTING" //
        ,
        1024 * 10 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK=3:Task_PID_PARM_SETTING OK\n");
#endif

    // INIT MODBUS

    mb.server(502); // Start Modbus IP //default port :502

#if defined(DEBUG_MODE)
    Serial.printf("\nStart Modbus-TCP  service OK\n");
#endif
    // Add SENSOR_IREG register - Use addIreg() for analog Inputs
    mb.addHreg(BT_HREG);

    mb.addHreg(HEAT_HREG);
    mb.addHreg(FAN_HREG);

    mb.addHreg(PID_SV_HREG);
    mb.addHreg(PID_STATUS_HREG);
    mb.addHreg(PID_P_HREG);
    mb.addHreg(PID_I_HREG);
    mb.addHreg(PID_D_HREG);

    mb.addHreg(AMB_RH_HREG);
    mb.addHreg(AMB_TEMP_HREG);

    // INIT MODBUS HREG VALUE
    mb.Hreg(BT_HREG, 0); // 初始化赋值

    mb.Hreg(HEAT_HREG, 0); // 初始化赋值
    mb.Hreg(FAN_HREG, 0);  // 初始化赋值

    mb.Hreg(PID_SV_HREG, 0);     // 初始化赋值
    mb.Hreg(PID_STATUS_HREG, 0); // 初始化赋值

    mb.Hreg(PID_P_HREG, 200);
    mb.Hreg(PID_I_HREG, 12);
    mb.Hreg(PID_D_HREG, 500);

    mb.Hreg(AMB_RH_HREG, 0);   // 初始化赋值
    mb.Hreg(AMB_TEMP_HREG, 0); // 初始化赋值

    // init PID

    Heat_pid_controller.SetMode(MANUAL);
    Heat_pid_controller.SetOutputLimits(PID_MIN_OUT, PID_MAX_OUT);
    Heat_pid_controller.SetSampleTime(int(pid_parm.pid_CT * 1000));

    // // Start ElegantOTA
    // server.on("/", handle_root);
    // ElegantOTA.begin(&server); // Start ElegantOTA
    // // ElegantOTA callbacks
    // ElegantOTA.onStart(onOTAStart);
    // ElegantOTA.onProgress(onOTAProgress);
    // ElegantOTA.onEnd(onOTAEnd);

    // server.begin();
// #if defined(DEBUG_MODE)
//     Serial.println("HTTP server started");
// #endif
#if defined(DEBUG_MODE)
    Serial.printf("\nEEPROM value check ...\n");
    Serial.printf("pid_CT:%4.2f\n", pid_parm.pid_CT);
    Serial.printf("PID kp:%4.2f\n", pid_parm.p);
    Serial.printf("PID ki:%4.2f\n", pid_parm.i);
    Serial.printf("PID kd:%4.2f\n", pid_parm.d);
    Serial.printf("BT fix:%4.2f\n", pid_parm.BT_tempfix);
    Serial.printf("ET fix:%4.2f\n", pid_parm.ET_tempfix);
#endif

    // init watchdog
    //  If the TWDT was not initialized automatically on startup, manually intialize it now
    esp_task_wdt_init(3, true);
#if defined(DEBUG_MODE)
    Serial.println("\nWDT started\n");
#endif
}

void loop()
{

    mb.task();
    //     server.handleClient();
    //     ElegantOTA.loop();
    //     // disconnecting
    //     if (!deviceConnected && oldDeviceConnected)
    //     {
    //         delay(500);                  // give the bluetooth stack the chance to get things ready
    //         pServer->startAdvertising(); // restart advertising
    // #if defined(DEBUG_MODE)
    //         Serial.println("start advertising");
    // #endif
    //         oldDeviceConnected = deviceConnected;
    //     }
    //     // connecting
    //     if (deviceConnected && !oldDeviceConnected)
    //     {
    //         // do stuff here on connecting
    //         oldDeviceConnected = deviceConnected;
    //     }

    // #if !CONFIG_ESP_TASK_WDT_INIT
    //     // If we manually initialized the TWDT, deintialize it now
    // esp_task_wdt_deinit();

    // #endif // CONFIG_ESP_TASK_WDT_INIT
}