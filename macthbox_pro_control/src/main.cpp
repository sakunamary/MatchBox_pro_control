#include <Arduino.h>
#include <config.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <TASK_read_temp.h>
#include <TASK_BLE_Serial.h>
#include <TASK_HMI_Serial.h>
#include <TASK_modbus_handle.h>

String local_IP;

extern bool loopTaskWDTEnabled;
extern TaskHandle_t loopTaskHandle;

char ap_name[30];
uint8_t macAddr[6];

void setup()
{

    // Disable watchdog timers
    disableCore0WDT();
    disableLoopWDT();
    esp_task_wdt_delete(NULL);
    loopTaskWDTEnabled = true;

    xThermoDataMutex = xSemaphoreCreateMutex();
    xSerialReadBufferMutex = xSemaphoreCreateMutex();

    pinMode(PWM_HEAT, OUTPUT);
    pinMode(PWM_FAN, OUTPUT);
    // PWM Pins
    ledcSetup(PWM_FAN_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWM_FAN, PWM_FAN_CHANNEL);

    ledcSetup(PWM_HEAT_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWM_HEAT, PWM_HEAT_CHANNEL);

    Serial.begin(BAUDRATE);
    Serial_HMI.begin(HMI_BAUDRATE, SERIAL_8N1, -1, -1);
    #if defined(DEBUG_MODE)
    Serial.printf("\nStart HMI serial...\n");
#endif
    thermo_BT.begin(MAX31865_2WIRE); // set to 2WIRE or 4WIRE as necessary
    thermo_ET.begin(MAX31865_2WIRE); // set to 2WIRE or 4WIRE as necessary
#if defined(DEBUG_MODE)
    Serial.printf("\nStart MAX31865...\n");
#endif
    // 初始化网络服务
    WiFi.macAddress(macAddr);
    WiFi.mode(WIFI_AP);
    sprintf(ap_name, "ROASTER_%02X%02X%02X", macAddr[3], macAddr[4], macAddr[5]);
    WiFi.softAP(ap_name, "12345678"); // defualt IP address :192.168.4.1 password min 8 digis
#if defined(DEBUG_MODE)
    Serial.printf("\nStart WIFI...\n");
#endif

    // Init BLE Serial
    SerialBLE.begin(ap_name, false, -1); // FOR ESP32C3 SuperMini board
    SerialBLE.setTimeout(10);
#if defined(DEBUG_MODE)
    Serial.printf("\nStart Task...\n");
#endif

    /*---------- Task Definition ---------------------*/
    // Setup tasks to run independently.
    xTaskCreate(
        Task_Thermo_get_data, "Thermo_get_data" // 获取HB数据
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
        Task_modbus_handle, "modbus_handle" // 获取HB数据
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

    xTaskCreate(
        TASK_DATA_to_BLE, "TASK_DATA_to_BLE" // 获取HB数据
        ,
        1024 * 2 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        &xTASK_data_to_BLE // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK3:TASK_DATA_to_BLE...\n");
#endif

    xTaskCreate(
        TASK_CMD_From_BLE, "TASK_CMD_From_BLE" // 获取HB数据
        ,
        1024 * 2 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        &xTASK_CMD_BLE // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK4:TASK_CMD_From_BLE...\n");
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
    Serial.printf("\nTASK5:TASK_BLE_CMD_handle...\n");
#endif

    xTaskCreate(
        TASK_data_to_HMI, "TASK_data_to_HMI" // 获取HB数据
        ,
        1024 * 2 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        &xTASK_data_to_HMI // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK6:TASK_data_to_HMI...\n");
#endif

    xTaskCreate(
        TASK_CMD_FROM_HMI, "TASK_CMD_FROM_HMI" // 获取HB数据
        ,
        1024 * 2 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        &xTASK_CMD_HMI // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK7:TASK_CMD_FROM_HMI...\n");
#endif

    xTaskCreate(
        TASK_HMI_CMD_handle, "TASK_HMI_CMD_handle" // 获取HB数据
        ,
        1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 1 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        &xTASK_HMI_CMD_handle // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK8:TASK_HMI_CMD_handle...\n");
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