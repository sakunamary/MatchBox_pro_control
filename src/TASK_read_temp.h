#ifndef __TASK_READ_TEM_H__
#define __TASK_READ_TEMP_H__

#include <Arduino.h>
#include <config.h>
#include <Wire.h>
#include <MCP3424.h>
#include "DFRobot_BME280.h"

double BT_TEMP;
double ET_TEMP;
double AMB_RH;
double AMB_TEMP;
uint32_t AMB_PRESS;
extern int levelOT1;
extern int levelIO3;
extern double pid_sv;
extern bool pid_status;

// Need this for the lower level access to set them up.
uint8_t address = 0x68;
long Voltage; // Array used to store results

typedef DFRobot_BME280_IIC BME; // ******** use abbreviations instead of full names ********

/**IIC address is 0x77 when pin SDO is high */
/**IIC address is 0x76 when pin SDO is low */
BME bme(&Wire, 0x76); // select TwoWire peripheral and set sensor address

#define SEA_LEVEL_PRESSURE 1015.0f

#define R0 100
#define Rref 1000

MCP3424 MCP(address); // Declaration of MCP3424 A2=0 A1=1 A0=0
// DFRobot_AHT20 aht20;
//  TypeK temp_K_cal;

extern pid_setting_t pid_parm;
extern HardwareSerial Serial_HMI;

void Task_Thermo_get_data(void *pvParameters)
{ // function

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;
    char temp_data_buffer_ble[BLE_BUFFER_SIZE];
    uint8_t temp_data_buffer_hmi[HMI_BUFFER_SIZE];
    const TickType_t xIntervel = (pid_parm.pid_CT * 1000) / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    for (;;) // A Task shall never return or exit.
    {        // for loop
        // Wait for the next cycle (intervel 1500ms).
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
        {

            // if (aht20.startMeasurementReady(/* crcEn = */ true))
            // {
            AMB_TEMP = bme.getTemperature();
            AMB_PRESS = bme.getPressure();
            AMB_RH = bme.getHumidity();
            // #if defined(DEBUG_MODE)
            //                 Serial.printf("raw data:AMB_TEMP:%4.2f\n", AMB_TEMP);
            // #endif
            // }
            delay(200);
            MCP.Configuration(1, 16, 1, 1); // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
            Voltage = MCP.Measure();        // Measure is stocked in array Voltage, note that the library will wait for a completed conversion that takes around 200 ms@18bits            BT_TEMP = pid_parm.BT_tempfix + (((Voltage / 1000 * Rref) / ((3.3 * 1000) - Voltage / 1000) - R0) / (R0 * 0.0039083));
            BT_TEMP = pid_parm.BT_tempfix + (((Voltage / 1000 * Rref) / ((3.3 * 1000) - Voltage / 1000) - R0) / (R0 * 0.0039083));
            delay(200);
            MCP.Configuration(2, 16, 1, 1); // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
            Voltage = MCP.Measure();        // Measure is stocked in array Voltage, note that the library will wait for a completed conversion that takes around 200 ms@18bits
            ET_TEMP = pid_parm.ET_tempfix + (((Voltage / 1000 * Rref) / ((3.3 * 1000) - Voltage / 1000) - R0) / (R0 * 0.0039083));

            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }
        // PID ON:ambient,chan1,chan2,  heater duty, fan duty, SV
        if (xSemaphoreTake(xDATA_OUT_Mutex, 250 / portTICK_PERIOD_MS) == pdPASS) // 给温度数组的最后一个数值写入数据
        {
            // 封装BLE 协议
            sprintf(temp_data_buffer_ble, "#%4.2f,%4.2f,%4.2f,%d,%d,%4.2f;\n", AMB_TEMP, ET_TEMP, BT_TEMP, levelOT1, levelIO3, pid_sv);
            //Serial.print(temp_data_buffer_ble);

            xQueueSend(queue_data_to_BLE, &temp_data_buffer_ble, xIntervel);

            // 封装HMI 协议
            temp_data_buffer_hmi[0] = 0x69;
            temp_data_buffer_hmi[1] = 0xff;
            temp_data_buffer_hmi[2] = 0x01;
            temp_data_buffer_hmi[3] = lowByte(int(round(AMB_TEMP * 10)));
            temp_data_buffer_hmi[4] = highByte(int(round(AMB_TEMP * 10)));
            temp_data_buffer_hmi[5] = lowByte(int(round(ET_TEMP * 10)));
            temp_data_buffer_hmi[6] = highByte(int(round(ET_TEMP * 10)));
            temp_data_buffer_hmi[7] = lowByte(int(round(BT_TEMP * 10)));
            temp_data_buffer_hmi[8] = highByte(int(round(BT_TEMP * 10)));
            temp_data_buffer_hmi[9] = lowByte(int(round(pid_sv * 10)));
            temp_data_buffer_hmi[10] = highByte(int(round(pid_sv * 10)));
            temp_data_buffer_hmi[11] = levelOT1;
            temp_data_buffer_hmi[12] = levelIO3;
            if (pid_status)
            {
                temp_data_buffer_hmi[13] = 0x01;
            }
            else
            {
                temp_data_buffer_hmi[13] = 0x00;
            }
            temp_data_buffer_hmi[14] = 0xff;
            temp_data_buffer_hmi[15] = 0xff;
            temp_data_buffer_hmi[16] = 0xff;

            xQueueSend(queue_data_to_HMI, &temp_data_buffer_hmi, xIntervel);

            memset(temp_data_buffer_hmi, '\0', HMI_BUFFER_SIZE);

            xSemaphoreGive(xDATA_OUT_Mutex); // end of lock mutex
        }
        xTaskNotify(xTASK_data_to_BLE, 0, eIncrement); // send notify to TASK_data_to_HMI
        xTaskNotify(xTASK_data_to_HMI, 0, eIncrement); // send notify to TASK_data_to_HMIÍ
    }

} // function

#endif

// printh 00 00 00 ff ff ff 88 ff ff ff//输出上电信息到串口
// 69 ff 00 ff ff ff 69 69 69 67 67 67 ff ff ff ff ff //握手协议

// HMI --> MatchBox的数据帧 FrameLenght = 17
// 帧头: 69 FF
// 类型: 01 温度数据
// 环境温： 00 00 //uint16
// 温度1: 00 00 // uint16
// 温度2: 00 00 // uint16
// PID SV: 00 00 // uint16
// 火力 : 00
// 风力 : 00
// PID RUN: 00
// 帧尾:FF FF FF

// HMI --> MatchBox的数据帧 FrameLenght = 17
// 帧头: 69 FF
// 类型: 02 PID设定
// BT fix: 00 00 // uint16
// ET fix: 00 00 // uint16
// P: 00 00 // uint16
// I: 00 00 // uint16
// D: 00 00 // uint16
// PID ct: 00
// 帧尾:FF FF FF