#ifndef __TASK_READ_TEM_H__
#define __TASK_READ_TEMP_H__

#include <Arduino.h>
#include <config.h>
#include <Wire.h>
#include <ModbusIP_ESP8266.h>
#include <MCP3424.h>
#include "DFRobot_AHT20.h"

ModbusIP mb; // declear object

double BT_TEMP;
double ET_TEMP;
double AMB_RH;
double AMB_TEMP;

// Need this for the lower level access to set them up.
uint8_t address = 0x68;
long Voltage; // Array used to store results

#define R0 100
#define Rref 1000

MCP3424 MCP(address); // Declaration of MCP3424 A2=0 A1=1 A0=0
DFRobot_AHT20 aht20;
// TypeK temp_K_cal;

// Modbus Registers Offsets
const uint16_t BT_HREG = 3001;
const uint16_t ET_HREG = 3002;
const uint16_t AMB_RH_HREG = 3007;
const uint16_t AMB_TEMP_HREG = 3008;

extern pid_setting_t pid_parm;

void Task_Thermo_get_data(void *pvParameters)
{ // function

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;
    char   temp_data_buffer_ble[BLE_BUFFER_SIZE];
    uint8_t TEMP_DATA_Buffer[HMI_BUFFER_SIZE];
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

            if (aht20.startMeasurementReady(/* crcEn = */ true))
            {
                AMB_TEMP = aht20.getTemperature_C();
                AMB_RH = aht20.getHumidity_RH();
                Serial.printf("raw data:AMB_TEMP:%4.2f\n",AMB_TEMP);
            }
            vTaskDelay(200);
            MCP.Configuration(1, 16, 1, 1); // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
            Voltage = MCP.Measure();        // Measure is stocked in array Voltage, note that the library will wait for a completed conversion that takes around 200 ms@18bits
            BT_TEMP = pid_parm.BT_tempfix + (((Voltage / 1000 * Rref) / ((3.3 * 1000) - Voltage / 1000) - R0) / (R0 * 0.0039083));
            vTaskDelay(200);
            MCP.Configuration(2, 16, 1, 1); // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
            Voltage = MCP.Measure();        // Measure is stocked in array Voltage, note that the library will wait for a completed conversion that takes around 200 ms@18bits
            ET_TEMP = pid_parm.ET_tempfix + (((Voltage / 1000 * Rref) / ((3.3 * 1000) - Voltage / 1000) - R0) / (R0 * 0.0039083));

            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }

        // update  Hreg data
        mb.Hreg(BT_HREG, int(round(BT_TEMP * 10)));        // 初始化赋值
        mb.Hreg(ET_HREG, int(round(ET_TEMP * 10)));        // 初始化赋值
        mb.Hreg(AMB_RH_HREG, int(round(AMB_RH * 10)));     // 初始化赋值
        mb.Hreg(AMB_TEMP_HREG, int(round(AMB_TEMP * 10))); // 初始化赋值

        // // 封装HMI 协议
        // make_frame_head(TEMP_DATA_Buffer, 1);
        // make_frame_end(TEMP_DATA_Buffer, 1);
        // make_frame_data(TEMP_DATA_Buffer, 1, int(round(BT_TEMP * 10)), 3);
        // make_frame_data(TEMP_DATA_Buffer, 1, int(round(ET_TEMP * 10)), 5);
        // xQueueSend(queue_data_to_HMI, &TEMP_DATA_Buffer, xIntervel / 3);
        // xTaskNotify(xTASK_data_to_HMI, 0, eIncrement);
        // 封装BLE 协议
        // ambient,chan1,chan2,chan3,chan4

        sprintf(temp_data_buffer_ble, "%4.2f,%4.2f,%4.2f,0.00\n\r", AMB_TEMP, BT_TEMP, ET_TEMP);
        Serial.println(temp_data_buffer_ble);
        xQueueSend(queue_data_to_BLE, &temp_data_buffer_ble, xIntervel / 3);
        xTaskNotify(xTASK_data_to_BLE, 0, eIncrement); // send notify to TASK_data_to_HMI
    }

} // function

#endif

// HMI --> MatchBox的数据帧 FrameLenght = 12
// 帧头: 69 FF
// 类型: 01 温度数据
// 温度1: 00 00 // uint16
// 温度2: 00 00 // uint16
// 火力 : 00
// 风力 : 00
// 帧尾:FF FF FF

// HMI --> MatchBox的数据帧 FrameLenght = 12
// 帧头: 69 FF
// 类型: 02 PID设定
// P: 00 00 // uint16
// I: 00 00 // uint16
// D: 00 00 // uint16
// 帧尾:FF FF FF

// HMI --> MatchBox的数据帧 FrameLenght = 12
// 帧头: 69 FF
// 类型: 03 参数设定
// PID ct: 00 00 // uint16
// BT fix: 00 00 // uint16
// ET fix: 00 00 // uint16
// 帧尾:FF FF FF

// HMI --> MatchBox的数据帧 FrameLenght = 12
// 帧头: 69 FF
// 类型: 04 PID run
// PID SV: 00 00 // uint16
// PID STATUS: 00
// NULL :00 00 00
// 帧尾:FF FF FF