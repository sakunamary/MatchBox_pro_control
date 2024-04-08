#ifndef __TASK_READ_TEMP_H__
#define __TASK_READ_TEMP_H__

#include <Arduino.h>
#include <config.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include <ModbusIP_ESP8266.h>

ModbusIP mb; // declear object

double BT_TEMP;
double ET_TEMP;

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo_BT = Adafruit_MAX31865(SPI_CS_BT, SPI_MOSI, SPI_MISO, SPI_SCK);
Adafruit_MAX31865 thermo_ET = Adafruit_MAX31865(SPI_CS_ET, SPI_MOSI, SPI_MISO, SPI_SCK);

// Modbus Registers Offsets
const uint16_t BT_HREG = 3001;
const uint16_t ET_HREG = 3002;

extern pid_setting_t pid_parm;

void Task_Thermo_get_data(void *pvParameters)
{ // function

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;
    uint8_t TEMP_DATA_Buffer[HMI_BUFFER_SIZE];
    const TickType_t xIntervel = pid_parm.pid_CT / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    for (;;) // A Task shall never return or exit.
    {        // for loop
        // Wait for the next cycle (intervel 1500ms).
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
        {                                                          // lock the  mutex
            // 读取max6675数据
            BT_TEMP = round(((thermo_BT.temperature(RNOMINAL, RREF) + pid_parm.BT_tempfix) * 10) / 10);
            vTaskDelay(20);
            ET_TEMP = round(((thermo_ET.temperature(RNOMINAL, RREF) + pid_parm.ET_tempfix) * 10) / 10);
            vTaskDelay(20);

            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }

        // update  Hreg data
        mb.Hreg(BT_HREG, int(round(BT_TEMP * 10))); // 初始化赋值
        mb.Hreg(ET_HREG, int(round(ET_TEMP * 10))); // 初始化赋值                                        // 封装HMI 协议
        make_frame_head(TEMP_DATA_Buffer, 1);
        make_frame_end(TEMP_DATA_Buffer, 1);
        make_frame_data(TEMP_DATA_Buffer, 1, int(round(BT_TEMP * 10)), 3);
        make_frame_data(TEMP_DATA_Buffer, 1, int(round(ET_TEMP * 10)), 5);
        xQueueSend(queue_data_to_HMI, &TEMP_DATA_Buffer, xIntervel / 3);
        xTaskNotify(xTASK_data_to_HMI, 0, eIncrement);
        // 封装BLE 协议
        // xQueueSend(queue_data_to_BLE, &TEMP_DATA_Buffer, xIntervel / 3);
        // xTaskNotify(xTASK_data_to_BLE, 0, eIncrement);        // send notify to TASK_data_to_HMI
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