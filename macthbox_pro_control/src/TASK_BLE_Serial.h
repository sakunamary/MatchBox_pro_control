
#ifndef __TASK_BLE_SERIAL_H__
#define __TASK_BLE_SERIAL_H__
#include <Arduino.h>
#include <config.h>
#include <BleSerial.h>
#include <esp_attr.h>
#include <esp_task_wdt.h>
#include <driver/rtc_io.h>
#include "soc/rtc_wdt.h"

BleSerial SerialBLE;

const int BLE_BUFFER_SIZE = 1024;



void TASK_DATA_to_BLE(void *pvParameters)
{
    (void)pvParameters;
    uint8_t BLE_DATA_Buffer[BLE_BUFFER_SIZE];
    const TickType_t timeOut = 500;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;

    while (1)
    {
        xResult = xTaskNotifyWait(0x00,                 // 在运行前这个命令之前，先清除这几位
                                  0x00,                 // 运行后，重置所有的bits 0x00 or ULONG_MAX or 0xFFFFFFFF
                                  &ulNotificationValue, // 重置前的notification value
                                  portMAX_DELAY);       // 一直等待
        if (xResult == pdTRUE)
        {
            if (xQueueReceive(queue_data_to_BLE, &BLE_DATA_Buffer, timeOut) == pdPASS)
            { // 从接收QueueCMD 接收指令
                SerialBLE.write(BLE_DATA_Buffer, BLE_BUFFER_SIZE);
                vTaskDelay(20);
            }
        }
    }
}

void TASK_CMD_From_BLE(void *pvParameters)
{
    (void)pvParameters;
    uint8_t BLE_ReadBuffer[BLE_BUFFER_SIZE];
    const TickType_t timeOut = 500;
    while (true)
    {
        if (SerialBLE.available())
        {
            auto count = SerialBLE.readBytes(BLE_ReadBuffer, BLE_BUFFER_SIZE);
            xQueueSend(queueCMD_BLE, &BLE_ReadBuffer, timeOut);   // 串口数据发送至队列
            xTaskNotify(xTASK_BLE_CMD_handle, 0, eIncrement); // 通知处理任务干活
        }
        vTaskDelay(20);
    }
}



void TASK_BLE_CMD_handle(void *pvParameters)
{
    (void)pvParameters;
    uint8_t BLE_CMD_Buffer[BUFFER_SIZE];
    const TickType_t timeOut = 1000;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;
    TickType_t xLastWakeTime;
    const TickType_t xIntervel = 500 / portTICK_PERIOD_MS;
    uint16_t temp_pwr = 0;
    while (1)
    {

        xResult = xTaskNotifyWait(0x00,                 // 在运行前这个命令之前，先清除这几位
                                  0x00,                 // 运行后，重置所有的bits 0x00 or ULONG_MAX or 0xFFFFFFFF
                                  &ulNotificationValue, // 重置前的notification value
                                  portMAX_DELAY);       // 一直等待
        if (xResult == pdTRUE)
        {
            if (xQueueReceive(queueCMD_BLE, &BLE_CMD_Buffer, timeOut) == pdPASS)
            { // 从接收QueueCMD 接收指令
                // // HMI_CMD_Buffer[5] //火力开关
                // if (BLE_CMD_Buffer[5] != digitalRead(HEAT_RLY))
                // {
                //     if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 整合数据帧到HMI
                //     {
                //         mb.Hreg(HEAT_HREG, HMI_CMD_Buffer[5]);
                //         digitalWrite(HEAT_RLY, !digitalRead(HEAT_RLY)); // 将artisan的控制值控制开关
                //     }
                //     xSemaphoreGive(xThermoDataMutex); // end of lock mutex
                // }
                // // HMI_CMD_Buffer[7] //冷却开关
                // if (HMI_CMD_Buffer[7] != digitalRead(FAN_RLY))
                // {
                //     if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 整合数据帧到HMI
                //     {
                //         mb.Hreg(FAN_HREG, HMI_CMD_Buffer[7]);
                //         digitalWrite(FAN_RLY, !digitalRead(FAN_RLY)); // 将artisan的控制值控制开关
                //     }
                //     xSemaphoreGive(xThermoDataMutex); // end of lock mutex
                // }
                // // HMI_CMD_Buffer[3]   //火力数据
                // if (HMI_CMD_Buffer[3] != last_PWR)
                // {
                //     last_PWR = HMI_CMD_Buffer[3];
                //     mb.Hreg(PWR_HREG, last_PWR);                                                 // last 火力pwr数据更新
                //     pwm_heat.write(HEAT_OUT_PIN, map(last_PWR, 0, 100, 230, 850), frequency, resolution); // 输出新火力pwr到SSRÍ
                // }
                vTaskDelay(20);
            }
        }
    }
}


#endif