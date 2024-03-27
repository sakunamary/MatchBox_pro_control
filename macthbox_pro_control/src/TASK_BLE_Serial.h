
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

void TASK_CMD_From_BLE(void *pvParameters)
{
    const TickType_t timeOut = 1000;
    while (true)
    {
        if (SerialBLE.available())
        {
            auto count = SerialBLE.readBytes(bleReadBuffer, BUFFER_SIZE);

            memset(bleReadBuffer, '\0', sizeof(bleReadBuffer));
        }
        vTaskDelay(20);
    }
}

void TASK_DATA_to_BLE(void *pvParameters)
{
    (void)pvParameters;
    uint8_t Serial_DATA_Buffer[BUFFER_SIZE];
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
            if (xQueueReceive(queue_data_to_BLE, &Serial_DATA_Buffer, timeOut) == pdPASS)
            { // 从接收QueueCMD 接收指令
                SerialBLE.write(Serial_DATA_Buffer, BLE_BUFFER_SIZE);
                vTaskDelay(20);
            }
        }
    }

}



// //Task for reading Serial Port
// void ReadSerialTask(void *e) {
//   while (true) {
//     if (Serial.available()) {
//       auto count = Serial.readBytes(serialReadBuffer, BUFFER_SIZE);
//       SerialBT.write(serialReadBuffer, count);
//     }
//     delay(20);
//   }
// }

// //Task for reading BLE Serial
// void ReadBtTask(void *e) {
//   while (true) {
//     if (SerialBT.available()) {
//       auto count = SerialBT.readBytes(bleReadBuffer, BUFFER_SIZE);
//       Serial.write(bleReadBuffer, count);
//     }
//     delay(20);
//   }
// }


#endif