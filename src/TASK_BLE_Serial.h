
#ifndef __TASK_BLE_SERIAL_H__
#define __TASK_BLE_SERIAL_H__
#include <Arduino.h>
#include <config.h>
#include <esp_attr.h>
#include <esp_task_wdt.h>
#include <driver/rtc_io.h>
#include "soc/rtc_wdt.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

CmndInterp ci(DELIM); // command interpreter object

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
extern char ap_name[16];

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
    }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string rxValue = pCharacteristic->getValue();
        uint8_t BLE_DATA_Buffer[BLE_BUFFER_SIZE];
        int i = 0;
        while (i < rxValue.length() && rxValue.length() > 0)
        {
            Serial.print(rxValue[i]);
            if (rxValue[i] == 0x0A)
            {
                BLE_DATA_Buffer[i] = rxValue[i];                  // copy value
                xQueueSend(queueCMD_BLE, &BLE_DATA_Buffer, 100);  // 串口数据发送至队列
                xTaskNotify(xTASK_BLE_CMD_handle, 0, eIncrement); // 通知处理任务干活
                memset(&BLE_DATA_Buffer, '\0', BLE_BUFFER_SIZE);
                i = 0; // clearing
                break; // 跳出循环
            }
            else
            {
                BLE_DATA_Buffer[i] = rxValue[i];
                i++;
            }
        }
        vTaskDelay(50);
    }
};

// const int BLE_BUFFER_SIZE = 1024;

void TASK_DATA_to_BLE(void *pvParameters)
{
    (void)pvParameters;
    uint8_t BLE_DATA_Buffer[BLE_BUFFER_SIZE];
    const TickType_t timeOut = 150;
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
              // #if defined(DEBUG_MODE)
                Serial.println(String((char *)BLE_DATA_Buffer));
                // #endif
                if (deviceConnected)
                {
                    pTxCharacteristic->setValue(BLE_DATA_Buffer, sizeof(BLE_DATA_Buffer));
                    pTxCharacteristic->notify();
                }
                // data frame:PID ON:ambient,chan1,chan2,  heater duty, fan duty, SV
                vTaskDelay(50);
            }
        }
    }
}


void TASK_BLE_CMD_handle(void *pvParameters)
{
    (void)pvParameters;
    uint8_t BLE_CMD_Buffer[BLE_BUFFER_SIZE];
    const TickType_t timeOut = 150;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;
    TickType_t xLastWakeTime;
    const TickType_t xIntervel = 150 / portTICK_PERIOD_MS;
    uint16_t temp_pwr = 0;
    int i;
    String TC4_data_String;
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

                if (xSemaphoreTake(xSerialReadBufferMutex, xIntervel) == pdPASS)
                {
                    TC4_data_String = String((char *)BLE_CMD_Buffer);
                    // #if defined(DEBUG_MODE)
                    // Serial.println(TC4_data_String);
                    // #endif
                    ci.checkCmnd(TC4_data_String);
                    xSemaphoreGive(xSerialReadBufferMutex);
                }

                // if (!TC4_data_String.startsWith("#"))
                // { //
                //   // StringTokenizer TC4_Data(TC4_data_String, ",");
                //   // while (TC4_Data.hasNext())
                //   // {
                //   //     Data[i] = TC4_Data.nextToken().toDouble(); // prints the next token in the string
                //   //     i++;
                //   // }
                //   // PID ON:ambient,chan1,chan2,  heater duty, fan duty, SV
                //   // if ((mb.Hreg(PID_HREG) == 1) && (xSemaphoreTake(xserialReadBufferMutex, xIntervel) == pdPASS))
                //   // {
                //   //     mb.Hreg(HEAT_HREG, Data[3]); // 获取赋值
                //   // }
                //   // xSemaphoreGive(xSerialReadBufferMutex);
                //   //
                //   // i = 0;
                //     ci.checkCmnd(TC4_data_String);
                // }
                // else
                // {
                //     // TC4_data_String.replace("#DATA_OUT,", "");
                //     // ci.checkCmnd(TC4_data_String);
                // }

                vTaskDelay(50);
            }
        }
    }
}

#endif
