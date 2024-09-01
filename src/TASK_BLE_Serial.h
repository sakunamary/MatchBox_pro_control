
#ifndef __TASK_BLE_SERIAL_H__
#define __TASK_BLE_SERIAL_H__
#include <Arduino.h>
#include <config.h>
#include <StringTokenizer.h>
#include <esp_attr.h>
#include <esp_task_wdt.h>
#include <driver/rtc_io.h>
#include "soc/rtc_wdt.h"

#include <ESP32Servo.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// #include "ArduPID.h"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;

extern ESP32PWM pwm_heat;
extern ESP32PWM pwm_fan;
extern ExternalEEPROM I2C_EEPROM;
// extern ArduPID Heat_pid_controller;
extern PID Heat_pid_controller;
bool deviceConnected = false;
bool oldDeviceConnected = false;
extern char ap_name[16];
String CMD_Data[6];

extern int levelOT1;
extern int levelIO3;
extern bool pid_status;

extern double PID_output;
extern double pid_sv;
extern double pid_tune_output;

extern const uint32_t frequency;
extern const byte resolution;
extern const byte pwm_fan_out;
extern const byte pwm_heat_out;

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
#if defined(DEBUG_MODE)
            //Serial.print(rxValue[i]);
#endif
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
        delay(50);
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
#if defined(DEBUG_MODE)
                Serial.println(String((char *)BLE_DATA_Buffer));
#endif
                if (deviceConnected)
                {
                    pTxCharacteristic->setValue(BLE_DATA_Buffer, sizeof(BLE_DATA_Buffer));
                    pTxCharacteristic->notify();
                }
                // data frame:PID ON:ambient,chan1,chan2,  heater duty, fan duty, SV
                delay(50);
            }
        }
    }
}

void TASK_BLE_CMD_handle(void *pvParameters)
{
    (void)pvParameters;
    uint8_t BLE_CMD_Buffer[BLE_BUFFER_SIZE];
    char BLE_data_buffer_char[BLE_BUFFER_SIZE];
    uint8_t BLE_data_buffer_uint8[BLE_BUFFER_SIZE];
    const TickType_t timeOut = 150;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;
    TickType_t xLastWakeTime;
    const TickType_t xIntervel = 150 / portTICK_PERIOD_MS;
    int i = 0;
    int j = 0;
    String TC4_data_String;
    String CMD_String;
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
                    // cmd from BLE cleaning
                    TC4_data_String = String((char *)BLE_CMD_Buffer);

                    while (j < TC4_data_String.length() && TC4_data_String.length() > 0)
                    {

                        if (TC4_data_String[j] == '\n')
                        {
                            CMD_String += TC4_data_String[j]; // copy value
                            TC4_data_String = "";
                            j = 0; // clearing
                            break; // 跳出循环
                        }
                        else
                        {
                            CMD_String += TC4_data_String[j]; // copy value
                            j++;
                        }
                    }
                    CMD_String.trim();
                    CMD_String.toUpperCase();
#if defined(DEBUG_MODE)
                    Serial.println(CMD_String); // for debug
#endif

                    // cmd from BLE cleaning
                    StringTokenizer BLE_CMD(CMD_String, ",");

                    while (BLE_CMD.hasNext())
                    {
                        CMD_Data[i] = BLE_CMD.nextToken(); // prints the next token in the string
                        // Serial.println(CMD_Data[i]);
                        i++;
                    }
                    i = 0;
                    CMD_String = "";
                    xSemaphoreGive(xSerialReadBufferMutex);
                }
                // big handle case switch
                if (CMD_Data[0] == "IO3")
                {
                    if (CMD_Data[1] == "UP")
                    {
                        levelIO3 = levelIO3 + DUTY_STEP;
                        if (levelIO3 > MAX_IO3)
                            levelIO3 = MAX_IO3; // don't allow OT1 to exceed maximum
                        if (levelIO3 < MIN_IO3)
                            levelIO3 = MIN_IO3; // don't allow OT1 to turn on less than minimum
                        pwm_fan.write(map(levelIO3, 0, 100, 230, 1000));
                        // #if defined(DEBUG_MODE)
                        //                         Serial.printf("FAN:%d\n", levelIO3);//for debug
                        // #endif
                        // sprintf(BLE_data_buffer_char, "#DATA_OUT,OT3,%d\n", levelIO3);
                        // // 格式转换
                        // memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
                        // if (deviceConnected)
                        // {
                        //     pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                        //     pTxCharacteristic->notify();
                        // }
                    }
                    else if (CMD_Data[1] == "DOWN")
                    {
                        levelIO3 = levelIO3 - DUTY_STEP;
                        if (levelIO3 < MIN_IO3 & levelIO3 != 0)
                            levelIO3 = 0; // turn ot1 off if trying to go below minimum. or use levelOT1 = MIN_HTR ?
                        pwm_fan.write(map(levelIO3, 0, 100, 230, 1000));
                        // #if defined(DEBUG_MODE)
                        //                         Serial.printf("FAN:%d\n", levelIO3);//for debug
                        // #endif
                        // sprintf(BLE_data_buffer_char, "#DATA_OUT,OT3,%d\n", levelIO3);
                        // memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
                        // if (deviceConnected)
                        // {
                        //     pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                        //     pTxCharacteristic->notify();
                        // }
                    }
                    else
                    {
                        uint8_t len = sizeof(CMD_Data[1]);
                        if (len > 0)
                        {
                            levelIO3 = CMD_Data[1].toInt();
                            if (levelIO3 > MAX_IO3)
                                levelIO3 = MAX_IO3; // don't allow OT1 to exceed maximum
                            if (levelIO3 < MIN_IO3 & levelIO3 != 0)
                                levelIO3 = MIN_IO3; // don't allow to set less than minimum unless setting to zero
                            pwm_fan.write(map(levelIO3, 0, 100, 230, 1000));
                            // #if defined(DEBUG_MODE)
                            //                             Serial.printf("FAN:%d\n", levelIO3);//for debug
                            // #endif
                            // sprintf(BLE_data_buffer_char, "#DATA_OUT,OT3,%d\n", levelIO3);
                            // memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
                            // // 格式转换
                            // if (deviceConnected)
                            // {
                            //     pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                            //     pTxCharacteristic->notify();
                            // }
                        }
                    }
                }
                else if (CMD_Data[0] == "OT1")
                {
                    if (CMD_Data[1] == "UP")
                    {

                        levelOT1 = levelOT1 + DUTY_STEP;
                        if (levelOT1 > MAX_OT1)
                            levelOT1 = MAX_OT1; // don't allow OT1 to exceed maximum
                        if (levelOT1 < MIN_OT1)
                            levelOT1 = MIN_OT1; // don't allow OT1 to turn on less than minimum
                        pwm_heat.write(map(levelOT1, 0, 100, 0, 1000));
                        // #if defined(DEBUG_MODE)
                        //                         Serial.printf("HEAT:%d\n", levelOT1);//for debug
                        // #endif
                        // sprintf(BLE_data_buffer_char, "#DATA_OUT,OT1,%d\n", levelOT1);
                        // // Serial.print(BLE_data_buffer_char);
                        // memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
                        // // 格式转换
                        // if (deviceConnected)
                        // {
                        //     pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                        //     pTxCharacteristic->notify();
                        // }
                    }
                    else if (CMD_Data[1] == "DOWN")
                    {
                        levelOT1 = levelOT1 - DUTY_STEP;
                        if (levelOT1 < MIN_OT1 & levelOT1 != 0)
                            levelOT1 = 0; // turn ot1 off if trying to go below minimum. or use levelOT1 = MIN_HTR ?
                        pwm_heat.write(map(levelOT1, 0, 100, 0, 1000));
                        // #if defined(DEBUG_MODE)
                        //                         Serial.printf("HEAT:%d\n", levelOT1);//for debug
                        // #endif
                        // sprintf(BLE_data_buffer_char, "#DATA_OUT,OT1,%d\n", levelOT1);
                        // // Serial.print(BLE_data_buffer_char);
                        // memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
                        // // 格式转换
                        // if (deviceConnected)
                        // {
                        //     pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                        //     pTxCharacteristic->notify();
                        // }
                    }
                    else
                    {
                        uint8_t len = sizeof(CMD_Data[1]);
                        if (len > 0)
                        {
                            levelOT1 = CMD_Data[1].toInt();
                            if (levelOT1 > MAX_OT1)
                                levelOT1 = MAX_OT1; // don't allow OT1 to exceed maximum
                            if (levelOT1 < MIN_OT1 & levelOT1 != 0)
                                levelOT1 = MIN_OT1; // don't allow to set less than minimum unless setting to zero
                            pwm_heat.write(map(levelOT1, 0, 100, 0, 1000));
                            // #if defined(DEBUG_MODE)
                            //                             Serial.printf("HEAT:%d\n", levelOT1);//for debug
                            // #endif
                            // sprintf(BLE_data_buffer_char, "#DATA_OUT,OT1,%d\n", levelOT1);
                            // // Serial.print(BLE_data_buffer_char);
                            // memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
                            // // 格式转换
                            // if (deviceConnected)
                            // {
                            //     pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                            //     pTxCharacteristic->notify();
                            // }
                        }
                    }
                }
                else if (CMD_Data[0] == "PID")
                {
                    if (CMD_Data[1] == "ON")
                    {
                        pid_status = true;
                        Heat_pid_controller.SetMode(AUTOMATIC);
                        // Heat_pid_controller.start();
                        // #if defined(DEBUG_MODE)
                        //                         Serial.printf("PID is ON\n");//for debug
                        // #endif
                    }
                    else if (CMD_Data[1] == "OFF")
                    {
                        Heat_pid_controller.SetMode(MANUAL);
                        // Heat_pid_controller.stop();
                        // #if defined(DEBUG_MODE)
                        //                         Serial.printf("PID is OFF\n");//for debug
                        // #endif
                        I2C_EEPROM.get(0, pid_parm);
                        Heat_pid_controller.SetTunings(pid_parm.p, pid_parm.i, pid_parm.d);
                        // Heat_pid_controller.setCoefficients(pid_parm.p, pid_parm.i, pid_parm.d);
                        pid_status = false;
                        pid_sv = 0;
                    }
                    else if (CMD_Data[1] == "SV")
                    {
                        if (pid_status == true)
                        {

                            pid_sv = CMD_Data[2].toFloat();
                            // #if defined(DEBUG_MODE)
                            //                             Serial.printf("PID set SV:%4.2f\n", pid_sv);//for debug
                            // #endif
                            // Heat_pid_controller.compute();
                            Heat_pid_controller.Compute();
                            levelOT1 = int(round(PID_output));
                            // levelOT1 = map(PID_output, 0, 255, 0, 100);

                            // Serial.printf("PID ON OT1: %d;PID_output:%4.2f\n", levelOT1,PID_output);
                            pwm_heat.write(map(levelOT1, 0, 100, 0, 1000));
                            // #if defined(DEBUG_MODE)
                            //                             Serial.printf("HEAT PID set :%d\n", levelOT1);//for debug
                            // #endif
                        }
                    }
                    else if (CMD_Data[1] == "TUNE")
                    {
                        Heat_pid_controller.SetMode(MANUAL);
                        pid_status = false;
                        pid_sv = 0;

                        vTaskResume(xTask_PID_autotune);
                        delay(100);
                        xTaskNotify(xTask_PID_autotune, 0, eIncrement); // 通知处理任务干活
                        vTaskSuspend(xTASK_BLE_CMD_handle);
                    }
                }
                // END of  big handle case switch
                delay(50);
            }
        }
    }
}

#endif
// command line
// 0   1   2
// PID
//     ON
//     OFF
//     SV
//         data

// IO3
//     UP
//     DOWN
//     data
// OT1
//     UP
//     DOWN
//     data