// 发送指令到HMI
#ifndef __TASK_HMI_SERIAL_H__
#define __TASK_HMI_SERIAL_H__

#include <Arduino.h>
#include <config.h>

// For ESP32-C3
HardwareSerial Serial_HMI(0);

extern uint16_t last_FAN;
extern uint16_t last_PWR;

// Arbitrary setpoint and gains - adjust these as fit for your project:

extern double BT_TEMP; // pid_input
extern double PID_output;
extern double pid_sv;
extern bool pid_status;

void TASK_data_to_HMI(void *pvParameters)
{
    (void)pvParameters;
    uint8_t Serial_DATA_Buffer[HMI_BUFFER_SIZE];
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
            if (xQueueReceive(queue_data_to_HMI, &Serial_DATA_Buffer, timeOut) == pdPASS)
            { // 从接收QueueCMD 接收指令
                Serial_HMI.write(Serial_DATA_Buffer, HMI_BUFFER_SIZE);
                vTaskDelay(20);
            }
        }
    }
}

void TASK_CMD_FROM_HMI(void *pvParameters)
{
    (void)pvParameters;
    uint8_t HMI_ReadBuffer[HMI_BUFFER_SIZE];
    const TickType_t timeOut = 500;
    while (1)
    {
        if (Serial_HMI.available())
        {
            if (xSemaphoreTake(xSerialReadBufferMutex, timeOut) == pdPASS)
            {
                Serial_HMI.readBytes(HMI_ReadBuffer, HMI_BUFFER_SIZE);
                xQueueSend(queueCMD_HMI, &HMI_ReadBuffer, timeOut); // 串口数据发送至队列
                xTaskNotify(xTASK_HMI_CMD_handle, 0, eIncrement);   // 通知处理任务干活
            }
            xSemaphoreGive(xSerialReadBufferMutex);
        }
        vTaskDelay(20);
    }
}

void TASK_HMI_CMD_handle(void *pvParameters)
{
    (void)pvParameters;
    uint8_t HMI_CMD_Buffer[HMI_BUFFER_SIZE];
    const TickType_t timeOut = 1000;
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
            if (xQueueReceive(queueCMD_HMI, &HMI_CMD_Buffer, timeOut) == pdPASS)
            { // 从接收QueueCMD 接收指令

                switch (HMI_CMD_Buffer[2])
                {          // 判断命令的类型
                case 0x01: // 操作控制
                    if (xSemaphoreTake(xSerialReadBufferMutex, timeOut) == pdPASS)
                    {
                        if (pid_status == false && mb.Hreg(PID_STRTUS_HREG) == 0)
                        { // pid_status = false  and  pid_status_hreg ==0
                            if (HMI_CMD_Buffer[7] != last_PWR)
                            {
                                last_PWR = HMI_CMD_Buffer[7];
                                mb.Hreg(HEAT_HREG, last_PWR);                                 // last 火力pwr数据更新
                                PWMAnalogWrite(PWM_HEAT_CHANNEL, heat_level_to_artisan, 100); // 自动模式下，将heat数值转换后输出到pwm // 输出新火力pwr到SSRÍ
                            }
                        }
                        // HMI_CMD_Buffer[5]   //火力数据
                        if (HMI_CMD_Buffer[8] != last_FAN)
                        {
                            last_FAN = HMI_CMD_Buffer[8];
                            mb.Hreg(FAN_HREG, last_FAN);                                // last 火力pwr数据更新
                            PWMAnalogWrite(PWM_FAN_CHANNEL, fan_level_to_artisan, 100); // 自动模式下，将heat数值转换后输出到pwm // 输出新火力pwr到SSRÍ
                        }
                        xSemaphoreGive(xSerialReadBufferMutex);
                    }
                    break;
                case 0x02: // PID参数设置
                    if (xSemaphoreTake(xSerialReadBufferMutex, timeOut) == pdPASS)
                    {
                        pid_parm.p = double((HMI_CMD_Buffer[3] << 8 | HMI_CMD_Buffer[4]) / 100);
                        pid_parm.i = double((HMI_CMD_Buffer[5] << 8 | HMI_CMD_Buffer[6]) / 100);
                        pid_parm.d = double((HMI_CMD_Buffer[7] << 8 | HMI_CMD_Buffer[8]) / 100);
                        EEPROM.put(0, pid_parm);
                        EEPROM.commit();
                        xSemaphoreGive(xSerialReadBufferMutex);
                    }
                    break;
                case 0x03: // 其他参数设置
                    pid_parm.pid_CT = HMI_CMD_Buffer[3] << 8 | HMI_CMD_Buffer[4];
                    pid_parm.BT_tempfix = double((HMI_CMD_Buffer[5] << 8 | HMI_CMD_Buffer[6]) / 100);
                    pid_parm.ET_tempfix = double((HMI_CMD_Buffer[7] << 8 | HMI_CMD_Buffer[8]) / 100);
                    EEPROM.put(0, pid_parm);
                    EEPROM.commit();
                    break;
                case 0x04: // 其他参数设置
                           // if (xSemaphoreTake(xThermoDataMutex, timeOut) == pdPASS)
                           // {
                           //     pid_sv = double((HMI_CMD_Buffer[3] << 8 | HMI_CMD_Buffer[4]) / 100);
                           //     xSemaphoreGive(xThermoDataMutex);  　
                           // }

                    mb.Hreg(PID_STRTUS_HREG, HMI_CMD_Buffer[5]);
                    mb.Hreg(PID_TUNE_HREG, HMI_CMD_Buffer[6]);
                    break;
                default:
                    break;
                }
            }
            vTaskDelay(20);
        }
    }
}

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
// PID TUNE : 00
// NULL : 00 00
// 帧尾:FF FF FF