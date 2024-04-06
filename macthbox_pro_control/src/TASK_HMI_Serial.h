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
                if (xSemaphoreTake(xSerialReadBufferMutex, timeOut) == pdPASS)
                {
                    // HMI_CMD_Buffer[3]   //火力数据
                    if (HMI_CMD_Buffer[7] != last_PWR)
                    {
                        last_PWR = HMI_CMD_Buffer[7];
                        mb.Hreg(HEAT_HREG, last_PWR);                                 // last 火力pwr数据更新
                        PWMAnalogWrite(PWM_HEAT_CHANNEL, heat_level_to_artisan, 100); // 自动模式下，将heat数值转换后输出到pwm // 输出新火力pwr到SSRÍ
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
            }
            vTaskDelay(20);
        }
    }
}
#endif