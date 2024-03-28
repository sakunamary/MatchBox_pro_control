#ifndef __TASK_MODBUS_HANDLE_H__
#define __TASK_MODBUS_HANDLE_H__

#include <Arduino.h>
#include <Wire.h>
#include <ModbusIP_ESP8266.h>

bool init_status = true;


uint16_t last_FAN;
uint16_t last_PWR;



int heat_level_to_artisan = 0;
int fan_level_to_artisan = 0;

void Task_modbus_handle(void *pvParameters)
{ // function
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xIntervel = 200 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS)
        {
            if (init_status)
            {
                last_FAN = mb.Hreg(FAN_HREG);
                last_PWR = mb.Hreg(HEAT_HREG);
               // xQueueSend(queueCMD_BLE, &BLE_ReadBuffer, timeOut);   // 串口数据发送至队列

            }
            else
            {
                if (last_FAN != mb.Hreg(FAN_HREG)) // 发生变动
                {
                    fan_level_to_artisan = mb.Hreg(FAN_HREG);
                }
                if (last_PWR != mb.Hreg(HEAT_HREG)) // 发生变动
                {
                    heat_level_to_artisan = mb.Hreg(HEAT_HREG);
                }
            }

            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }
        PWMAnalogWrite(PWM_FAN_CHANNEL, fan_level_to_artisan); // 自动模式下，将heat数值转换后输出到pwm
        PWMAnalogWrite(PWM_HEAT_CHANNEL, heat_level_to_artisan); // 自动模式下，将heat数值转换后输出到pwm
    }
}

#endif