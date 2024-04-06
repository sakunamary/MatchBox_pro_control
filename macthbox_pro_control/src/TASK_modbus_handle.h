#ifndef __TASK_MODBUS_HANDLE_H__
#define __TASK_MODBUS_HANDLE_H__

#include <Arduino.h>
#include <Wire.h>
#include <ModbusIP_ESP8266.h>
#include "ArduPID.h"

ArduPID Heat_pid_controller;

bool init_status = true;

// Modbus Registers Offsets
const uint16_t HEAT_HREG = 3003;
const uint16_t FAN_HREG = 3004;
const uint16_t PID_STRTUS_HREG = 3005;
const uint16_t PID_SV_HREG = 3006;
const uint16_t PID_TUNE_HREG = 3007;

uint16_t last_FAN;
uint16_t last_PWR;
int heat_level_to_artisan = 0;
int fan_level_to_artisan = 0;

double PID_output;
double pid_sv = 0;
double p = 2.0;
double i = 0.12;
double d = 5;

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
            if (init_status) // 初始化
            {
                last_FAN = mb.Hreg(FAN_HREG);
                last_PWR = mb.Hreg(HEAT_HREG);
                fan_level_to_artisan = last_FAN;
                heat_level_to_artisan = last_PWR;
                // xQueueSend(queueCMD_BLE, &BLE_ReadBuffer, timeOut);   // 串口数据发送至队列
                init_status = false;
                PWMAnalogWrite(PWM_FAN_CHANNEL, fan_level_to_artisan, 100);   // 自动模式下，将heat数值转换后输出到pwm
                PWMAnalogWrite(PWM_HEAT_CHANNEL, heat_level_to_artisan, 100); // 自动模式下，将heat数值转换后输出到pwm
            }
            else
            {
                if (last_FAN != mb.Hreg(FAN_HREG)) // 发生变动
                {
                    fan_level_to_artisan = mb.Hreg(FAN_HREG);
                }
                // 判断是否在pid自动烘焙模式
                if (mb.Hreg(PID_STRTUS_HREG) != 0) // 开pid控制
                {
                   Heat_pid_controller.start(); 
                    //  Heat_pid_controller.begin(&BT_TEMP, &PID_output, &pid_sv, p, i, d);
                    pid_sv = mb.Hreg(PID_SV_HREG)/10;      // 获取pid的SV
                    Heat_pid_controller.compute();      // 计算输出值
                    heat_level_to_artisan = PID_output*255/100; // 更新pid计算后的数值
                    last_PWR =  PID_output*255/100;              // 更新pid计算后的数值
                    mb.Hreg(HEAT_HREG,  round(PID_output*255/100));     // 更新pid计算后的数值

                    Heat_pid_controller.debug(&Serial, 
                    "Heat_pid_controller", PRINT_INPUT |     
                      PRINT_OUTPUT |  PRINT_SETPOINT | 
                      PRINT_BIAS);
                }
                else // 关闭pid控制--手动控制
                {
                   Heat_pid_controller.stop(); 
                   mb.Hreg(PID_SV_HREG,0); 
                    if (last_PWR != mb.Hreg(HEAT_HREG)) // 发生变动
                    {
                        heat_level_to_artisan = mb.Hreg(HEAT_HREG);
                    }
                }
            }
            PWMAnalogWrite(PWM_FAN_CHANNEL, fan_level_to_artisan, 100);   // 自动模式下，将heat数值转换后输出到pwm
            PWMAnalogWrite(PWM_HEAT_CHANNEL, heat_level_to_artisan, 100); // 自动模式下，将heat数值转换后输出到pwm
            xSemaphoreGive(xThermoDataMutex);                             // end of lock mutex
        }
    }
}

#endif