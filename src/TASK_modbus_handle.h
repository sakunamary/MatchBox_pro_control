#ifndef __TASK_MODBUS_HANDLE_H__
#define __TASK_MODBUS_HANDLE_H__

#include <Arduino.h>
#include <Wire.h>
#include <ModbusIP_ESP8266.h>
#include "ArduPID.h"
#include <pwmWrite.h>

// Modbus Registers Offsets
const uint16_t HEAT_HREG = 3003;
const uint16_t FAN_HREG = 3004;
const uint16_t PID_STATUS_HREG = 3005;
const uint16_t PID_SV_HREG = 3006;

uint16_t last_FAN;
uint16_t last_PWR;
int heat_level_to_artisan = 0;
int fan_level_to_artisan = 0;
bool pid_status = false;
extern double BT_TEMP;
bool init_status = true;

double PID_output;
double pid_sv = 0;
double pid_tune_output;
const uint32_t frequency = PWM_FREQ;
const byte resolution = PWM_RESOLUTION;
const byte pwm_fan_out = PWM_FAN;
const byte pwm_heat_out = PWM_HEAT;

Pwm pwm = Pwm();

ArduPID Heat_pid_controller;
PIDAutotuner tuner = PIDAutotuner();

void Task_modbus_handle(void *pvParameters)
{ // function
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t timeOut = 500;
    const TickType_t xIntervel = 500 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();
    uint8_t TEMP_DATA_Buffer[HMI_BUFFER_SIZE];

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS)
        {
            if (init_status) // 初始化
            {
                init_status = false;
                last_FAN = mb.Hreg(FAN_HREG);
                last_PWR = mb.Hreg(HEAT_HREG);
                fan_level_to_artisan = last_FAN;
                heat_level_to_artisan = last_PWR;
                pid_status = false;
                // xQueueSend(queueCMD_BLE, &BLE_ReadBuffer, timeOut);   // 串口数据发送至队列

                pwm.write(pwm_fan_out, map(fan_level_to_artisan, 0, 100, 100, 1000), frequency, resolution);
                pwm.write(pwm_heat_out, map(heat_level_to_artisan, 0, 100, 230, 850), frequency, resolution);
            }
            else
            {
                // 判断是否在pid自动烘焙模式
                if (mb.Hreg(PID_STATUS_HREG) == 1) // 开pid控制
                {
                    if (pid_status == false)
                    {                                                     // pid_status == false  and pid_status_hreg ==1
                        pid_status = true;                                // update value
                        Heat_pid_controller.start();                      //  Heat_pid_controller.begin(&BT_TEMP, &PID_output, &pid_sv, p, i, d);
                        pid_sv = mb.Hreg(PID_SV_HREG) / 10;               // 获取pid的SV
                        Heat_pid_controller.compute();                    // 计算输出值
                        heat_level_to_artisan = PID_output * 255 / 100;   // 更新pid计算后的数值
                        last_PWR = heat_level_to_artisan;                 // 更新pid计算后的数值
                        mb.Hreg(HEAT_HREG, round(heat_level_to_artisan)); // 更新pid计算后的数值

#if defined(DEBUG_MODE)
                        Heat_pid_controller.debug(&Serial,
                                                  "Heat_pid_controller", PRINT_INPUT | PRINT_OUTPUT | PRINT_SETPOINT | PRINT_BIAS);
#endif
                    }
                    else
                    {                                                     // pid_status == true and pid_status_hreg ==1
                        pid_sv = mb.Hreg(PID_SV_HREG) / 10;               // 获取pid的SV
                        Heat_pid_controller.compute();                    // 计算输出值
                        heat_level_to_artisan = PID_output * 255 / 100;   // 更新pid计算后的数值
                        last_PWR = heat_level_to_artisan;                 // 更新pid计算后的数值
                        mb.Hreg(HEAT_HREG, round(heat_level_to_artisan)); // 更新pid计算后的数值
#if defined(DEBUG_MODE)
                        Heat_pid_controller.debug(&Serial,
                                                  "Heat_pid_controller", PRINT_INPUT | PRINT_OUTPUT | PRINT_SETPOINT | PRINT_BIAS);
#endif
                    }
                }
                else // 关闭pid控制--手动控制
                {
                    if (pid_status == true) // pid_status = true and  pid_status_hreg ==0
                    {
                        Heat_pid_controller.stop();
                        mb.Hreg(PID_SV_HREG, 0);
                        pid_status = false;                        // update value
                        heat_level_to_artisan = last_PWR;          // 恢复上一次的数据
                        mb.Hreg(HEAT_HREG, heat_level_to_artisan); // 恢复上一次的数据
                    }
                    else // pid_status = false  and  pid_status_hreg ==0
                    {
                        if (last_PWR != mb.Hreg(HEAT_HREG)) // 发生变动
                        {
                            heat_level_to_artisan = mb.Hreg(HEAT_HREG);
                        }
                    }
                }
            }
            /////////////////////////////////风力控制始终手动
            if (last_FAN != mb.Hreg(FAN_HREG)) // 发生变动
            {
                fan_level_to_artisan = mb.Hreg(FAN_HREG);
            }
            pwm.write(pwm_fan_out, map(fan_level_to_artisan, 0, 100, 100, 1000), frequency, resolution);
            pwm.write(pwm_heat_out, map(heat_level_to_artisan, 0, 100, 230, 850), frequency, resolution);
            // 封装HMI数据
            make_frame_head(TEMP_DATA_Buffer, 1);
            TEMP_DATA_Buffer[7] = heat_level_to_artisan;
            TEMP_DATA_Buffer[8] = fan_level_to_artisan;
            make_frame_end(TEMP_DATA_Buffer, 1);

            xQueueSend(queue_data_to_HMI, &TEMP_DATA_Buffer, timeOut);
#if defined(DEBUG_MODE)
            Serial.write(TEMP_DATA_Buffer, HMI_BUFFER_SIZE);
#endif
            xTaskNotify(xTASK_data_to_HMI, 0, eIncrement);
            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
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
// NULL :00 00 00
// 帧尾:FF FF FF