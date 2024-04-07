#ifndef __TASK_MODBUS_HANDLE_H__
#define __TASK_MODBUS_HANDLE_H__

#include <Arduino.h>
#include <Wire.h>
#include <ModbusIP_ESP8266.h>
#include "ArduPID.h"

ArduPID Heat_pid_controller;
PIDAutotuner tuner = PIDAutotuner();
bool init_status = true;

// Modbus Registers Offsets
const uint16_t HEAT_HREG = 3003;
const uint16_t FAN_HREG = 3004;
const uint16_t PID_STRTUS_HREG = 3005;
const uint16_t PID_SV_HREG = 3006;
const uint16_t PID_TUNE_HREG = 3007;

long microseconds;
long prevMicroseconds;
uint16_t last_FAN;
uint16_t last_PWR;
int heat_level_to_artisan = 0;
int fan_level_to_artisan = 0;
bool pid_status = false;
extern double BT_TEMP;

double PID_output;
double pid_sv = 0;
double pid_tune_output;

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

                PWMAnalogWrite(PWM_FAN_CHANNEL, fan_level_to_artisan, 100);   // 自动模式下，将fan数值转换后输出到pwm
                PWMAnalogWrite(PWM_HEAT_CHANNEL, heat_level_to_artisan, 100); // 自动模式下，将heat数值转换后输出到pwm
            }
            else
            {
                if (last_FAN != mb.Hreg(FAN_HREG)) // 发生变动
                {
                    fan_level_to_artisan = mb.Hreg(FAN_HREG);
                }
                // 判断是否在pid自动烘焙模式
                if (mb.Hreg(PID_STRTUS_HREG) == 1) // 开pid控制
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
            PWMAnalogWrite(PWM_FAN_CHANNEL, fan_level_to_artisan, 100);   // 自动模式下，将heat数值转换后输出到pwm
            PWMAnalogWrite(PWM_HEAT_CHANNEL, heat_level_to_artisan, 100); // 自动模式下，将heat数值转换后输出到pwm
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
    if (mb.Hreg(PID_TUNE_HREG) == 1)
    {
        // Run a loop until tuner.isFinished() returns true
        long microseconds;
        while (!tuner.isFinished())
        {

            // This loop must run at the same speed as the PID control loop being tuned
            prevMicroseconds = microseconds;
            microseconds = micros();

            // Get input value here (temperature, encoder position, velocity, etc)
            // Call tunePID() with the input value
            pid_tune_output = tuner.tunePID(BT_TEMP);

            // Set the output - tunePid() will return values within the range configured
            // by setOutputRange(). Don't change the value or the tuning results will be
            // incorrect.
            PWMAnalogWrite(PWM_FAN_CHANNEL, 40, 100);               // 自动模式下，将heat数值转换后输出到pwm
            PWMAnalogWrite(PWM_HEAT_CHANNEL, pid_tune_output, 100); // 自动模式下，将heat数值转换后输出到pwm
            // This loop must run at the same speed as the PID control loop being tuned
            while (micros() - microseconds < pid_parm.pid_CT)
                delayMicroseconds(1);
        }

        // Turn the output off here.
        PWMAnalogWrite(PWM_HEAT_CHANNEL, 0, 100);

        // Get PID gains - set your PID controller's gains to these
        pid_parm.p = tuner.getKp();
        pid_parm.i = tuner.getKi();
        pid_parm.d = tuner.getKd();
        EEPROM.put(0, pid_parm);
        EEPROM.commit();
        mb.Hreg(PID_TUNE_HREG, 0);
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