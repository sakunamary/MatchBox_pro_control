#ifndef __TASK_MODBUS_CONTROL_H__
#define __TASK_MODBUS_CONTROL_H__

#include <Arduino.h>
#include <config.h>
#include <ESP32Servo.h>

const int HEAT_OUT_PIN = PWM_HEAT; // GPIO26
const int FAN_OUT_PIN = PWM_FAN;   // GPIO26
const int frequency = PWM_FREQ;
const byte resolution = PWM_RESOLUTION; // pwm -0-4096

extern ESP32PWM pwm_heat;
extern ESP32PWM pwm_fan;

extern PID Heat_pid_controller;
uint16_t last_PWR;
uint16_t last_FAN;

const uint16_t FAN_HREG = 3005;  // COOLING FAN SWITCH
const uint16_t HEAT_HREG = 3006; // HEAT SWTICH

const uint16_t PID_P_HREG = 3007;
const uint16_t PID_I_HREG = 3008;
const uint16_t PID_D_HREG = 3009;
const uint16_t PID_SV_HREG = 3010;     // PID SV
const uint16_t PID_STATUS_HREG = 3011; // PID RUNNING STATUS

int levelOT1 = 0;
int levelIO3 = 30;
bool init_status = true;
bool pid_status = false;

const byte pwm_heat_out = HEAT_OUT_PIN;
const byte pwm_fan_out = FAN_OUT_PIN;

double PID_output; // 取值范围 （0-255）
double pid_sv = 0;

int pid_out_min = PID_MIN_OUT;
int pid_out_max = PID_MAX_OUT;

void Task_modbus_control(void *pvParameters)
{ // function
    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xIntervel = 200 / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while (1) // A Task shall never return or exit.
    {         // for loop
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        // HEAT_HREG
        if (init_status) // 初始化状态
        {
            //
            if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) //
            {
                last_PWR = mb.Hreg(HEAT_HREG);
                last_FAN = mb.Hreg(FAN_HREG);
                levelOT1 = last_PWR;
                levelIO3 = last_FAN;
                pid_sv = 0;
                mb.Hreg(PID_SV_HREG, 0);
                pwm_heat.write(map(levelOT1, 0, 100, PWM_HEAT_MIN, PWM_HEAT_MAX));

                // pwm_heat.write(HEAT_OUT_PIN, map(heat_pwr_to_SSR, 0, 100, 230, 850), frequency, resolution); // 输出新火力pwr到SSRÍ
                xSemaphoreGive(xThermoDataMutex); // end of lock mutex
            }

            init_status = false;
        }
        else // 正常运作情况
        {
            if (mb.Hreg(PID_STATUS_HREG) == 1) // Artisan pid 开启
            {
                if (pid_status == false)                                       // 机器不在PID循环
                {                                                              // pid_status = false and pid_status_hreg =1
                    if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
                    {
                        pid_status = true; // update value
                        Heat_pid_controller.SetMode(AUTOMATIC);
                        pid_sv = mb.Hreg(PID_SV_HREG) / 10;

                        // Heat_pid_controller.start();
                        Heat_pid_controller.Compute();                  // 计算pid输出
                        levelOT1 = map(PID_output - 2, 0, 255, 0, 100); // 转换为格式 pid_output (0,255) -> (0,100)
                        last_PWR = levelOT1;
                        mb.Hreg(HEAT_HREG, levelOT1);
                        xSemaphoreGive(xThermoDataMutex);
                    }
                }
                else                                                           //
                {                                                              // pid_status = true and pid_status_hreg =1
                    if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
                    {
                        pid_sv = mb.Hreg(PID_SV_HREG) / 10; // 计算pid输出
                        Heat_pid_controller.Compute();
                        levelOT1 = map(PID_output - 2, 0, 255, 0, 100); // 转换为格式 pid_output (0,255) -> (0,100)
                        last_PWR = levelOT1;
                        mb.Hreg(HEAT_HREG, levelOT1);
                        xSemaphoreGive(xThermoDataMutex);
                    }
                }
            }
            else // Artisan PID 关闭 --手动模式
            {
                if (pid_status == true)
                {                                                              // pid_status = true and pid_status_hreg =0
                    if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
                    {
                        Heat_pid_controller.SetMode(MANUAL);
                        pid_sv = 0;
                        mb.Hreg(PID_SV_HREG, 0);
                        pid_status = false;
                        levelOT1 = last_PWR;
                        mb.Hreg(HEAT_HREG, levelOT1);
                        xSemaphoreGive(xThermoDataMutex);
                    }
                }
                else
                {                                       // pid_status = false and pid_status_hreg =0
                    if (mb.Hreg(HEAT_HREG) != last_PWR) // 火力pwr数值发生变动
                    {
                        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
                        {
                            last_PWR = mb.Hreg(HEAT_HREG); // last 火力pwr数据更新
                            levelOT1 = last_PWR;           // 发送新火力pwr数据到 SSR
                            xSemaphoreGive(xThermoDataMutex);
                        }
                    }
                }
            }
            pwm_heat.write(map(levelOT1, 0, 100, PWM_HEAT_MIN, PWM_HEAT_MAX));
            // pwm_heat.write(HEAT_OUT_PIN, map(heat_pwr_to_SSR, 0, 100, 230, 850), frequency, resolution); // 输出新火力pwr到SSRÍ
        }
        vTaskDelay(20);
        /////////////////////////////////////////////////////////////////////////////

        // FAN 手动控制  不走 PID循环
        if (mb.Hreg(FAN_HREG) != last_FAN) // 风扇开关状态发生变动
        {

            last_FAN = mb.Hreg(FAN_HREG);
            levelIO3 = last_FAN;
            if (levelIO3 > MAX_IO3)
            {
                levelIO3 = MAX_IO3;
                last_FAN = levelIO3;
                mb.Hreg(FAN_HREG, last_FAN);
            } // don't allow OT1 to exceed maximum
            if (levelIO3 < MIN_IO3 & levelIO3 != 0)
            {
                levelIO3 = MIN_IO3;
                last_FAN = levelIO3;
                mb.Hreg(FAN_HREG, last_FAN);
            }

            pwm_fan.write(map(levelIO3, 0, 100, PWM_FAN_MIN, PWM_FAN_MAX));
        }
    }
    vTaskDelay(20);
}

void Task_PID_PARM_SETTING(void *pvParameters)
{ // function
    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xIntervel = 500 / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while (1) // A Task shall never return or exit.
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);

        if (mb.Hreg(PID_P_HREG) != int(pid_parm.p * 100))
        {
            pid_parm.p = (double)mb.Hreg(PID_P_HREG) / 100;
        }
        if (mb.Hreg(PID_I_HREG) != int(pid_parm.i * 100))
        {
            pid_parm.i = (double)mb.Hreg(PID_I_HREG) / 100;
        }
        if (mb.Hreg(PID_D_HREG) != int(pid_parm.d * 100))
        {
            pid_parm.d = (double)mb.Hreg(PID_D_HREG) / 100;
        }

        Heat_pid_controller.SetTunings(pid_parm.p, pid_parm.i, pid_parm.d);

    }
}

#endif