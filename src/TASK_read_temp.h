#ifndef __TASK_READ_TEM_H__
#define __TASK_READ_TEMP_H__

#include <Arduino.h>
#include <config.h>
#include <Wire.h>
#include <MCP3424.h>
#include "DFRobot_BME280.h"

double BT_TEMP;
double ET_TEMP;
double AMB_RH;
double AMB_TEMP;
uint32_t AMB_PRESS;
extern int levelOT1;
extern int levelIO3;
extern double pid_sv;
extern bool pid_status;
extern bool PID_TUNNING;
extern pid_setting_t pid_parm;
float PID_TUNE_SV;
long prevMicroseconds;
long microseconds;
double pid_tune_output;

extern ExternalEEPROM I2C_EEPROM;
extern PID Heat_pid_controller;
filterRC BT_TEMP_ft;
filterRC ET_TEMP_ft;

// extern ArduPID Heat_pid_controller;
extern PIDAutotuner tuner;
extern ESP32PWM pwm_heat;
extern ESP32PWM pwm_fan;
// Need this for the lower level access to set them up.
uint8_t address = 0x68;
long Voltage; // Array used to store results
unsigned long temp_check[3];

typedef DFRobot_BME280_IIC BME; // ******** use abbreviations instead of full names ********

/**IIC address is 0x77 when pin SDO is high */
/**IIC address is 0x76 when pin SDO is low */
BME bme(&Wire, 0x76); // select TwoWire peripheral and set sensor address

#define SEA_LEVEL_PRESSURE 1015.0f

#define R0 100
#define Rref 1000

MCP3424 MCP(address); // Declaration of MCP3424 A2=0 A1=1 A0=0
// DFRobot_AHT20 aht20;
//  TypeK temp_K_cal;

extern pid_setting_t pid_parm;
extern HardwareSerial Serial_HMI;

void Task_Thermo_get_data(void *pvParameters)
{ // function

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;
    char temp_data_buffer_ble[BLE_BUFFER_SIZE];
    uint8_t temp_data_buffer_hmi[HMI_BUFFER_SIZE];
    const TickType_t xIntervel = (pid_parm.pid_CT * 1000) / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_add(NULL);
    // if (esp_task_wdt_status(NULL) != ESP_OK)
    // {
    //     esp_task_wdt_add(NULL);
    // }

    for (;;) // A Task shall never return or exit.
    {        // for loop
             // 喂狗
        esp_task_wdt_reset();
        // Wait for the next cycle (intervel 1500ms).
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
        {

            // if (aht20.startMeasurementReady(/* crcEn = */ true))
            // {
            AMB_TEMP = bme.getTemperature();
            AMB_PRESS = bme.getPressure();
            AMB_RH = bme.getHumidity();
            // #if defined(DEBUG_MODE)
            //                 Serial.printf("raw data:AMB_TEMP:%4.2f\n", AMB_TEMP);
            // #endif
            // }
            delay(200);
            MCP.Configuration(1, 16, 1, 1);               // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
            Voltage = MCP.Measure();                      // Measure is stocked in array Voltage, note that the library will wait for a completed conversion that takes around 200 ms@18bits            BT_TEMP = pid_parm.BT_tempfix + (((Voltage / 1000 * Rref) / ((3.3 * 1000) - Voltage / 1000) - R0) / (R0 * 0.0039083));
            Voltage = BT_TEMP_ft.doFilter(Voltage << 10); // multiply by 1024 to create some resolution for filter
            Voltage >>= 10;
            BT_TEMP = pid_parm.BT_tempfix + (((Voltage / 1000 * Rref) / ((3.3 * 1000) - Voltage / 1000) - R0) / (R0 * 0.0039083));
            delay(200);
            MCP.Configuration(2, 16, 1, 1);               // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
            Voltage = MCP.Measure();                      // Measure is stocked in array Voltage, note that the library will wait for a completed conversion that takes around 200 ms@18bits
            Voltage = ET_TEMP_ft.doFilter(Voltage << 10); // multiply by 1024 to create some resolution for filter
            Voltage >>= 10;
            ET_TEMP = pid_parm.ET_tempfix + (((Voltage / 1000 * Rref) / ((3.3 * 1000) - Voltage / 1000) - R0) / (R0 * 0.0039083));
            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }
        // 检查温度是否达到切换PID参数
#if defined(PID_AUTO_SHIFT)
        if (pid_status && !PID_TUNNING)
        {
            if (BT_TEMP >= PID_TUNE_SV_1)
            {
                I2C_EEPROM.get(128, pid_parm);
                Heat_pid_controller.SetOutputLimits(PID_STAGE_2_MIN_OUT, PID_STAGE_2_MAX_OUT);
                Heat_pid_controller.SetTunings(pid_parm.p, pid_parm.i, pid_parm.d);
            }
            else if (BT_TEMP >= PID_TUNE_SV_2)
            {
                I2C_EEPROM.get(256, pid_parm);
                Heat_pid_controller.SetOutputLimits(PID_STAGE_3_MIN_OUT, PID_STAGE_3_MAX_OUT);
                Heat_pid_controller.SetTunings(pid_parm.p, pid_parm.i, pid_parm.d);
            }
        }
#endif
        // 检查温度是否达到降温降风
        if (PID_TUNNING == false && pid_status == false)
        {
            if (BT_TEMP > 50 && BT_TEMP < 60)
            {
                temp_check[0] = millis();
#if defined(DEBUG_MODE)
                Serial.printf("\nTempCheck[0]:%ld\n", temp_check[0]);
#endif
            }
            if (BT_TEMP > 120 && BT_TEMP < 135)
            {
                temp_check[1] = millis();
#if defined(DEBUG_MODE)
                Serial.printf("\nTempCheck[1]:%ld\n", temp_check[1]);
#endif
            }
            if (BT_TEMP > 180)
            {
                temp_check[2] = millis();
#if defined(DEBUG_MODE)
                Serial.printf("\nTempCheck[2]:%ld\n", temp_check[2]);
#endif
            }

            if (temp_check[2] != 0 && temp_check[1] != 0 && temp_check[0] != 0) // 确认是机器运行中
            {
                if (temp_check[2] < temp_check[1] && temp_check[1] < temp_check[0]) // 判断温度趋势是下降
                {
#if defined(DEBUG_MODE)
                    Serial.printf("\n Turn Down fan t0:%ld t1:%ld t2:%ld\n", temp_check[0], temp_check[1], temp_check[2]);
#endif
                    levelIO3 = 35;
                    pwm_fan.write(map(levelIO3, MIN_IO3, MAX_IO3, PWM_FAN_MIN, PWM_FAN_MAX));
                    pwm_heat.write(1); // for safe
                    temp_check[2] = 0;
                    temp_check[1] = 0;
                    temp_check[0] = 0;
                }
            }
        }

        // PID ON:ambient,chan1,chan2,  heater duty, fan duty, SV
        if (xSemaphoreTake(xDATA_OUT_Mutex, 250 / portTICK_PERIOD_MS) == pdPASS) // 给温度数组的最后一个数值写入数据
        {
            // 封装BLE 协议
            sprintf(temp_data_buffer_ble, "#%4.2f,%4.2f,%4.2f,%d,%d,%4.2f;\n", AMB_TEMP, ET_TEMP, BT_TEMP, levelOT1, levelIO3, pid_sv);
            // Serial.print(temp_data_buffer_ble);

            xQueueSend(queue_data_to_BLE, &temp_data_buffer_ble, xIntervel);

            // 封装HMI 协议
            temp_data_buffer_hmi[0] = 0x69;
            temp_data_buffer_hmi[1] = 0xff;
            temp_data_buffer_hmi[2] = 0x01;
            temp_data_buffer_hmi[3] = lowByte(int(round(AMB_TEMP * 10)));
            temp_data_buffer_hmi[4] = highByte(int(round(AMB_TEMP * 10)));
            temp_data_buffer_hmi[5] = lowByte(int(round(ET_TEMP * 10)));
            temp_data_buffer_hmi[6] = highByte(int(round(ET_TEMP * 10)));
            temp_data_buffer_hmi[7] = lowByte(int(round(BT_TEMP * 10)));
            temp_data_buffer_hmi[8] = highByte(int(round(BT_TEMP * 10)));
            temp_data_buffer_hmi[9] = lowByte(int(round(pid_sv * 10)));
            temp_data_buffer_hmi[10] = highByte(int(round(pid_sv * 10)));
            temp_data_buffer_hmi[11] = levelOT1;
            temp_data_buffer_hmi[12] = levelIO3;
            if (pid_status)
            {
                temp_data_buffer_hmi[13] = 0x01;
            }
            else
            {
                temp_data_buffer_hmi[13] = 0x00;
            }
            temp_data_buffer_hmi[14] = 0xff;
            temp_data_buffer_hmi[15] = 0xff;
            temp_data_buffer_hmi[16] = 0xff;

            xQueueSend(queue_data_to_HMI, &temp_data_buffer_hmi, xIntervel);
            memset(temp_data_buffer_hmi, '\0', HMI_BUFFER_SIZE);
            xSemaphoreGive(xDATA_OUT_Mutex); // end of lock mutex
        }
        xTaskNotify(xTASK_data_to_BLE, 0, eIncrement); // send notify to TASK_data_to_HMI
        xTaskNotify(xTASK_data_to_HMI, 0, eIncrement); // send notify to TASK_data_to_HMIÍ
    }

} // function

void Task_PID_autotune(void *pvParameters)
{
    (void)pvParameters;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;
    const TickType_t xIntervel = 3000 / portTICK_PERIOD_MS;

    while (1)
    {
        xResult = xTaskNotifyWait(0x00,                 // 在运行前这个命令之前，先清除这几位
                                  0x00,                 // 运行后，重置所有的bits 0x00 or ULONG_MAX or 0xFFFFFFFF
                                  &ulNotificationValue, // 重置前的notification value
                                  portMAX_DELAY);       // 一直等待

        if (xResult == pdTRUE)
        {
            // 开始 PID自动整定

            for (int loop = 0; loop < 3; loop++)
            {
                if (loop == 0)
                {
                    tuner.startTuningLoop(pid_parm.pid_CT * uS_TO_S_FACTOR);
                    PID_TUNE_SV = PID_TUNE_SV_1;
                    levelIO3 = PID_TUNE_FAN_1;
                    tuner.setOutputRange(round(PID_STAGE_1_MIN_OUT * 255 / 100), round(PID_STAGE_1_MAX_OUT * 255 / 100));
                    tuner.setTuningCycles(PID_TUNE_CYCLE);
                    tuner.setTargetInputValue(PID_TUNE_SV);
                    pwm_heat.writeScaled(0.0);
                    
                 pwm_fan.write(map(levelIO3, MIN_IO3, MAX_IO3, PWM_FAN_MIN, PWM_FAN_MAX));
                    vTaskDelay(1000);
                    while (!tuner.isFinished()) // 开始自动整定循环
                    {
                        prevMicroseconds = microseconds;
                        microseconds = micros();

                        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
                        {
                            pid_tune_output = tuner.tunePID(BT_TEMP, microseconds);
                            pwm_heat.write(map(pid_tune_output, 0, 255, 0, 1000));
                            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
                        }
#if defined(DEBUG_MODE)
                        Serial.printf("PID set1 :PID SV %4.2f \n", PID_TUNE_SV);
                        Serial.printf("PID Auto Tuneing...OUTPUT:%4.2f BT_temp:%4.2f AMB_TEMP:%4.2f\n", pid_tune_output, BT_TEMP, AMB_TEMP);
#endif
                        //  This loop must run at the same speed as the PID control loop being tuned
                        while (micros() - microseconds < pid_parm.pid_CT * uS_TO_S_FACTOR)
                        {
                            delayMicroseconds(1);
                        } // time units : us
                    }
                    // Turn the output off here.
                    pwm_heat.writeScaled(0.0);
                    pwm_fan.write(map(levelIO3, MIN_IO3, MAX_IO3, PWM_FAN_MIN, PWM_FAN_MAX));
                    // Get PID gains - set your PID controller's gains to these
                    pid_parm.p = tuner.getKp();
                    pid_parm.i = tuner.getKi();
                    pid_parm.d = tuner.getKd();
                    I2C_EEPROM.put(0, pid_parm);
#if defined(DEBUG_MODE)
                    Serial.printf("\nPID Auto Tune First step Finished ...\n");
                    Serial.printf("\nPID kp:%4.2f\n", pid_parm.p);
                    Serial.printf("\nPID ki:%4.2f\n", pid_parm.i);
                    Serial.printf("\nPID kd:%4.2f\n", pid_parm.d);
                    Serial.printf("\nBT fix:%4.2f", pid_parm.BT_tempfix);
                    Serial.printf("\nET fix:%4.2f", pid_parm.ET_tempfix);
#endif
                    Serial.printf("\nPID parms saved ...\n");
                }
                else if (loop == 1)
                {
                    tuner.startTuningLoop(pid_parm.pid_CT * uS_TO_S_FACTOR);
                    PID_TUNE_SV = PID_TUNE_SV_2;
                    levelIO3 = PID_TUNE_FAN_2;
                    tuner.setTuningCycles(PID_TUNE_CYCLE);
                    tuner.setOutputRange(round(PID_STAGE_2_MIN_OUT * 255 / 100), round(PID_STAGE_2_MAX_OUT * 255 / 100));
                    tuner.setTargetInputValue(PID_TUNE_SV);
                    tuner.setTargetInputValue(PID_TUNE_SV);
                    pwm_heat.writeScaled(0.0);
                    pwm_fan.write(map(levelIO3, MIN_IO3, MAX_IO3, PWM_FAN_MIN, PWM_FAN_MAX));
                    vTaskDelay(1000);
                    while (!tuner.isFinished()) // 开始自动整定循环
                    {
                        prevMicroseconds = microseconds;
                        microseconds = micros();

                        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
                        {
                            pid_tune_output = tuner.tunePID(BT_TEMP, microseconds);
                            pwm_heat.write(map(pid_tune_output, 0, 255, 0, 1000)); // 输出新火力pwr到SSRÍ
                            xSemaphoreGive(xThermoDataMutex);                      // end of lock mutex
                        }
#if defined(DEBUG_MODE)
                        Serial.printf("PID set2 :PID SV %4.2f \n", PID_TUNE_SV);
                        Serial.printf("PID Auto Tuneing...OUTPUT:%4.2f BT_temp:%4.2f AMB_TEMP:%4.2f\n", pid_tune_output, BT_TEMP, AMB_TEMP);
#endif
                        //  This loop must run at the same speed as the PID control loop being tuned
                        while (micros() - microseconds < pid_parm.pid_CT * uS_TO_S_FACTOR) // time units : us
                        {
                            delayMicroseconds(1);
                        } // time units : us
                    }
                    // Turn the output off here.
                    pwm_heat.writeScaled(0.0);
                    pwm_fan.write(map(levelIO3, MIN_IO3, MAX_IO3, PWM_FAN_MIN, PWM_FAN_MAX));
                    // Get PID gains - set your PID controller's gains to these
                    pid_parm.p = tuner.getKp();
                    pid_parm.i = tuner.getKi();
                    pid_parm.d = tuner.getKd();
                    I2C_EEPROM.put(128, pid_parm);
#if defined(DEBUG_MODE)
                    Serial.printf("\nPID Auto Tune Second step Finished ...\n");
                    Serial.printf("\nPID kp:%4.2f\n", pid_parm.p);
                    Serial.printf("\nPID ki:%4.2f\n", pid_parm.i);
                    Serial.printf("\nPID kd:%4.2f\n", pid_parm.d);
                    Serial.printf("\nBT fix:%4.2f", pid_parm.BT_tempfix);
                    Serial.printf("\nET fix:%4.2f", pid_parm.ET_tempfix);
                    Serial.printf("\nPID parms saved ...\n");
#endif
                }
                else if (loop == 2)
                {
                    tuner.startTuningLoop(pid_parm.pid_CT * uS_TO_S_FACTOR);
                    PID_TUNE_SV = PID_TUNE_SV_3;
                    levelIO3 = PID_TUNE_FAN_3;
                    tuner.setTuningCycles(PID_TUNE_CYCLE);
                    tuner.setOutputRange(round(PID_STAGE_3_MIN_OUT * 255 / 100), round(PID_STAGE_3_MAX_OUT * 255 / 100));
                    tuner.setTargetInputValue(PID_TUNE_SV);
                    pwm_heat.writeScaled(0.0);
                    pwm_fan.write(map(levelIO3, MIN_IO3, MAX_IO3, PWM_FAN_MIN, PWM_FAN_MAX));
                    vTaskDelay(1000);
                    while (!tuner.isFinished()) // 开始自动整定循环
                    {
                        prevMicroseconds = microseconds;
                        microseconds = micros();

                        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
                        {
                            pid_tune_output = tuner.tunePID(BT_TEMP, microseconds);
                            pwm_heat.write(map(pid_tune_output, 0, 255, 0, 1000)); // 输出新火力pwr到SSRÍ
                            xSemaphoreGive(xThermoDataMutex);                      // end of lock mutex
                        }
#if defined(DEBUG_MODE)
                        Serial.printf("PID set3 :PID SV %4.2f \n", PID_TUNE_SV);
                        Serial.printf("PID Auto Tuneing...OUTPUT:%4.2f BT_temp:%4.2f AMB_TEMP:%4.2f\n", pid_tune_output, BT_TEMP, AMB_TEMP);
#endif
                        //  This loop must run at the same speed as the PID control loop being tuned
                        while (micros() - microseconds < pid_parm.pid_CT * uS_TO_S_FACTOR) // time units : us
                        {
                            delayMicroseconds(1);
                        } // time units : us
                    }
                    // Turn the output off here.
                    levelIO3 = 30;
                    pwm_heat.writeScaled(0.0);
                    pwm_fan.write(map(levelIO3, MIN_IO3, MAX_IO3, PWM_FAN_MIN, PWM_FAN_MAX));
                    // Get PID gains - set your PID controller's gains to these
                    pid_parm.p = tuner.getKp();
                    pid_parm.i = tuner.getKi();
                    pid_parm.d = tuner.getKd();
                    I2C_EEPROM.put(256, pid_parm);
#if defined(DEBUG_MODE)
                    Serial.printf("\nPID Auto Tune Second step Finished ...\n");
                    Serial.printf("\nPID kp:%4.2f\n", pid_parm.p);
                    Serial.printf("\nPID ki:%4.2f\n", pid_parm.i);
                    Serial.printf("\nPID kd:%4.2f\n", pid_parm.d);
                    Serial.printf("\nBT fix:%4.2f", pid_parm.BT_tempfix);
                    Serial.printf("\nET fix:%4.2f", pid_parm.ET_tempfix);
                    Serial.printf("\nPID parms saved ...\n");
#endif
                }
            }
        }
    }
    PID_TUNNING = false;
    delay(3000);
    ESP.restart();
    vTaskResume(xTASK_CMD_FROM_HMI);
    vTaskResume(xTASK_HMI_CMD_handle);
    vTaskResume(xTASK_BLE_CMD_handle);

    vTaskSuspend(NULL);
}

#endif

// printh 00 00 00 ff ff ff 88 ff ff ff//输出上电信息到串口
// 69 ff 00 ff ff ff 69 69 69 67 67 67 ff ff ff ff ff //握手协议

// HMI --> MatchBox的数据帧 FrameLenght = 17
// 帧头: 69 FF
// 类型: 01 温度数据
// 环境温： 00 00 //uint16
// 温度1: 00 00 // uint16
// 温度2: 00 00 // uint16
// PID SV: 00 00 // uint16
// 火力 : 00
// 风力 : 00
// PID RUN: 00
// 帧尾:FF FF FF

// HMI --> MatchBox的数据帧 FrameLenght = 17
// 帧头: 69 FF
// 类型: 02 PID设定
// BT fix: 00 00 // uint16
// ET fix: 00 00 // uint16
// P: 00 00 // uint16
// I: 00 00 // uint16
// D: 00 00 // uint16
// PID ct: 00
// 帧尾:FF FF FF