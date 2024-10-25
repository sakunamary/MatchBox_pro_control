#ifndef __TASK_READ_TEM_H__
#define __TASK_READ_TEMP_H__

#include <Arduino.h>
#include <config.h>
#include <Wire.h>
#include <MCP3424.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// #include "DFRobot_BME280.h"
#if defined(TC_TYPE_K)
#include "TypeK.h"
#include "DFRobot_AHT20.h"
#endif

double BT_TEMP;
double ET_TEMP;
double AMB_RH;
double AMB_TEMP;
double ror;

float rx;
int32_t ftemps;     // heavily filtered temps
int32_t ftimes;     // filtered sample timestamps
int32_t ftemps_old; // for calculating derivative
int32_t ftimes_old; // for calculating derivative

// uint32_t AMB_PRESS;
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
extern bool first;

uint8_t anlg1 = ANIN1;     // analog input pins
int32_t old_reading_anlg1; // previous analogue reading
uint8_t anlg2 = ANIN2;     // analog input pins
int32_t old_reading_anlg2; // previous analogue reading

extern ExternalEEPROM I2C_EEPROM;
extern PID Heat_pid_controller;
extern HD44780LCD LCD;

filterRC AMB_ft;
filterRC BT_TEMP_ft;
filterRC ET_TEMP_ft;
filterRC fRise;
filterRC fRoR;

// extern ArduPID Heat_pid_controller;
extern PIDAutotuner tuner;
extern ESP32PWM pwm_heat;
extern ESP32PWM pwm_fan;

extern BLEServer *pServer;
extern BLECharacteristic *pTxCharacteristic;
extern bool deviceConnected;
extern bool oldDeviceConnected;

// Need this for the lower level access to set them up.
uint8_t address = 0x68;
long Voltage; // Array used to store results

unsigned long temp_check[3];

// typedef DFRobot_BME280_IIC BME; // ******** use abbreviations instead of full names ********

/**IIC address is 0x77 when pin SDO is high */
/**IIC address is 0x76 when pin SDO is low */
// BME bme(&Wire, 0x76); // select TwoWire peripheral and set sensor address

// #define SEA_LEVEL_PRESSURE 1015.0f

#define R0 100
#define Rref 1000

MCP3424 MCP(address); // Declaration of MCP3424 A2=0 A1=1 A0=0
#if defined(TC_TYPE_K)
DFRobot_AHT20 aht20;
TypeK temp_K_cal;
#endif

extern pid_setting_t pid_parm;
extern HardwareSerial Serial_HMI;

int32_t getAnalogValue(uint8_t CH);
void readAnlg1();
void readAnlg2();

void Task_Thermo_get_data(void *pvParameters)
{ // function

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;
    char temp_data_buffer_ble[BLE_BUFFER_SIZE];
    // uint8_t temp_data_buffer_ble_out[BLE_BUFFER_SIZE];
    const TickType_t xIntervel = (pid_parm.pid_CT * 1000) / portTICK_PERIOD_MS;
    const TickType_t timeOut = 500 / portTICK_PERIOD_MS;
    int i = 0;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_add(NULL);
    while (1) // A Task shall never return or exit.
    {         // for loop
              // 喂狗
        esp_task_wdt_reset();
        // Wait for the next cycle (intervel 1500ms).
        vTaskDelayUntil(&xLastWakeTime, xIntervel);

        // step1:
        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
        {
#if defined(TC_TYPE_K)
            if (aht20.startMeasurementReady(/* crcEn = */ true))
            {
                AMB_TEMP = aht20.getTemperature_C();
                AMB_TEMP = AMB_ft.doFilter(AMB_TEMP);
                // AMB_RH = aht20.getHumidity_RH();
            }
#endif
            if (!first)
            {
                ftemps_old = ftemps; // save old filtered temps for RoR calcs
                ftimes_old = ftimes; // save old timestamps for filtered temps for RoR calcs
            }
            // delay(200);
            MCP.Configuration(1, 16, 1, 1);               // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
            Voltage = MCP.Measure();                      // Measure is stocked in array Voltage, note that the library will wait for a completed conversion that takes around 200 ms@18bits            BT_TEMP = pid_parm.BT_tempfix + (((Voltage / 1000 * Rref) / ((3.3 * 1000) - Voltage / 1000) - R0) / (R0 * 0.0039083));
            Voltage = BT_TEMP_ft.doFilter(Voltage << 10); // multiply by 1024 to create some resolution for filter
            Voltage >>= 10;
#if defined(TC_TYPE_K)
            BT_TEMP = temp_K_cal.Temp_C(Voltage * 0.001, AMB_TEMP) + pid_parm.BT_tempfix;
#else
            BT_TEMP = pid_parm.BT_tempfix + (((Voltage / 1000 * Rref) / ((3.3 * 1000) - Voltage / 1000) - R0) / (R0 * 0.0039083));
#endif

            delay(100);
            MCP.Configuration(2, 16, 1, 1);               // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
            Voltage = MCP.Measure();                      // Measure is stocked in array Voltage, note that the library will wait for a completed conversion that takes around 200 ms@18bits
            Voltage = ET_TEMP_ft.doFilter(Voltage << 10); // multiply by 1024 to create some resolution for filter
            Voltage >>= 10;

#if defined(TC_TYPE_K)
            ET_TEMP = temp_K_cal.Temp_C(Voltage * 0.001, AMB_TEMP) + pid_parm.ET_tempfix;
#else
            ET_TEMP = pid_parm.ET_tempfix + (((Voltage / 1000 * Rref) / ((3.3 * 1000) - Voltage / 1000) - R0) / (R0 * 0.0039083));
#endif

            // cal RoR
            ftimes = millis();
            ftemps = fRise.doFilter(BT_TEMP * 1000);
            if (!first)
            {
                rx = fRise.calcRise(ftemps_old, ftemps, ftimes_old, ftimes);
                ror = fRoR.doFilter(rx / D_MULT) * D_MULT;
            }

            first = false;

            // 获取 旋钮数值
            readAnlg1();
            delay(50);   // IO1
            readAnlg2(); // OT3
            // end of 获取 旋钮数值
            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }
        // step2:
        //  PID ON:ambient,chan1,chan2,  heater duty, fan duty, SV
        if (xSemaphoreTake(xThermoDataMutex, timeOut) == pdPASS) // 给温度数组的最后一个数值写入数据
        {
            // 封装BLE 协议
            sprintf(temp_data_buffer_ble, "#%4.2f,%4.2f,%4.2f,%d,%d,%4.2f;\n", AMB_TEMP, ET_TEMP, BT_TEMP, levelOT1, levelIO3, pid_sv);
            xQueueSend(queue_data_to_BLE, &temp_data_buffer_ble, xIntervel);
            xTaskNotify(xTASK_data_to_BLE, 0, eIncrement); // send notify to TASK_data_to_HMI
            memset(&temp_data_buffer_ble, '\0', BLE_BUFFER_SIZE);
            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }
        // step3:
        //  检查温度是否达到切换PID参数
#if defined(PID_AUTO_SHIFT)
        if (pid_status == true && PID_TUNNING == false)
        {
            if (BT_TEMP >= PID_TUNE_SV_1)
            {
                I2C_EEPROM.get(64, pid_parm);
                Heat_pid_controller.SetOutputLimits(PID_STAGE_2_MIN_OUT, PID_STAGE_2_MAX_OUT);
                Heat_pid_controller.SetTunings(pid_parm.p, pid_parm.i, pid_parm.d);
            }
            else if (BT_TEMP >= PID_TUNE_SV_2)
            {
                I2C_EEPROM.get(128, pid_parm);
                Heat_pid_controller.SetOutputLimits(PID_STAGE_3_MIN_OUT, PID_STAGE_3_MAX_OUT);
                Heat_pid_controller.SetTunings(pid_parm.p, pid_parm.i, pid_parm.d);
            }
        }

#endif
        // step4:
        //  检查温度是否达到降温降风
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

    } // while loop
} // function

void Task_PID_autotune(void *pvParameters)
{
    (void)pvParameters;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;
    const TickType_t xIntervel = 250 / portTICK_PERIOD_MS;
    while (1)
    {
        xResult = xTaskNotifyWait(0x00,                 // 在运行前这个命令之前，先清除这几位
                                  0x00,                 // 运行后，重置所有的bits 0x00 or ULONG_MAX or 0xFFFFFFFF
                                  &ulNotificationValue, // 重置前的notification value
                                  portMAX_DELAY);       // 一直等待

        if (xResult == pdTRUE)
        {
            // 开始 PID自动整定
            vTaskSuspend(xTASK_BLE_CMD_handle);
            for (int loop = 0; loop < 3; loop++)
            {
                if (loop == 0)
                {
                    tuner.startTuningLoop(pid_parm.pid_CT * uS_TO_S_FACTOR);
                    PID_TUNE_SV = PID_TUNE_SV_1;
                    pid_sv = PID_TUNE_SV;
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
                            levelOT1 = map(pid_tune_output, 0, 255, 0, 100);
                            pwm_heat.write(map(levelOT1, 0, 100, PWM_HEAT_MIN, PWM_HEAT_MAX));
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
                    pid_sv = PID_TUNE_SV;
                    levelIO3 = PID_TUNE_FAN_2;
                    tuner.setTuningCycles(PID_TUNE_CYCLE);
                    tuner.setOutputRange(round(PID_STAGE_2_MIN_OUT * 255 / 100), round(PID_STAGE_2_MAX_OUT * 255 / 100));
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
                            levelOT1 = map(pid_tune_output, 0, 255, 0, 100);
                            pwm_heat.write(map(levelOT1, 0, 100, PWM_HEAT_MIN, PWM_HEAT_MAX));
                            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
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
                    I2C_EEPROM.put(64, pid_parm);
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
                    pid_sv = PID_TUNE_SV;
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
                            levelOT1 = map(pid_tune_output, 0, 255, 0, 100);
                            pwm_heat.write(map(levelOT1, 0, 100, PWM_HEAT_MIN, PWM_HEAT_MAX));
                            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
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

                    if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
                    {
                        levelIO3 = 30;
                        levelOT1 = 0;
                        PID_TUNNING = false;
                        pid_status = false;
                        pid_sv = 0.0;
                        pwm_heat.write(map(levelOT1, 0, 100, PWM_HEAT_MIN, PWM_HEAT_MAX));
                        pwm_fan.write(map(levelIO3, MIN_IO3, MAX_IO3, PWM_FAN_MIN, PWM_FAN_MAX));
                        xSemaphoreGive(xThermoDataMutex); // end of lock mutex
                    }
                    // Get PID gains - set your PID controller's gains to these
                    pid_parm.p = tuner.getKp();
                    pid_parm.i = tuner.getKi();
                    pid_parm.d = tuner.getKd();
                    I2C_EEPROM.put(128, pid_parm);
                    // end of pid autotuning
                    Heat_pid_controller.SetMode(MANUAL);

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
    Heat_pid_controller.SetMode(MANUAL);
    PID_TUNNING = false;
    pid_status = false;
    pid_sv = 0.0;
    LCD.PCF8574_LCDClearScreen();
    LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberOne, 0);
    LCD.PCF8574_LCDSendString("PID TUNE DONE.");
    LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberTwo, 0);
    LCD.PCF8574_LCDSendString("PLS PWR OFF.");

    delay(3000);
    // vTaskResume(xTASK_BLE_CMD_handle);
    vTaskSuspend(xTask_PID_autotune);
}

int32_t getAnalogValue(uint8_t CH)
{
    int32_t mod, trial, min_anlg1, max_anlg1, min_anlg2, max_anlg2;
    min_anlg1 = MIN_OT1;
    max_anlg1 = MAX_OT1;
    min_anlg2 = MIN_IO3;
    max_anlg2 = MAX_IO3;
    float aval;
    MCP.Configuration(CH, 16, 1, 1);
    delay(100);

    aval = map(MCP.Measure(), 6750, 2047900, 0, 1023);

    if (CH == anlg1)
    {
        aval = min_anlg1 * 10.24 + (aval / 1024) * 10.24 * (max_anlg1 - min_anlg1); // scale analogue value to new range
        if (aval == (min_anlg1 * 10.24))
            aval = 0; // still allow OT1 to be switched off at minimum value. NOT SURE IF THIS FEATURE IS GOOD???????
        mod = min_anlg1;
        trial = (aval + 0.001) * 100; // to fix weird rounding error from previous calcs?????
        trial /= 1023;
        trial = (trial / DUTY_STEP) * DUTY_STEP; // truncate to multiple of DUTY_STEP
        if (trial < mod)
            trial = 0;
    }
    if (CH == anlg2)
    {
        aval = min_anlg2 * 10.24 + (aval / 1024) * 10.24 * (max_anlg2 - min_anlg2); // scale analogue value to new range
        if (aval == (min_anlg2 * 10.24))
            aval = 0; // still allow OT2 to be switched off at minimum value. NOT SURE IF THIS FEATURE IS GOOD???????
        mod = min_anlg2;
        trial = (aval + 0.001) * 100; // to fix weird rounding error from previous calcs?????
        trial /= 1023;
        trial = (trial / DUTY_STEP) * DUTY_STEP; // truncate to multiple of DUTY_STEP
        if (trial < mod)
            trial = MIN_IO3;
    }

    return trial;
}

// ---------------------------------
void readAnlg1()
{ // read analog port 1 and adjust OT1 output
    char pstr[5];
    int32_t reading;
    if (pid_status == false)
    {
        reading = getAnalogValue(anlg1);
        if (reading <= 100 && reading != old_reading_anlg1)
        {                                // did it change?
            old_reading_anlg1 = reading; // save reading for next time
            levelOT1 = reading;
            pwm_heat.write(map(levelOT1, 0, 100, PWM_HEAT_MIN, PWM_HEAT_MAX));
        }
    }
}

// ---------------------------------
void readAnlg2()
{ // read analog port 2 and adjust OT2 output
    char pstr[5];
    int32_t reading;
    if (pid_status == false)
    {
        reading = getAnalogValue(anlg2);
        if (reading <= 100 && reading != old_reading_anlg2)
        {                                // did it change?
            old_reading_anlg2 = reading; // save reading for next time
            levelIO3 = reading;
            pwm_fan.write(map(levelIO3, MIN_IO3, MAX_IO3, PWM_FAN_MIN, PWM_FAN_MAX));
        }
    }
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