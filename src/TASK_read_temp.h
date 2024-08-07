#ifndef __TASK_READ_TEM_H__
#define __TASK_READ_TEMP_H__

#include <Arduino.h>
#include <config.h>
#include <Wire.h>
#include <MCP3424.h>
#include "DFRobot_AHT20.h"


double BT_TEMP;
double ET_TEMP;
double AMB_RH;
double AMB_TEMP;
extern int levelOT1;
extern int levelIO3;
extern double pid_sv;
extern bool pid_status;
extern pid_setting_t pid_parm;


// Need this for the lower level access to set them up.
uint8_t address = 0x68;
long Voltage; // Array used to store results

#define R0 100
#define Rref 1000

MCP3424 MCP(address); // Declaration of MCP3424 A2=0 A1=1 A0=0
DFRobot_AHT20 aht20;
// TypeK temp_K_cal;
extern ExternalEEPROM I2C_EEPROM;
extern ArduPID Heat_pid_controller;

void Task_Thermo_get_data(void *pvParameters)
{ // function

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;
    char temp_data_buffer_ble[BLE_BUFFER_SIZE];
    uint8_t TEMP_DATA_Buffer[HMI_BUFFER_SIZE];
    const TickType_t xIntervel = (pid_parm.pid_CT * 1000) / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    for (;;) // A Task shall never return or exit.
    {        // for loop
        // Wait for the next cycle (intervel 1500ms).
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
        {

            if (aht20.startMeasurementReady(/* crcEn = */ true))
            {
                AMB_TEMP = aht20.getTemperature_C();
                AMB_RH = aht20.getHumidity_RH();
                // #if defined(DEBUG_MODE)
                //                 Serial.printf("raw data:AMB_TEMP:%4.2f\n", AMB_TEMP);
                // #endif
            }
            delay(200);
            MCP.Configuration(1, 16, 1, 1); // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
            Voltage = MCP.Measure();        // Measure is stocked in array Voltage, note that the library will wait for a completed conversion that takes around 200 ms@18bits
            BT_TEMP = pid_parm.BT_tempfix + (((Voltage / 1000 * Rref) / ((3.3 * 1000) - Voltage / 1000) - R0) / (R0 * 0.0039083));
            delay(200);
            MCP.Configuration(2, 16, 1, 1); // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
            Voltage = MCP.Measure();        // Measure is stocked in array Voltage, note that the library will wait for a completed conversion that takes around 200 ms@18bits
            ET_TEMP = pid_parm.ET_tempfix + (((Voltage / 1000 * Rref) / ((3.3 * 1000) - Voltage / 1000) - R0) / (R0 * 0.0039083));

            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }

        if (pid_status)
        {
            if(BT_TEMP >= PID_TUNE_SV_1 )  {
                 I2C_EEPROM.get(1, pid_parm);
                 Heat_pid_controller.setCoefficients(pid_parm.p,pid_parm.i,pid_parm.d);


            }else  if(BT_TEMP >= PID_TUNE_SV_2 ) {
                I2C_EEPROM.get(2, pid_parm);
                Heat_pid_controller.setCoefficients(pid_parm.p,pid_parm.i,pid_parm.d);
            }
        }


        // 封装BLE 协议
        // PID ON:ambient,chan1,chan2,  heater duty, fan duty, SV
        if (xSemaphoreTake(xSerialReadBufferMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
        {
            sprintf(temp_data_buffer_ble, "#%4.2f,%4.2f,%4.2f,%d,%d,%4.2f;\n", AMB_TEMP, ET_TEMP, BT_TEMP, levelOT1, levelIO3, pid_sv);
           // Serial.print(temp_data_buffer_ble);
            xQueueSend(queue_data_to_BLE, &temp_data_buffer_ble, xIntervel);
            xTaskNotify(xTASK_data_to_BLE, 0, eIncrement); // send notify to TASK_data_to_HMI
        }
        xSemaphoreGive(xSerialReadBufferMutex); // end of lock mutex
    }

} // function

#endif
