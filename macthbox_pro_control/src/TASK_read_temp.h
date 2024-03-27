#ifndef __TASK_READ_TEMP_H__
#define __TASK_READ_TEMP_H__

#include <Arduino.h>
#include <Wire.h>


#include <Adafruit_MAX31865.h>
#include <ModbusIP_ESP8266.h>

double BT_TEMP;
double ET_TEMP;

SemaphoreHandle_t xThermoDataMutex = NULL;

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo_BT = Adafruit_MAX31865(SPI_CS_CT, SPI_MOSI, SPI_MISO, SPI_SCK);
Adafruit_MAX31865 thermo_ET = Adafruit_MAX31865(SPI_CS_ET, SPI_MOSI, SPI_MISO, SPI_SCK);


// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0


// Modbus Registers Offsets
const uint16_t BT_HREG = 3001;
const uint16_t ET_HREG = 3002;

void Task_Thermo_get_data(void *pvParameters)
{ // function

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;

    const TickType_t xIntervel = 1500 / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    for (;;) // A Task shall never return or exit.
    {        // for loop
        // Wait for the next cycle (intervel 1500ms).
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
        {                                                          // lock the  mutex
            // 读取max6675数据
            BT_TEMP = round((thermo_BT.readCelsius() * 10) / 10);
            vTaskDelay(20);
            ET_TEMP = round((thermo_ET.readCelsius() * 10) / 10);
            vTaskDelay(20);
            mb.Hreg(BT_HREG, BT_TEMP);        // 更新赋值
            mb.Hreg(ET_HREG, ET_TEMP);        // 更新赋值
            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }
    }

} // function

#endif