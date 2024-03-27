#ifndef __TASK_READ_TEMP_H__
#define __TASK_READ_TEMP_H__

#include <Arduino.h>
#include <Wire.h>


#include <Adafruit_MAX31865.h>
#include <ModbusIP_ESP8266.h>

double BT_TEMP;
double ET_TEMP;

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo_BT = Adafruit_MAX31865(SPI_CS_CT, SPI_MOSI, SPI_MISO, SPI_SCK);
Adafruit_MAX31865 thermo_ET = Adafruit_MAX31865(SPI_CS_ET, SPI_MOSI, SPI_MISO, SPI_SCK);




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

            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }

        // update  Hreg data
        mb.Hreg(BT_HREG, int(round(BT_TEMP * 10)));       // 初始化赋值
        mb.Hreg(ET_HREG, int(round(ET_HREG * 10))); // 初始化赋值

        make_frame_head(TEMP_DATA_Buffer, 1);
        make_frame_end(TEMP_DATA_Buffer, 1);
        make_frame_data(TEMP_DATA_Buffer, 1, int(round(BT_TEMP * 10)), 3);
        make_frame_data(TEMP_DATA_Buffer, 1, int(round(ET_HREG * 10)), 5);
        xQueueSend(queue_data_to_HMI, &TEMP_DATA_Buffer, xIntervel / 3);
        // send notify to TASK_data_to_HMI
        xTaskNotify(xTASK_data_to_HMI, 0, eIncrement);
    }

} // function

#endif