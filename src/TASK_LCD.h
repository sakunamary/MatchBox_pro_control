#ifndef __TASK_BLE_SERIAL_H__
#define __TASK_BLE_SERIAL_H__

#include <Arduino.h>
#include <Wire.h>
#include <config.h>
#include <LiquidCrystal_I2C.h>

// const uint16_t BT_HREG = 3001;
// const uint16_t ET_HREG = 3002;
// const uint16_t AMB_RH_HREG = 3007;
// const uint16_t AMB_TEMP_HREG = 3008;

// // Modbus Registers Offsets
// const uint16_t HEAT_HREG = 3003;
// const uint16_t FAN_HREG = 3004;
// const uint16_t PID_STATUS_HREG = 3005;
// const uint16_t PID_SV_HREG = 3006;

// double BT_TEMP;
// double ET_TEMP;
// double AMB_RH;
// double AMB_TEMP;
// Modbus Registers Offsets
// const uint16_t HEAT_HREG = 3003;
// const uint16_t FAN_HREG = 3004;
// const uint16_t PID_STATUS_HREG = 3005;
// const uint16_t PID_SV_HREG = 3006;

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

void Task_OLED(void *pvParameters)
{ // function

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xIntervel = 1000 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();
    String ver = VERSION;
    
    lcd.backlight();
    lcd.setCursor(4, 0);
    lcd.print(F("MatchBox Pro"));
    lcd.setCursor(4, 1);
    lcd.print(F(ver));



    for (;;) // A Task shall never return or exit.
    {
        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xIntervel);

        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // Mutex to make the data more clean
        {
            // lcd.backlight();
            // lcd.setCursor(3,0);
            // lcd.print("Hello, world!");
            xSemaphoreGive(xThermoDataMutex);
        }
    }
}

#endif
