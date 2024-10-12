#ifndef __TASK_LCD_H__
#define __TASK_LCD_H__
#include <Arduino.h>
#include <config.h>
#include "HD44780_LCD_PCF8574.h"

#define DISPLAY_DELAY_1 1000
#define DISPLAY_DELAY_2 2000
#define DISPLAY_DELAY 5000

HD44780LCD LCD(4, 20, 0x27, &Wire); // instantiate an object

extern double BT_TEMP;
extern double ET_TEMP;
extern int levelOT1;
extern int levelIO3;
extern double pid_sv;
extern bool pid_status;
extern double ror;
char line1[20];
char line2[20];
char line3[20];
char line4[20];

void TASK_LCD(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t xIntervel = 1000 / portTICK_PERIOD_MS;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;
    TickType_t xLastWakeTime;
    const TickType_t timeOut = 200 / portTICK_PERIOD_MS;
    // for start banner
    LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberOne, 0);
    LCD.PCF8574_LCDSendString(BANNER);
    LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberTwo, 0);
    LCD.PCF8574_LCDSendString(VERSION);
    vTaskDelayUntil(&xLastWakeTime, 2000 / portTICK_PERIOD_MS);
    LCD.PCF8574_LCDClearScreen();

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);

        if (pid_status)
        {

            if (xSemaphoreTake(xThermoDataMutex, timeOut) == pdPASS) // 给温度数组的最后一个数值写入数据
            {
                LCD.PCF8574_LCDClearLine(LCD.LCDLineNumberOne);
                sprintf(line1, "MODE:PID     ET:%4d", (int)round(ET_TEMP));
                LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberOne, 0);
                LCD.PCF8574_LCDSendString(line1);
                LCD.PCF8574_LCDClearLine(LCD.LCDLineNumberThree);
                sprintf(line3, "HTR:%3d      SV:%4d", (int)round(levelOT1), (int)round(pid_sv));
                LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberThree, 0);
                LCD.PCF8574_LCDSendString(line3);
                xSemaphoreGive(xThermoDataMutex); // end of lock mutex
            }
        }
        else // pid_status = false
        {
            if (xSemaphoreTake(xThermoDataMutex, timeOut) == pdPASS) // 给温度数组的最后一个数值写入数据
            {
                LCD.PCF8574_LCDClearLine(LCD.LCDLineNumberOne);
                sprintf(line1, "MODE:MAN     ET:%4d", (int)round(ET_TEMP));
                LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberOne, 0);
                LCD.PCF8574_LCDSendString(line1);

                LCD.PCF8574_LCDClearLine(LCD.LCDLineNumberThree);
                sprintf(line3, "HTR:%3d ", (int)round(levelOT1));
                LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberThree, 0);
                LCD.PCF8574_LCDSendString(line3);
                xSemaphoreGive(xThermoDataMutex); // end of lock mutex
            }
        }

        if (xSemaphoreTake(xThermoDataMutex, timeOut) == pdPASS) // 给温度数组的最后一个数值写入数据
        {
            sprintf(line2, "RoR:%3d      BT:%4d", (int)round(ror), (int)round(BT_TEMP));
            LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberTwo, 0);
            LCD.PCF8574_LCDSendString(line2);
            sprintf(line4, "FAN:%3d  ", (int)round(levelIO3));
            LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberFour, 0);
            LCD.PCF8574_LCDSendString(line4);
            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }
    }
}

#endif