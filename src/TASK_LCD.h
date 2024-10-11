#ifndef __TASK_LCD_H__
#define __TASK_LCD_H__
#include <Arduino.h>
#include <config.h>
#include "HD44780_LCD_PCF8574.h"

#define DISPLAY_DELAY_1 1000
#define DISPLAY_DELAY_2 2000
#define DISPLAY_DELAY 5000

HD44780LCD LCD(4, 20, 0x27, &Wire); // instantiate an object

char str1[] = "BT:";
char str2[] = "ET:";
char str3[] = "";
char str4[] = "";

void TASK_LCD(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t xIntervel = 1000 / portTICK_PERIOD_MS;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;
    TickType_t xLastWakeTime;

    LCD.PCF8574_LCDGOTO(1, 0);
    LCD.PCF8574_LCDSendString(BANNER);
    LCD.PCF8574_LCDGOTO(2, 0);
    LCD.PCF8574_LCDSendString(VERSION);
    vTaskDelayUntil(&xLastWakeTime, 3000 / portTICK_PERIOD_MS);
    LCD.PCF8574_LCDClearScreen();
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberTwo, 0);
        LCD.PCF8574_LCDSendString(str1);
        LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberThree, 0);
        LCD.PCF8574_LCDSendString(str2); // Display a string
    }
}

#endif