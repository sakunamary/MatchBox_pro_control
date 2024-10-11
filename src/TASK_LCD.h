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
int ror = 20;
char str1[6];

void TASK_LCD(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t xIntervel = 1000 / portTICK_PERIOD_MS;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;
    TickType_t xLastWakeTime;
    const TickType_t timeOut = 500 / portTICK_PERIOD_MS;
    // for start banner
    LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberOne, 0);
    LCD.PCF8574_LCDSendString(BANNER);
    LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberTwo, 0);
    LCD.PCF8574_LCDSendString(VERSION);
    vTaskDelayUntil(&xLastWakeTime, 3000 / portTICK_PERIOD_MS);
    LCD.PCF8574_LCDClearScreen();
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xSemaphoreTake(xThermoDataMutex, timeOut) == pdPASS) // 给温度数组的最后一个数值写入数据
        {
            LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberOne, 0);
            LCD.PCF8574_LCDSendString("MODE:");

            LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberOne, 5);

            if (pid_status)
            {
                LCD.PCF8574_LCDSendString("PID");
                //     sprintf(str1, "SV:%4d", (int)round(pid_sv));
                //     LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberThree, 13);
                //     LCD.PCF8574_LCDSendString(str1);
                }
                else
                {
                    LCD.PCF8574_LCDSendString("MAN");
                }
                // sprintf(str1, "ET:%4d", (int)round(ET_TEMP));
                // LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberOne, 13);
                // LCD.PCF8574_LCDSendString(str1);

                // sprintf(str1, "BT:%4d", (int)round(BT_TEMP));
                // LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberTwo, 13);
                // LCD.PCF8574_LCDSendString(str1);

                sprintf(str1, "RoR:%4d", ror);
                LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberTwo, 0);
                LCD.PCF8574_LCDSendString(str1);

                // sprintf(str1, "HTR:%4d", levelOT1);
                // LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberThree, 0);
                // LCD.PCF8574_LCDSendString(str1);
                // LCD.PCF8574_LCDSendChar('%');

                // sprintf(str1, "FAN:%4d", levelIO3);
                // LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberFour, 0);
                // LCD.PCF8574_LCDSendString(str1);
                // LCD.PCF8574_LCDSendChar('%');
                xSemaphoreGive(xThermoDataMutex); // end of lock mutex
            }
        }
    }

#endif