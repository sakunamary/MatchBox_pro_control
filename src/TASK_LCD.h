#ifndef __TASK_LCD_H__
#define __TASK_LCD_H__
#include <Arduino.h>
#include <config.h>
#include "HD44780_LCD_PCF8574.h"

#define DISPLAY_DELAY_1 1000
#define DISPLAY_DELAY_2 2000
#define DISPLAY_DELAY 5000

HD44780LCD LCD(4, 20, 0x27, &Wire); // instantiate an object

double BT_TEMP_LCD;
double ET_TEMP_LCD;
double ror_LCD;
int levelOT1_LCD;
int levelIO3_LCD;
double pid_sv_LCD;

extern int levelOT1;
extern int levelIO3;
extern bool pid_status;
extern double BT_TEMP;
extern double ET_TEMP;
extern double AMB_RH;
extern double AMB_TEMP;

char line1[20];
char line2[20];
char line3[20];
char line4[20];

void TASK_LCD(void *pvParameters)
{
    (void)pvParameters;
    char temp_data_buffer_ble[BLE_BUFFER_SIZE];
    const TickType_t xIntervel = 1500 / portTICK_PERIOD_MS;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;
    TickType_t xLastWakeTime;
    const TickType_t timeOut = 250 / portTICK_PERIOD_MS;
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

        if (xSemaphoreTake(xLCDDataMutex, timeOut) == pdPASS) // 给温度数组的最后一个数值写入数据
        {
            BT_TEMP_LCD = BT_TEMP;
            ET_TEMP_LCD = ET_TEMP;
            ror_LCD = ror;
            levelOT1_LCD = levelOT1;
            levelIO3_LCD = levelIO3;
            pid_sv_LCD = pid_sv;
            xSemaphoreGive(xLCDDataMutex); // end of lock mutex
        }

        if (xSemaphoreTake(xThermoDataMutex, timeOut) == pdPASS) // 给温度数组的最后一个数值写入数据
        {
            // 封装BLE 协议
            sprintf(temp_data_buffer_ble, "#0.00,%4.2f,%4.2f,%d,%d;\n", ET_TEMP, BT_TEMP, levelOT1, levelIO3);
            // sprintf(temp_data_buffer_ble, "#0.00,%4.2f,%4.2f,%d,%d,%f4.0f,%4.2f;\n", ET_TEMP, BT_TEMP, levelOT1, levelIO3,ror,pid_sv);
            xQueueSend(queue_data_to_BLE, &temp_data_buffer_ble, xIntervel);
            xTaskNotify(xTASK_data_to_BLE, 0, eIncrement); // send notify to TASK_data_to_HMI
            memset(&temp_data_buffer_ble, '\0', BLE_BUFFER_SIZE);
            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }

        if (xSemaphoreTake(xLCDDataMutex, timeOut) == pdPASS) // 给温度数组的最后一个数值写入数据
        {

            if (pid_status == true && PID_TUNNING == true) // 自动整定情况
            {
                // LCD.PCF8574_LCDClearLine(LCD.LCDLineNumberOne);
                sprintf(line1, "MODE:TUNE    ET:%4d", (int)round(ET_TEMP_LCD));
                LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberOne, 0);
                LCD.PCF8574_LCDSendString(line1);
                // LCD.PCF8574_LCDClearLine(LCD.LCDLineNumberThree);
                sprintf(line3, "HTR:%4d     SV:%4d", (int)round(levelOT1_LCD), (int)round(pid_sv_LCD));
                LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberThree, 0);
                LCD.PCF8574_LCDSendString(line3);
            }
            else if (pid_status == true && PID_TUNNING == false) // PID跑线情况
            {
                // LCD.PCF8574_LCDClearLine(LCD.LCDLineNumberOne);
                sprintf(line1, "MODE:PID     ET:%4d", (int)round(ET_TEMP_LCD));
                LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberOne, 0);
                LCD.PCF8574_LCDSendString(line1);
                // LCD.PCF8574_LCDClearLine(LCD.LCDLineNumberThree);
                sprintf(line3, "HTR:%4d     SV:%4d", (int)round(levelOT1_LCD), (int)round(pid_sv_LCD));
                LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberThree, 0);
                LCD.PCF8574_LCDSendString(line3);
            }
            else
            {
                // LCD.PCF8574_LCDClearLine(LCD.LCDLineNumberOne);
                sprintf(line1, "MODE:MAN     ET:%4d", (int)round(ET_TEMP_LCD));
                LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberOne, 0);
                LCD.PCF8574_LCDSendString(line1);
                // LCD.PCF8574_LCDClearLine(LCD.LCDLineNumberThree);
                sprintf(line3, "HTR:%4d ", (int)round(levelOT1_LCD));
                LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberThree, 0);
                LCD.PCF8574_LCDSendString(line3);
            }
            xSemaphoreGive(xLCDDataMutex); // end of lock mutex
        }

        if (xSemaphoreTake(xLCDDataMutex, timeOut) == pdPASS) // 给温度数组的最后一个数值写入数据
        {
            sprintf(line2, "RoR:%4d     BT:%4d", (int)round(ror_LCD), (int)round(BT_TEMP_LCD));
            LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberTwo, 0);
            LCD.PCF8574_LCDSendString(line2);
            sprintf(line4, "FAN:%4d  ", (int)round(levelIO3_LCD));
            LCD.PCF8574_LCDGOTO(LCD.LCDLineNumberFour, 0);
            LCD.PCF8574_LCDSendString(line4);
            xSemaphoreGive(xLCDDataMutex); // end of lock mutex
        }
    }
}

#endif