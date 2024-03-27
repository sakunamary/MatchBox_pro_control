
#ifndef __TASK_BLE_SERIAL_H__
#define __TASK_BLE_SERIAL_H__

#include <config.h>
#include <BleSerial.h>
#include <esp_attr.h>
#include <esp_task_wdt.h>
#include <driver/rtc_io.h>
#include "soc/rtc_wdt.h"


BleSerial SerialBLE;


const int BLE_BUFFER_SIZE = 1024;

void TASK_CMD_From_BLE(void *pvParameters)
{
    const TickType_t timeOut = 1000;
    while (true)
    {
        if (SerialBLE.available())
        {
            auto count = SerialBLE.readBytes(bleReadBuffer, BUFFER_SIZE);

            memset(bleReadBuffer, '\0', sizeof(bleReadBuffer));
        }
        vTaskDelay(20);
    }
}



// //Task for reading Serial Port
// void ReadSerialTask(void *e) {
//   while (true) {
//     if (Serial.available()) {
//       auto count = Serial.readBytes(serialReadBuffer, BUFFER_SIZE);
//       SerialBT.write(serialReadBuffer, count);
//     }
//     delay(20);
//   }
// }

// //Task for reading BLE Serial
// void ReadBtTask(void *e) {
//   while (true) {
//     if (SerialBT.available()) {
//       auto count = SerialBT.readBytes(bleReadBuffer, BUFFER_SIZE);
//       Serial.write(bleReadBuffer, count);
//     }
//     delay(20);
//   }
// }


#endif