
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <Wire.h>

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 115200        // serial port baudrate

#define DEBUG_MODE
#define BLE_BUFFER_SIZE 64
#define HMI_BUFFER_SIZE 16

#define VERSION "1.0.5"

#define SPI_SCK 8
#define SPI_MISO 9
#define SPI_MOSI 10

#define SPI_CS_BT 4
#define SPI_CS_ET 3

#define I2C_SCL 7
#define I2C_SDA 6

#define TXD_HMI 21
#define RXD_HMI 20

// pwm setting
#define PWM_FAN 5
#define PWM_HEAT 2
#define PWM_FREQ 3922
#define PWM_RESOLUTION 10 // 0-1024

#define PID_MAX_OUT 80
#define PID_MIN_OUT 30

//
typedef struct eeprom_settings
{
    long pid_CT;
    double p;
    double i;
    double d;
    double BT_tempfix;
    double ET_tempfix;
} pid_setting_t;

// publc funciton

// uint8_t make_frame_head(uint8_t data_array[HMI_BUFFER_SIZE], int cmd_type)
// // pagkage the data frame end .cmd_type:1/data_frame;2/run_status;3/HMI_cmd
// {
//     data_array[0] = 0x69; // frame head
//     data_array[1] = 0xff; // frame head

//     switch (cmd_type)
//     {
//     case 1:                   // data_frame
//         data_array[2] = 0x01; // data type
//         break;
//     case 2:                   // run_status
//         data_array[2] = 0x02; // data type
//         break;
//     case 3:                   // run_status
//         data_array[2] = 0x03; // data type
//         break;
//     case 4:                   // run_status
//         data_array[2] = 0x04; // data type
//         break;
//     default:
//         break;
//     }
//     return data_array[HMI_BUFFER_SIZE];
// }

// uint8_t make_frame_end(uint8_t data_array[HMI_BUFFER_SIZE], int cmd_type)
// // pagkage the data frame end .cmd_type:1/data_frame;2/run_status;3/HMI_cmd
// {

//     switch (cmd_type)
//     {
//     case 1: // data_frame

//         data_array[9] = 0xff;  // frame end
//         data_array[10] = 0xff; // frame end
//         data_array[11] = 0xff; // frame end
//         break;
//     case 2:                    // run_status
//         data_array[9] = 0xff;  // frame end
//         data_array[10] = 0xff; // frame end
//         data_array[11] = 0xff; // frame end
//         break;
//     case 3:                    // run_status
//         data_array[9] = 0xff;  // frame end
//         data_array[10] = 0xff; // frame end
//         data_array[11] = 0xff; // frame end
//         break;
//     case 4:   
//         data_array[7] = 0x00;  // frame end
//         data_array[8] = 0x00; // frame end
//         data_array[9] = 0xff;  // frame end
//         data_array[10] = 0xff; // frame end
//         data_array[11] = 0xff; // frame end
//         break;
//     default:
//         break;
//     }
//     return data_array[HMI_BUFFER_SIZE];
// }

// uint8_t make_frame_data(uint8_t data_array[HMI_BUFFER_SIZE], int cmd_type, uint16_t in_val, int uBit)
// // pagkage the data frame.cmd_type:1/data_frame;2/run_status;3/HMI_cmd
// {
//     uint8_t high = highByte(in_val);
//     uint8_t low = lowByte(in_val);
//     switch (cmd_type)
//     {
//     case 1:
//         if (uBit > 2 && uBit < 7)
//         {
//             data_array[uBit] = low;      // frame end
//             data_array[uBit + 1] = high; // frame end
//         }

//         break;
//     case 2:
//         if (uBit > 2 && uBit < 9)
//         {
//             data_array[uBit] = low;      // frame end
//             data_array[uBit + 1] = high; // frame end
//         }
//         break;
//     case 3:
//         if (uBit > 2 && uBit < 9)
//         {
//             data_array[uBit] = low;      // frame end
//             data_array[uBit + 1] = high; // frame end
//         }
//         break;

//     default:

//         break;
//     }
//     return data_array[HMI_BUFFER_SIZE];
// }

// HMI --> MatchBox的数据帧 FrameLenght = 12
// 帧头: 69 FF
// 类型: 01 温度数据
// 温度1: 00 00 // uint16
// 温度2: 00 00 // uint16
// 火力 : 00
// 风力 : 00
// 帧尾:FF FF FF

// HMI --> MatchBox的数据帧 FrameLenght = 12
// 帧头: 69 FF
// 类型: 02 PID设定
// P: 00 00 // uint16
// I: 00 00 // uint16
// D: 00 00 // uint16
// 帧尾:FF FF FF

// HMI --> MatchBox的数据帧 FrameLenght = 12
// 帧头: 69 FF
// 类型: 03 参数设定
// PID ct: 00 00 // uint16
// BT fix: 00 00 // uint16
// ET fix: 00 00 // uint16
// 帧尾:FF FF FF

// HMI --> MatchBox的数据帧 FrameLenght = 12
// 帧头: 69 FF
// 类型: 04 PID run
// PID SV: 00 00 // uint16
// PID STATUS: 00
// PID TUNE : 00
// NULL : 00 00
// 帧尾:FF FF FF

// static TaskHandle_t xTASK_data_to_HMI = NULL;
// static TaskHandle_t xTASK_CMD_HMI = NULL;
// static TaskHandle_t xTASK_HMI_CMD_handle = NULL;
 static TaskHandle_t xTASK_data_to_BLE = NULL;
// static TaskHandle_t xTASK_CMD_BLE = NULL;
static TaskHandle_t xTASK_BLE_CMD_handle = NULL;

SemaphoreHandle_t xThermoDataMutex = NULL;
SemaphoreHandle_t xSerialReadBufferMutex = NULL;

QueueHandle_t queue_data_to_HMI = xQueueCreate(15, sizeof(uint8_t[HMI_BUFFER_SIZE])); // 发送到HMI的数据 hex格式化数据
QueueHandle_t queueCMD_HMI = xQueueCreate(15, sizeof(uint8_t[HMI_BUFFER_SIZE]));      // 从HMI接收到的Hex格式命令
QueueHandle_t queueCMD_BLE = xQueueCreate(8, sizeof(char[BLE_BUFFER_SIZE]));
QueueHandle_t queue_data_to_BLE = xQueueCreate(8, sizeof(char[BLE_BUFFER_SIZE]));

#endif