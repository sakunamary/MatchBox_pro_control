
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <Wire.h>

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 115200        // serial port baudrate
#define HMI_BAUDRATE 9600

#define DEBUG_MODE
#define HMI_BUFFER_SIZE 32

#define VERSION "1.0.1"


#define SPI_SCK D8
#define SPI_MISO D9
#define SPI_MOSI D10

#define SPI_CS_BT D2
#define SPI_CS_ET D1



#define I2C_SCL D5
#define I2C_SDA D4

#define TXD_HMI D6
#define RXD_HMI D7

#define PWM_FAN D3
#define PWM_HEAT D2

// pwm setting
#define PWM_HEAT_CHANNEL    0
#define PWM_FAN_CHANNEL    1

#define PWM_FREQ 5000
#define PWM_RESOLUTION 10 // 0-1024


// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0


// publc funciton

uint8_t make_frame_head(uint8_t data_array[HMI_BUFFER_SIZE], int cmd_type)
// pagkage the data frame end .cmd_type:1/data_frame;2/run_status;3/HMI_cmd
{
    data_array[0] = 0x69; // frame head
    data_array[1] = 0xff; // frame head

    switch (cmd_type)
    {
    case 1:                   // data_frame
        data_array[2] = 0x01; // data type
        break;
    case 2:                   // run_status
        data_array[2] = 0x02; // data type
        break;
    default:
        break;
    }
    return data_array[HMI_BUFFER_SIZE];
}

uint8_t make_frame_end(uint8_t data_array[HMI_BUFFER_SIZE], int cmd_type)
// pagkage the data frame end .cmd_type:1/data_frame;2/run_status;3/HMI_cmd
{

    switch (cmd_type)
    {
    case 1: // data_frame

        data_array[9] = 0xff; // frame end
        data_array[10] = 0xff; // frame end
        data_array[11] = 0xff; // frame end
        break;
    case 2:                    // run_status
        data_array[9] = 0xff;  // frame end
        data_array[10] = 0xff; // frame end
        data_array[11] = 0xff; // frame end
        break;
    default:
        break;
    }
    return data_array[HMI_BUFFER_SIZE];
}

uint8_t make_frame_data(uint8_t data_array[HMI_BUFFER_SIZE], int cmd_type, uint16_t in_val, int uBit)
// pagkage the data frame.cmd_type:1/data_frame;2/run_status;3/HMI_cmd
{
    uint8_t high = highByte(in_val);
    uint8_t low = lowByte(in_val);
    switch (cmd_type)
    {
    case 1:
        if (uBit > 2 && uBit < 7)
        {
            data_array[uBit] = low;      // frame end
            data_array[uBit + 1] = high; // frame end
        }

        break;
    case 2:
        if (uBit > 2 && uBit < 7)
        {
            data_array[uBit] = low; // frame end
        }
        break;
    case 3:
        if (uBit > 2 && uBit < 7)
        {
            data_array[uBit] = low; // frame end
        }
        break;

    default:
        break;
    }
    return data_array[HMI_BUFFER_SIZE];
}



// HB --> HMI的数据帧 FrameLenght = 12
// 帧头: 69 FF
// 类型: 01温度数据
// 温度1: 00 00 // uint16
// 温度2: 00 00 // uint16
// 火力 : 00
// 风力 : 00
// 帧尾:FF FF FF


static TaskHandle_t xTASK_data_to_HMI = NULL;
static TaskHandle_t xTASK_CMD_HMI = NULL;
static TaskHandle_t xTASK_HMI_CMD_handle = NULL;
static TaskHandle_t xTASK_data_to_BLE = NULL;
static TaskHandle_t xTASK_CMD_BLE = NULL;
static TaskHandle_t xTASK_BLE_CMD_handle = NULL;

SemaphoreHandle_t xThermoDataMutex = NULL;
SemaphoreHandle_t xSerialReadBufferMutex = NULL;

QueueHandle_t queue_data_to_HMI = xQueueCreate(15, sizeof(uint8_t[HMI_BUFFER_SIZE])); // 发送到HMI的数据 hex格式化数据
QueueHandle_t queueCMD_HMI = xQueueCreate(15, sizeof(uint8_t[HMI_BUFFER_SIZE]));          //从HMI接收到的Hex格式命令
QueueHandle_t queueCMD_BLE = xQueueCreate(8, sizeof(char[HMI_BUFFER_SIZE]));
QueueHandle_t queue_data_to_BLE = xQueueCreate(8, sizeof(char[HMI_BUFFER_SIZE]));


#endif