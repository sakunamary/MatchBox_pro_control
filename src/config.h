
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <Wire.h>

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 115200        // serial port baudrate
#define HMI_BAUDRATE 9600        // serial port baudrate

//#define DEBUG_MODE
#define BLE_BUFFER_SIZE 128
#define HMI_BUFFER_SIZE 17

#define VERSION "1.1.9"

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

#define PID_MAX_OUT 100
#define PID_MIN_OUT 10

// -------------------------- slew rate limitations for fan control
#define MAX_SLEW 25                                           // percent per second
#define SLEW_STEP 3                                           // increase in steps of 5% for smooth transition
#define SLEW_STEP_TIME (uint32_t)(SLEW_STEP * 500 / MAX_SLEW) // min ms delay between steps
#define DUTY_STEP 2                                         // Use 1, 2, 4, 5, or 10.

////////////////////
// Heater and Fan Limits/Options
#define MIN_OT1 0   // Set output % for lower limit for OT1.  0% power will always be available
#define MAX_OT1 100 // Set output % for upper limit for OT1

#define MIN_IO3 30  // Set output % for lower limit for IO3.  0% power will always be available
#define MAX_IO3 100 // Set output % for upper limit for IO3

#define PID_TUNE_SV_3 200
#define PID_TUNE_SV_2 190
#define PID_TUNE_SV_1 160

#define PID_TUNE_FAN_3 50
#define PID_TUNE_FAN_2 53
#define PID_TUNE_FAN_1 55

//
typedef struct eeprom_settings
{
    double pid_CT;
    double p;
    double i;
    double d;
    double BT_tempfix;
    double ET_tempfix;
} pid_setting_t;


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"



static TaskHandle_t xTASK_data_to_HMI = NULL;
static TaskHandle_t xTASK_CMD_FROM_HMI = NULL;
static TaskHandle_t xTASK_HMI_CMD_handle = NULL;
static TaskHandle_t xTASK_data_to_BLE = NULL;
static TaskHandle_t xTASK_BLE_CMD_handle = NULL;
static TaskHandle_t xTask_Thermo_get_data = NULL;
static TaskHandle_t xTask_PID_autotune = NULL;



SemaphoreHandle_t xThermoDataMutex = NULL;
SemaphoreHandle_t xDATA_OUT_Mutex = NULL;


QueueHandle_t queue_data_to_HMI = xQueueCreate(15, HMI_BUFFER_SIZE); // 发送到HMI的数据 hex格式化数据
QueueHandle_t queueCMD_HMI = xQueueCreate(15, sizeof(uint8_t[HMI_BUFFER_SIZE]));      // 从HMI接收到的Hex格式命令
QueueHandle_t queueCMD_BLE = xQueueCreate(8, sizeof(char[BLE_BUFFER_SIZE]));
QueueHandle_t queue_data_to_BLE = xQueueCreate(8, sizeof(char[BLE_BUFFER_SIZE]));


#endif

// printh 00 00 00 ff ff ff 88 ff ff ff//输出上电信息到串口
// 69 ff 00 ff ff ff 69 69 69 67 67 67 ff ff ff ff ff //握手协议

// HMI --> MatchBox的数据帧 FrameLenght = 17
// 帧头: 69 FF
// 类型: 01 温度数据
// 环境温： 00 00 //uint16
// 温度1: 00 00 // uint16
// 温度2: 00 00 // uint16
// PID SV: 00 00 // uint16
// 火力 : 00
// 风力 : 00
// PID RUN: 00
// 帧尾:FF FF FF

// HMI --> MatchBox的数据帧 FrameLenght = 17
// 帧头: 69 FF
// 类型: 02 PID设定
// BT fix: 00 00 // uint16
// ET fix: 00 00 // uint16
// P: 00 00 // uint16
// I: 00 00 // uint16
// D: 00 00 // uint16
// PID ct: 00
// 帧尾:FF FF FF