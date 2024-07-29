
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <Wire.h>

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 115200        // serial port baudrate

#define DEBUG_MODE
#define BLE_BUFFER_SIZE 128
#define HMI_BUFFER_SIZE 17

#define VERSION "1.2.3"

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
#define PID_MIN_OUT 0
#define PID_TUNE_SV_1 150.0
#define PID_TUNE_SV_2 180.0
#define PID_TUNE_SV_3 200.0

#define ADC_BIT 16
#define LOCATION_SETTINGS 0
#define R0 100
#define Rref 1000
//
typedef struct eeprom_settings
{
    int pid_CT;
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


// static TaskHandle_t xTASK_data_to_HMI = NULL;
// static TaskHandle_t xTASK_CMD_HMI = NULL;
// static TaskHandle_t xTASK_HMI_CMD_handle = NULL;
 static TaskHandle_t xTASK_data_to_BLE = NULL;
static TaskHandle_t xTASK_BLE_CMD_handle = NULL;

SemaphoreHandle_t xThermoDataMutex = NULL;
SemaphoreHandle_t xSerialReadBufferMutex = NULL;
SemaphoreHandle_t xContrlDataMutex = NULL;


// QueueHandle_t queue_data_to_HMI = xQueueCreate(15, sizeof(uint8_t[HMI_BUFFER_SIZE])); // 发送到HMI的数据 hex格式化数据
// QueueHandle_t queueCMD_HMI = xQueueCreate(15, sizeof(uint8_t[HMI_BUFFER_SIZE]));      // 从HMI接收到的Hex格式命令
QueueHandle_t queueCMD_BLE = xQueueCreate(8, sizeof(char[BLE_BUFFER_SIZE]));
QueueHandle_t queue_data_to_BLE = xQueueCreate(8, sizeof(char[BLE_BUFFER_SIZE]));




#endif