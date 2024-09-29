
#ifndef __CONFIG_H__
#define __CONFIG_H__
//MATCH BOX MINI 
#include <Wire.h>

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 115200        // serial port baudrate
#define VERSION "1.1.2"
//  DEBUG_MODE 会在串口输出用于调试的测试反馈信息
//#define DEBUG_MODE 



// 下面代码不要动，主板硬件IO对应。已测试。
#define BLE_BUFFER_SIZE 64
// #define HMI_BUFFER_SIZE 16
#define SPI_SCK 8
#define SPI_MISO 9
#define SPI_MOSI 10

#define SPI_CS_BT 4
#define SPI_CS_ET 3

#define I2C_SCL 7
#define I2C_SDA 6

#define TXD_HMI 21
#define RXD_HMI 20

#define PWM_FAN 5
#define PWM_HEAT 2

// 上面代码不要动，主板硬件IO对应。已测试。



// pwm setting
#define PWM_FREQ 1000     //PWM 信号频率 单位：Hz
#define PWM_RESOLUTION 10 // 0-1024 PWM 信号分辨率 10bit 是0-1024级 ，12bit 是 0-4096级，改了主程序的分辨率输出map函数也要相应调整

#define PWM_FAN_MIN 250
#define PWM_HEAT_MIN 1

#define PWM_FAN_MAX 1000  //最大值，对应10bit的1024 ，pwm满载需要保留一点空隙。不能到1024
#define PWM_HEAT_MAX 1000 //最大值，对应10bit的1024 ，pwm满载需要保留一点空隙。不能到1024

// -------------------------- slew rate limitations for fan control 风门缓降参数设置
#define MAX_SLEW 25                                           // percent per second
#define SLEW_STEP 3                                           // increase in steps of 5% for smooth transition
#define SLEW_STEP_TIME (uint32_t)(SLEW_STEP * 500 / MAX_SLEW) // min ms delay between steps
#define DUTY_STEP 2                                         // Use 1, 2, 4, 5, or 10.


//PID自动整定 三个阶段的温度设置
#define PID_TUNE_SV_1 160
#define PID_TUNE_SV_2 190
#define PID_TUNE_SV_3 200

#define PID_TUNE_FAN_1 55
#define PID_TUNE_FAN_2 50
#define PID_TUNE_FAN_3 45


////////////////////
// Heater and Fan Limits/Options
#define MIN_OT1 0   // Set output % for lower limit for OT1.  0% power will always be available
#define MAX_OT1 100 // Set output % for upper limit for OT1

#define MIN_IO3 30  // Set output % for lower limit for IO3.  0% power will always be available
#define MAX_IO3 100 // Set output % for upper limit for IO3

#define PID_MAX_OUT 100 //0-100 ，跟OT3 IO1的数值一样
#define PID_MIN_OUT 10  //0-100 ，跟OT3 IO1的数值一样

//以下代码不要动，FreeRTOS用的代码
typedef struct eeprom_settings
{
    double pid_CT;
    double p;
    double i;
    double d;
    double BT_tempfix;
    double ET_tempfix;
} pid_setting_t;


#define TWDT_TIMEOUT_S 3


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

 static TaskHandle_t xTASK_data_to_BLE = NULL;
static TaskHandle_t xTask_PID_autotune = NULL;

static TaskHandle_t xTASK_BLE_CMD_handle = NULL;

SemaphoreHandle_t xThermoDataMutex = NULL;
SemaphoreHandle_t xSerialReadBufferMutex = NULL;
// SemaphoreHandle_t xContrlDataMutex = NULL;

QueueHandle_t queueCMD_BLE = xQueueCreate(8, sizeof(char[BLE_BUFFER_SIZE]));
QueueHandle_t queue_data_to_BLE = xQueueCreate(8, sizeof(char[BLE_BUFFER_SIZE]));

//以上代码不要动，FreeRTOS用的代码
#endif