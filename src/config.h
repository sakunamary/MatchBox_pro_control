
#ifndef __CONFIG_H__
#define __CONFIG_H__


// MATCH BOX H& 2004LCD
#include <Wire.h>

#define BANNER "MATCHBOX H7 v2"
#define VERSION "1.2.0b"

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 115200        // serial port baudrate
#define HMI_BAUDRATE 9600        // serial port baudrate

//  DEBUG_MODE 会在串口输出用于调试的测试反馈信息
//#define DEBUG_MODE
//不注释就选这Type K热电偶，注释掉就会选PT100
#define TC_TYPE_K

#define BLE_BUFFER_SIZE 64
#define HMI_BUFFER_SIZE 17

// 下面代码不要动，主板硬件IO对应。已测试。
#define SPI_SCK 8
#define SPI_MISO 9
#define SPI_MOSI 10

#define SPI_CS_BT 4
#define SPI_CS_ET 3

#define I2C_SCL 7
#define I2C_SDA 6

#define TXD_HMI 21
#define RXD_HMI 20
// 上面代码不要动，主板硬件IO对应。已测试。

// pwm setting
#define PWM_FAN 5
#define PWM_HEAT 2
#define PWM_FREQ 4000
#define PWM_RESOLUTION 10 // 0-1024


#define PWM_FAN_MIN 200
#define PWM_HEAT_MIN 100

#define PWM_FAN_MAX 1000
#define PWM_HEAT_MAX 1000

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

#define MIN_IO3 10  // Set output % for lower limit for IO3.  0% power will always be available
#define MAX_IO3 100 // Set output % for upper limit for IO3

// PID自动整定 三个阶段的温度设置
#define PID_TUNE_SV_3 190
#define PID_TUNE_SV_2 170
#define PID_TUNE_SV_1 150

#define PID_TUNE_FAN_3 45
#define PID_TUNE_FAN_2 50
#define PID_TUNE_FAN_1 55

#define PID_STAGE_1_MAX_OUT 100 // 0-100 ，跟OT3 IO1的数值一样,PID 和自整定保持一致,可以调整
#define PID_STAGE_1_MIN_OUT 5  // 0-100 ，跟OT3 IO1的数值一样,PID 和自整定保持一致,可以调整

#define PID_STAGE_2_MAX_OUT 100 // 0-100 ，跟OT3 IO1的数值一样,PID 和自整定保持一致,可以调整
#define PID_STAGE_2_MIN_OUT 5  // 0-100 ，跟OT3 IO1的数值一样,PID 和自整定保持一致,可以调整

#define PID_STAGE_3_MAX_OUT 100 // 0-100 ，跟OT3 IO1的数值一样,PID 和自整定保持一致,可以调整
#define PID_STAGE_3_MIN_OUT 5  // 0-100 ，跟OT3 IO1的数值一样,PID 和自整定保持一致,可以调整
// PID自动整定的测定循环次数
#define PID_TUNE_CYCLE 5

// PID 自动切换模式，注释就只用eeprom内第一组pid参数
#define PID_AUTO_SHIFT

////////////////////
// Analogue inputs (optional)
// Comment out if not required
#define ANALOGUE1 // if potentiometer connected on ANLG1
#define ANALOGUE2 // if potentiometer connected on ANLG2
#define ANIN1  4// if potentiometer connected on ANLG1 OT1
#define ANIN2  3// if potentiometer connected on ANLG2 OT3


#define BT_FILTER 70
#define ET_FILTER 70
#define AMB_FILTER 70
#define D_MULT 0.001 
#define RISE_FILTER 85 // heavy filtering on non-displayed BT for RoR calculations
#define ROR_FILTER 80 // post-filtering for the computed RoR values

// 以下代码不要动，FreeRTOS用的代码
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
static TaskHandle_t xTASK_LCD = NULL;
static TaskHandle_t xTASK_BLE_CMD_handle = NULL;
static TaskHandle_t xTask_Thermo_get_data = NULL;
static TaskHandle_t xTask_PID_autotune = NULL;



SemaphoreHandle_t xThermoDataMutex = NULL;
SemaphoreHandle_t xLCDDataMutex = NULL;

QueueHandle_t queueCMD_BLE = xQueueCreate(10, sizeof(char[BLE_BUFFER_SIZE]));
QueueHandle_t queue_data_to_BLE = xQueueCreate(10, sizeof(char[BLE_BUFFER_SIZE]));
// 以上代码不要动，FreeRTOS用的代码



const char index_html[] PROGMEM = R"rawliteral(

<!doctype html><html lang='cn'>
 <head>
<title>MATCH BOX H7 SETUP</title>
</head> 
 <body>
<main>
    <h1 align='center'>BLE version:%version%</h1>
       <div align='center'><a href='/update' target='_blank'>FIRMWARE UPDATE</a>
        </br>
        </br>
        <label>PID:CT (current: %pid_CT%)</label>
       </br>
       <label>PID:P (current: %pid_P%)</label>
       </br>
        <label>PID:I (current: %pid_I%)</label>
       </br>
        <label>PID:D (current: %pid_D%)</label>
       </br>
               <label>BT fix: (%bt_fix%)</label>
       </br>
               <label>ET fix: (%et_fix%)</label>
       </br>
        </main>
         </div>
    </body>
 </html>
)rawliteral";

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