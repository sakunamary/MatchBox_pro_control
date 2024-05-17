// 发送指令到HMI
#ifndef __TASK_HMI_SERIAL_H__
#define __TASK_HMI_SERIAL_H__

#include <Arduino.h>
#include <config.h>

// For ESP32-C3
HardwareSerial Serial_HMI(0);

extern uint16_t last_FAN;
extern uint16_t last_PWR;

long microseconds;
long prevMicroseconds;
extern int heat_level_to_artisan;
extern int fan_level_to_artisan;

// Arbitrary setpoint and gains - adjust these as fit for your project:

extern double BT_TEMP; // pid_input
extern double PID_output;
extern double pid_sv;
extern bool pid_status;

void TASK_data_to_HMI(void *pvParameters)
{
    (void)pvParameters;
    uint8_t Serial_DATA_Buffer[HMI_BUFFER_SIZE];
    const TickType_t timeOut = 500;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;

    while (1)
    {

        xResult = xTaskNotifyWait(0x00,                 // 在运行前这个命令之前，先清除这几位
                                  0x00,                 // 运行后，重置所有的bits 0x00 or ULONG_MAX or 0xFFFFFFFF
                                  &ulNotificationValue, // 重置前的notification value
                                  portMAX_DELAY);       // 一直等待

        if (xResult == pdTRUE)
        {
            if (xQueueReceive(queue_data_to_HMI, &Serial_DATA_Buffer, timeOut) == pdPASS)
            { // 从接收QueueCMD 接收指令
                Serial_HMI.write(Serial_DATA_Buffer, HMI_BUFFER_SIZE);
                vTaskDelay(20);
            }
        }
    }
}

void TASK_CMD_FROM_HMI(void *pvParameters)
{
    (void)pvParameters;
    uint8_t HMI_ReadBuffer[HMI_BUFFER_SIZE];
    const TickType_t timeOut = 500;
    while (1)
    {
        if (Serial_HMI.available())
        {
            if (xSemaphoreTake(xSerialReadBufferMutex, timeOut) == pdPASS)
            {
                Serial_HMI.readBytes(HMI_ReadBuffer, HMI_BUFFER_SIZE);
                xQueueSend(queueCMD_HMI, &HMI_ReadBuffer, timeOut); // 串口数据发送至队列
                xTaskNotify(xTASK_HMI_CMD_handle, 0, eIncrement);   // 通知处理任务干活
            }
            xSemaphoreGive(xSerialReadBufferMutex);
        }
        vTaskDelay(20);
    }
}

void TASK_HMI_CMD_handle(void *pvParameters)
{
    (void)pvParameters;
    uint8_t HMI_CMD_Buffer[HMI_BUFFER_SIZE];
    const TickType_t timeOut = 1000;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;
    while (1)
    {
        xResult = xTaskNotifyWait(0x00,                 // 在运行前这个命令之前，先清除这几位
                                  0x00,                 // 运行后，重置所有的bits 0x00 or ULONG_MAX or 0xFFFFFFFF
                                  &ulNotificationValue, // 重置前的notification value
                                  portMAX_DELAY);       // 一直等待

        if (xResult == pdTRUE)
        {
            if (xQueueReceive(queueCMD_HMI, &HMI_CMD_Buffer, timeOut) == pdPASS)
            { // 从接收QueueCMD 接收指令

                switch (HMI_CMD_Buffer[2])
                {          // 判断命令的类型
                case 0x01: // 操作控制
                    if (xSemaphoreTake(xSerialReadBufferMutex, timeOut) == pdPASS)
                    {
                        if (pid_status == false && mb.Hreg(PID_STATUS_HREG) == 0)
                        { // pid_status = false  and  pid_status_hreg ==0
                            if (HMI_CMD_Buffer[7] != last_PWR)
                            {
                                last_PWR = HMI_CMD_Buffer[7];
                                mb.Hreg(HEAT_HREG, last_PWR); // last 火力pwr数据更新
                                pwm.write(pwm_heat_out, map(last_PWR, 0, 100, 230, 850), frequency, resolution);
                            }
                            // HMI_CMD_Buffer[5]   //风力数据
                            if (HMI_CMD_Buffer[8] != last_FAN)
                            {
                                last_FAN = HMI_CMD_Buffer[8];
                                mb.Hreg(FAN_HREG, last_FAN); // last 火力pwr数据更新
                                pwm.write(pwm_fan_out, map(last_FAN, 0, 100, 10, 250), frequency, resolution);
                                xSemaphoreGive(xSerialReadBufferMutex);
                            }
                            break;
                        case 0x02: // PID参数设置
                            if (xSemaphoreTake(xSerialReadBufferMutex, timeOut) == pdPASS)
                            {
                                pid_parm.p = double((HMI_CMD_Buffer[3] << 8 | HMI_CMD_Buffer[4]) / 100);
                                pid_parm.i = double((HMI_CMD_Buffer[5] << 8 | HMI_CMD_Buffer[6]) / 100);
                                pid_parm.d = double((HMI_CMD_Buffer[7] << 8 | HMI_CMD_Buffer[8]) / 100);
                                EEPROM.put(0, pid_parm);
                                EEPROM.commit();
                                xSemaphoreGive(xSerialReadBufferMutex);
                            }
                            break;
                        case 0x03: // 其他参数设置
                            pid_parm.pid_CT = HMI_CMD_Buffer[3] << 8 | HMI_CMD_Buffer[4];
                            pid_parm.BT_tempfix = double((HMI_CMD_Buffer[5] << 8 | HMI_CMD_Buffer[6]) / 100);
                            pid_parm.ET_tempfix = double((HMI_CMD_Buffer[7] << 8 | HMI_CMD_Buffer[8]) / 100);
                            EEPROM.put(0, pid_parm);
                            EEPROM.commit();
                            break;
                        case 0x04: // 其他参数设置
                            mb.Hreg(PID_STATUS_HREG, HMI_CMD_Buffer[5]);

                            if (HMI_CMD_Buffer[6] == 1) // pid 自动设定
                            {
                                mb.Hreg(PID_STATUS_HREG, 0); // 关闭 pid
                                vTaskDelay(1000);            // 让pid关闭有足够时间执行
                                while (!tuner.isFinished())  // 开始自动整定
                                {
                                    prevMicroseconds = microseconds;
                                    microseconds = micros();
                                    pid_tune_output = tuner.tunePID(BT_TEMP, microseconds);

                                    pwm.write(pwm_fan_out, map(fan_level_to_artisan, 0, 100, 10, 250), frequency, resolution);
                                    pwm.write(pwm_heat_out, map(pid_tune_output, 0, 100, 230, 850), frequency, resolution);
                                    // This loop must run at the same speed as the PID control loop being tuned
                                    while (micros() - microseconds < pid_parm.pid_CT)
                                        delayMicroseconds(1);
                                }

                                // Turn the output off here.
                                pwm.write(pwm_fan_out, map(fan_level_to_artisan, 0, 100, 10, 250), frequency, resolution);
                                pwm.write(pwm_heat_out, map(pid_tune_output, 0, 100, 230, 850), frequency, resolution);

                                // Get PID gains - set your PID controller's gains to these
                                pid_parm.p = tuner.getKp();
                                pid_parm.i = tuner.getKi();
                                pid_parm.d = tuner.getKd();
                                EEPROM.put(0, pid_parm);
                                EEPROM.commit();
                                // 发送最新的pid参数到HMI，封装数据
                                // make_frame_head(TEMP_DATA_Buffer, 3);
                                // make_frame_data(TEMP_DATA_Buffer, 3, int(pid_parm.p), 3);
                                // make_frame_data(TEMP_DATA_Buffer, 3, int(pid_parm.i), 5);
                                // make_frame_data(TEMP_DATA_Buffer, 3, int(pid_parm.d), 7);
                                // make_frame_end(TEMP_DATA_Buffer, 3);

                                // // 跳出PID自动整定，发送命令到HMI
                                // make_frame_head(TEMP_DATA_Buffer, 4);
                                // TEMP_DATA_Buffer[6] = fan_level_to_artisan;
                                // make_frame_end(TEMP_DATA_Buffer, 4);
                            }
                            break;
                        default:
                            break;
                        }
                    }
                    vTaskDelay(20);
                }
            }
        }
    }
}
#endif

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