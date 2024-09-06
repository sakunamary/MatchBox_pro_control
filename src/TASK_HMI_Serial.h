// 发送指令到HMI
#ifndef __TASK_HMI_SERIAL_H__
#define __TASK_HMI_SERIAL_H__

#include <Arduino.h>
#include <config.h>

// For ESP32-C3
HardwareSerial Serial_HMI(0);
extern int heat_level_to_artisan;
extern int fan_level_to_artisan;

// Arbitrary setpoint and gains - adjust these as fit for your project:

extern double BT_TEMP; // pid_input
extern double ET_TEMP;
extern double AMB_RH;
extern double AMB_TEMP;
extern uint32_t AMB_PRESS;
extern double PID_output;
extern double pid_sv;
extern bool pid_status;

void TASK_data_to_HMI(void *pvParameters)
{
    (void)pvParameters;
    uint8_t Serial_DATA_Buffer[HMI_BUFFER_SIZE];
    const TickType_t timeOut = 150 / portTICK_PERIOD_MS;
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
                memset(Serial_DATA_Buffer, '\0', HMI_BUFFER_SIZE);
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
        }
    }
}

void TASK_CMD_FROM_HMI(void *pvParameters)
{
    (void)pvParameters;
    uint8_t HMI_ReadBuffer[HMI_BUFFER_SIZE];
    const TickType_t timeOut = 250 / portTICK_PERIOD_MS;

    while (1)
    {
        if (Serial_HMI.available())
        {

            Serial_HMI.readBytes(HMI_ReadBuffer, HMI_BUFFER_SIZE);
            xQueueSend(queueCMD_HMI, &HMI_ReadBuffer, timeOut); // 串口数据发送至队列
            xTaskNotify(xTASK_HMI_CMD_handle, 0, eIncrement);   // 通知处理任务干活
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void TASK_HMI_CMD_handle(void *pvParameters)
{
    (void)pvParameters;
    uint8_t HMI_CMD_Buffer[HMI_BUFFER_SIZE];

    char temp_data_buffer_ble[BLE_BUFFER_SIZE];
    uint8_t temp_data_buffer_hmi[HMI_BUFFER_SIZE];
    const TickType_t timeOut = 500 / portTICK_PERIOD_MS;
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
            {
                // Serial.write(HMI_CMD_Buffer, HMI_BUFFER_SIZE);
                // 67 FF 01 00 00 00 00 00 00 00 00 25 37 00 FF FF FF

                if (HMI_CMD_Buffer[0] == 0x67 && HMI_CMD_Buffer[1] == 0xFF && HMI_CMD_Buffer[16] == 0xFF && HMI_CMD_Buffer[15] == 0xFF && HMI_CMD_Buffer[14] == 0xFF)
                {
                    switch (HMI_CMD_Buffer[2])
                    {          // 判断命令的类型
                    case 0x01: // 操作控制
                        if (xSemaphoreTake(xDATA_OUT_Mutex, timeOut) == pdPASS)
                        {
                            if (HMI_CMD_Buffer[13] != pid_status) // 获取pid run off
                            {
                                pid_status = HMI_CMD_Buffer[13];
                            }
                            //////////////// 判断是否在 pid run 模式下////////////////////
                            if (pid_status == false)                // 手动模式下
                            {                                       // pid_status = false  and  pid_status_hreg ==0
                                if (HMI_CMD_Buffer[11] != levelOT1) // HMI输入新的火力值
                                {
                                    levelOT1 = HMI_CMD_Buffer[11];
                                    pwm_heat.write(map(levelOT1, 0, 100, PWM_HEAT_MIN, PWM_HEAT_MAX));
                                }
                                // HMI_CMD_Buffer[5]   //风力数据
                                if (HMI_CMD_Buffer[12] != levelIO3) // HMI输入新的风力值
                                {
                                    levelIO3 = HMI_CMD_Buffer[12];
                                    pwm_fan.write(map(levelIO3, 0, 100, 600, 1000));
                                }
                            }
                            else // pid run on
                            {
                                if (pid_sv != double((HMI_CMD_Buffer[9] << 8 | HMI_CMD_Buffer[10]) / 100)) // 获取 pid sv值
                                {
                                    pid_sv = double((HMI_CMD_Buffer[9] << 8 | HMI_CMD_Buffer[10]) / 100);
                                }
                            }
                            /////////////////更新HMI数据 增加速度///////////////////

                            // PID ON:ambient,chan1,chan2,  heater duty, fan duty, SV

                            // 封装BLE 协议
                            sprintf(temp_data_buffer_ble, "#%4.2f,%4.2f,%4.2f,%d,%d,%4.2f;\n", AMB_TEMP, ET_TEMP, BT_TEMP, levelOT1, levelIO3, pid_sv);
                            // Serial.print(temp_data_buffer_ble);
                            xQueueSend(queue_data_to_BLE, &temp_data_buffer_ble, timeOut);

                            // 封装HMI 协议
                            temp_data_buffer_hmi[0] = 0x69;
                            temp_data_buffer_hmi[1] = 0xff;
                            temp_data_buffer_hmi[2] = 0x01;
                            temp_data_buffer_hmi[3] = lowByte(int(round(AMB_TEMP * 10)));
                            temp_data_buffer_hmi[4] = highByte(int(round(AMB_TEMP * 10)));
                            temp_data_buffer_hmi[5] = lowByte(int(round(ET_TEMP * 10)));
                            temp_data_buffer_hmi[6] = highByte(int(round(ET_TEMP * 10)));
                            temp_data_buffer_hmi[7] = lowByte(int(round(BT_TEMP * 10)));
                            temp_data_buffer_hmi[8] = highByte(int(round(BT_TEMP * 10)));
                            temp_data_buffer_hmi[9] = lowByte(int(round(pid_sv * 10)));
                            temp_data_buffer_hmi[10] = highByte(int(round(pid_sv * 10)));
                            temp_data_buffer_hmi[11] = levelOT1;
                            temp_data_buffer_hmi[12] = levelIO3;
                            if (pid_status)
                            {
                                temp_data_buffer_hmi[13] = 0x01;
                            }
                            else
                            {
                                temp_data_buffer_hmi[13] = 0x00;
                            }
                            temp_data_buffer_hmi[14] = 0xff;
                            temp_data_buffer_hmi[15] = 0xff;
                            temp_data_buffer_hmi[16] = 0xff;

                            xQueueSend(queue_data_to_HMI, &temp_data_buffer_hmi, timeOut);
                            memset(temp_data_buffer_hmi, '\0', HMI_BUFFER_SIZE);
                            xSemaphoreGive(xDATA_OUT_Mutex); // mutex unlock
                        }
                        xTaskNotify(xTASK_data_to_BLE, 0, eIncrement); // send notify to TASK_data_to_HMI
                        xTaskNotify(xTASK_data_to_HMI, 0, eIncrement); // send notify to TASK_data_to_HMIÍ

                        break;
                    case 0x02: // PID参数设置
                        if (xSemaphoreTake(xDATA_OUT_Mutex, timeOut) == pdPASS)
                        {
                            xSemaphoreGive(xDATA_OUT_Mutex);
                        }
                        break;
                    default:
                        break;
                    }
                }

                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
        }
    }
}

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

// 从接收QueueCMD 接收指令

// switch (HMI_CMD_Buffer[2])
// {          // 判断命令的类型
// case 0x01: // 操作控制
//     if (xSemaphoreTake(xSerialReadBufferMutex, timeOut) == pdPASS)
//     {
//         if (pid_status == false && mb.Hreg(PID_STATUS_HREG) == 0)
//         { // pid_status = false  and  pid_status_hreg ==0
//             if (HMI_CMD_Buffer[7] != last_PWR)
//             {
//                 last_PWR = HMI_CMD_Buffer[7];
//                 mb.Hreg(HEAT_HREG, last_PWR); // last 火力pwr数据更新
//                 pwm.write(pwm_heat_out, map(last_PWR, 0, 100, 230, 850), frequency, resolution);
//             }
//             // HMI_CMD_Buffer[5]   //风力数据
//             if (HMI_CMD_Buffer[8] != last_FAN)
//             {
//                 last_FAN = HMI_CMD_Buffer[8];
//                 mb.Hreg(FAN_HREG, last_FAN); // last 火力pwr数据更新
//                 pwm.write(pwm_fan_out, map(last_FAN, 0, 100, 10, 250), frequency, resolution);
//                 xSemaphoreGive(xSerialReadBufferMutex);
//             }
//             break;
//         case 0x02: // PID参数设置
//             if (xSemaphoreTake(xSerialReadBufferMutex, timeOut) == pdPASS)
//             {
//                 pid_parm.p = double((HMI_CMD_Buffer[3] << 8 | HMI_CMD_Buffer[4]) / 100);
//                 pid_parm.i = double((HMI_CMD_Buffer[5] << 8 | HMI_CMD_Buffer[6]) / 100);
//                 pid_parm.d = double((HMI_CMD_Buffer[7] << 8 | HMI_CMD_Buffer[8]) / 100);
//                 EEPROM.put(0, pid_parm);
//                 EEPROM.commit();
//                 xSemaphoreGive(xSerialReadBufferMutex);
//             }
//             break;
//         case 0x03: // 其他参数设置
//             pid_parm.pid_CT = HMI_CMD_Buffer[3] << 8 | HMI_CMD_Buffer[4];
//             pid_parm.BT_tempfix = double((HMI_CMD_Buffer[5] << 8 | HMI_CMD_Buffer[6]) / 100);
//             pid_parm.ET_tempfix = double((HMI_CMD_Buffer[7] << 8 | HMI_CMD_Buffer[8]) / 100);
//             EEPROM.put(0, pid_parm);
//             EEPROM.commit();
//             break;
//         case 0x04: // 其他参数设置
//             mb.Hreg(PID_STATUS_HREG, HMI_CMD_Buffer[5]);

//             if (HMI_CMD_Buffer[6] == 1) // pid 自动设定
//             {
//                 mb.Hreg(PID_STATUS_HREG, 0); // 关闭 pid
//                 vTaskDelay(1000);            // 让pid关闭有足够时间执行
//                 while (!tuner.isFinished())  // 开始自动整定
//                 {
//                     prevMicroseconds = microseconds;
//                     microseconds = micros();
//                     pid_tune_output = tuner.tunePID(BT_TEMP, microseconds);

//                     pwm.write(pwm_fan_out, map(fan_level_to_artisan, 0, 100, 10, 250), frequency, resolution);
//                     pwm.write(pwm_heat_out, map(pid_tune_output, 0, 100, 230, 850), frequency, resolution);
//                     // This loop must run at the same speed as the PID control loop being tuned
//                     while (micros() - microseconds < pid_parm.pid_CT)
//                         delayMicroseconds(1);
//                 }

//                 // Turn the output off here.
//                 pwm.write(pwm_fan_out, map(fan_level_to_artisan, 0, 100, 10, 250), frequency, resolution);
//                 pwm.write(pwm_heat_out, map(pid_tune_output, 0, 100, 230, 850), frequency, resolution);

//                 // Get PID gains - set your PID controller's gains to these
//                 pid_parm.p = tuner.getKp();
//                 pid_parm.i = tuner.getKi();
//                 pid_parm.d = tuner.getKd();
//                 EEPROM.put(0, pid_parm);
//                 EEPROM.commit();
//                 // 发送最新的pid参数到HMI，封装数据
//                 // make_frame_head(TEMP_DATA_Buffer, 3);
//                 // make_frame_data(TEMP_DATA_Buffer, 3, int(pid_parm.p), 3);
//                 // make_frame_data(TEMP_DATA_Buffer, 3, int(pid_parm.i), 5);
//                 // make_frame_data(TEMP_DATA_Buffer, 3, int(pid_parm.d), 7);
//                 // make_frame_end(TEMP_DATA_Buffer, 3);

//                 // // 跳出PID自动整定，发送命令到HMI
//                 // make_frame_head(TEMP_DATA_Buffer, 4);
//                 // TEMP_DATA_Buffer[6] = fan_level_to_artisan;
//                 // make_frame_end(TEMP_DATA_Buffer, 4);
//             }
//             break;
//         default:
//             break;