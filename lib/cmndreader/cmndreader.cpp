// cmndreader.cpp
//----------------

// code that defines specific commands for aArtisan

// *** BSD License ***
// ------------------------------------------------------------------------------------------
// Copyright (c) 2011, MLG Properties, LLC
// All rights reserved.
//
// Contributor:  Jim Gallt
//
// Redistribution and use in source and binary forms, with or without modification, are
// permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this list of
//   conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this list
//   of conditions and the following disclaimer in the documentation and/or other materials
//   provided with the distribution.
//
//   Neither the name of the copyright holder(s) nor the names of its contributors may be
//   used to endorse or promote products derived from this software without specific prior
//   written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ------------------------------------------------------------------------------------------

// Version 1.10

#include "cmndreader.h"

// define command objects (all are derived from CmndBase)

pidCmnd pid;
io3Cmnd io3;
ot1Cmnd ot1;

// const uint16_t HEAT_HREG = 3003;
// const uint16_t FAN_HREG = 3004;
// const uint16_t SV_HREG = 3005;
// const uint16_t RESET_HREG = 3006;
// const uint16_t PID_HREG = 3007;
// const uint16_t PID_P_HREG = 3008;
// const uint16_t PID_I_HREG = 3009;
// const uint16_t PID_D_HREG = 3010;

// ----------------------------- pidCmnd
// constructor
pidCmnd::pidCmnd() : CmndBase(PID_CMD)
{
}

// execute the PID command
// PID;ON\n ;OFF\n ;TIME\n ;P;x\n ;T(or T_POM);ppp;iii;ddd\n

boolean pidCmnd::doCommand(CmndParser *pars)
{
    // Serial.println((char *)pars);

    if (strcmp(keyword, pars->cmndName()) == 0)
    {
        if (strcmp(pars->paramStr(1), "ON") == 0)
        {
            // mb.Hreg(PID_HREG, 1); // Hreg 设置为1
            pid_status = true;
            Heat_pid_controller.start();
            return true;
        }
        else if (strcmp(pars->paramStr(1), "OFF") == 0)
        {
            // mb.Hreg(PID_HREG, 0); // Hreg 设置为1
            Heat_pid_controller.stop();
            pid_status = false;
            pid_sv = 0;
            return true;
        }
        // else if (strcmp(pars->paramStr(1), "OUT") == 0)
        // {
        //     // uint16_t CHAN_TEMP = atoi(pars->paramStr(2));
        //     // uint16_t PID_OUT_PWR = atoi(pars->paramStr(3));
        //     // mb.Hreg(HEAT_HREG, PID_OUT_PWR);

        //     return true;
        // }
        else if (strcmp(pars->paramStr(1), "SV") == 0)
        {
            // 持续发送sv数据，TC4输出：#DATA_OUT，PID，SV，val
            uint8_t PID_SV_FROM_BLE = atoi(pars->paramStr(2));
            if (pid_status == true)
            {
                pid_sv = PID_SV_FROM_BLE*100/100;
                if (pid_status == true)
                {
                    Heat_pid_controller.compute();
                    levelOT1=map(PID_output-2, 0, 255, 0, 100);
                    pwm.write(pwm_heat_out, map(levelOT1, 1, 100, 230, 950), frequency, resolution);
                }

                // sprintf(BLE_data_buffer_char, "#DATA_OUT,PID,SV,%4.2f\n", pid_sv);
                // // Serial.print(BLE_data_buffer_char);
                // // 格式转换
                // memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));

                // if (deviceConnected)
                // {
                //     pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                //     pTxCharacteristic->notify();
                // }
            }

            // mb.Hreg(SV_HREG, PID_SV * 10);

            return true;
        }
        /*
        else if (strcmp(pars->paramStr(1), "TIME") == 0)
        {
            return true;
        }
        else if (strcmp(pars->paramStr(1), "GO") == 0)
        {

            return true;
        }
        else if (strcmp(pars->paramStr(1), "STOP") == 0)
        {

            return true;
        }
        else if (strcmp(pars->paramStr(1), "P") == 0)
        { // Select profile

            return true;
        }
        else if (pars->paramStr(1)[0] == 'T')
        { // Tune PID

            return true;
        }

        else if (strcmp(pars->paramStr(1), "CT") == 0)
        {

            return true;
        }
        else if (strcmp(pars->paramStr(1), "CHAN") == 0)
        {

            pid_chan = atoi(pars->paramStr(2));


            return true;
        }
        else if (strcmp(pars->paramStr(1), "LIMIT") == 0)
        {
            // uint8_t lower_limit = atoi(pars->paramStr(2));
            // uint8_t upper_limit = atoi(pars->paramStr(3));
            // myPID.SetOutputLimits(lower_limit, upper_limit); // set output limits to user defined limits

            return true;
        }*/

        else
        {
            return false;
        }
    }
}

// ----------------------------- io3Cmnd
// constructor
io3Cmnd::io3Cmnd() : CmndBase(IO3_CMD)
{
}

// execute the IO3 command
// IO3;ddd\n

boolean io3Cmnd::doCommand(CmndParser *pars)
{

    if (strcmp(keyword, pars->cmndName()) == 0)
    {
        if (strcmp(pars->paramStr(1), "UP") == 0)
        {
            levelIO3 = levelIO3 + DUTY_STEP;
            if (levelIO3 > MAX_IO3)
                levelIO3 = MAX_IO3; // don't allow OT1 to exceed maximum
            if (levelIO3 < MIN_IO3)
                levelIO3 = MIN_IO3; // don't allow OT1 to turn on less than minimum
            pwm.write(pwm_fan_out, map(levelIO3, MIN_IO3, MAX_IO3, 750, 1000), frequency, resolution);
            // Serial.printf("FAN:%d\n", levelIO3);
            sprintf(BLE_data_buffer_char, "#DATA_OUT,OT3,%d\n", levelIO3);
            // Serial.print(BLE_data_buffer_char);
            // 格式转换
            memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));

            if (deviceConnected)
            {
                pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                pTxCharacteristic->notify();
            }
            return true;
        }
        else if (strcmp(pars->paramStr(1), "DOWN") == 0)
        {
            levelIO3 = levelIO3 - DUTY_STEP;
            if (levelIO3 < MIN_IO3 & levelIO3 != 0)
                levelIO3 = 0; // turn ot1 off if trying to go below minimum. or use levelOT1 = MIN_HTR ?
            pwm.write(pwm_fan_out, map(levelIO3, MIN_IO3, MAX_IO3, 750, 1000), frequency, resolution);
            sprintf(BLE_data_buffer_char, "#DATA_OUT,OT3,%d\n", levelIO3);
            // Serial.print(BLE_data_buffer_char);
            memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
            // 格式转换
            // for (int i = 0; i <64; i++)
            // {
            //     BLE_data_buffer_uint8[i] = BLE_data_buffer_char[i];
            // }
            if (deviceConnected)
            {
                pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                pTxCharacteristic->notify();
            }

            return true;
        }
        else
        {
            uint8_t len = strlen(pars->paramStr(1));
            if (len > 0)
            {
                levelIO3 = atoi(pars->paramStr(1));
                if (levelIO3 > MAX_IO3)
                    levelIO3 = MAX_IO3; // don't allow OT1 to exceed maximum
                if (levelIO3 < MIN_IO3 & levelIO3 != 0)
                    levelIO3 = MIN_IO3; // don't allow to set less than minimum unless setting to zero
                pwm.write(pwm_fan_out, map(levelIO3, MIN_IO3, MAX_IO3, 750, 1000), frequency, resolution);
                sprintf(BLE_data_buffer_char, "#DATA_OUT,OT3,%d\n", levelIO3);
                // Serial.print(BLE_data_buffer_char);
                memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
                // 格式转换
                if (deviceConnected)
                {
                    pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                    pTxCharacteristic->notify();
                }
            }
            return true;
        }
    }
    else
    {
        return false;
    }
}

// ----------------------------- io3Cmnd
// constructor
ot1Cmnd::ot1Cmnd() : CmndBase(OT1_CMD)
{
}

// execute the OT1 command
// IO1;ddd\n

boolean ot1Cmnd::doCommand(CmndParser *pars)
{

    if (strcmp(keyword, pars->cmndName()) == 0)
    {
        if (strcmp(pars->paramStr(1), "UP") == 0)
        {
            levelOT1 = levelOT1 + DUTY_STEP;
            if (levelOT1 > MAX_OT1)
                levelOT1 = MAX_OT1; // don't allow OT1 to exceed maximum
            if (levelOT1 < MIN_OT1)
                levelOT1 = MIN_OT1; // don't allow OT1 to turn on less than minimum
            pwm.write(pwm_heat_out, map(levelOT1, 1, 100, 230, 950), frequency, resolution);
            // sprintf(BLE_data_buffer_char, "#DATA_OUT,OT1,%d\n", levelOT1);
            // // Serial.print(BLE_data_buffer_char);
            // memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
            // // 格式转换
            // if (deviceConnected)
            // {
            //     pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
            //     pTxCharacteristic->notify();
            // }
            return true;
        }
        else if (strcmp(pars->paramStr(1), "DOWN") == 0)
        {
            levelOT1 = levelOT1 - DUTY_STEP;
            if (levelOT1 < MIN_OT1 & levelOT1 != 0)
                levelOT1 = 0; // turn ot1 off if trying to go below minimum. or use levelOT1 = MIN_HTR ?
            pwm.write(pwm_heat_out, map(levelOT1, 1, 100, 230, 950), frequency, resolution);
            // sprintf(BLE_data_buffer_char, "#DATA_OUT,OT1,%d\n", levelOT1);
            // // Serial.print(BLE_data_buffer_char);
            // memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
            // // 格式转换
            // if (deviceConnected)
            // {
            //     pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
            //     pTxCharacteristic->notify();
            // }
            return true;
        }
        else
        {
            uint8_t len = strlen(pars->paramStr(1));
            if (len > 0)
            {
                levelOT1 = atoi(pars->paramStr(1));
                if (levelOT1 > MAX_OT1)
                    levelOT1 = MAX_OT1; // don't allow OT1 to exceed maximum
                if (levelOT1 < MIN_OT1 & levelOT1 != 0)
                    levelOT1 = MIN_OT1; // don't allow to set less than minimum unless setting to zero
                pwm.write(pwm_heat_out, map(levelOT1, 1, 100, 230, 950), frequency, resolution);
                // sprintf(BLE_data_buffer_char, "#DATA_OUT,OT1,%d\n", levelOT1);
                // // Serial.print(BLE_data_buffer_char);
                // memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
                // // 格式转换
                // if (deviceConnected)
                // {
                //     pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                //     pTxCharacteristic->notify();
                // }
            }
            return true;
        }
    }
    else
    {
        return false;
    }
}