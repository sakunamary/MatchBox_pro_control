// cmndreader.h
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

#ifndef CMNDREADER_H
#define CMNDREADER_H

#include "cmndproc.h"
#include <pwmWrite.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


// ----------------------- commands
#define PID_CMD "PID" // turn PID ON or OFF
#define IO3_CMD "IO3" // 0 to 100 percent PWM 5V output on IO3
#define OT1_CMD "OT1" // 0 to 100 percent PWM 5V output on IO3

// -------------------------- slew rate limitations for fan control
#define MAX_SLEW 25                                           // percent per second
#define SLEW_STEP 3                                           // increase in steps of 5% for smooth transition
#define SLEW_STEP_TIME (uint32_t)(SLEW_STEP * 500 / MAX_SLEW) // min ms delay between steps
#define DUTY_STEP 1                                           // Use 1, 2, 4, 5, or 10.

////////////////////
// Heater and Fan Limits/Options
#define MIN_OT1 0   // Set output % for lower limit for OT1.  0% power will always be available
#define MAX_OT1 100 // Set output % for upper limit for OT1

#define MIN_IO3 30  // Set output % for lower limit for IO3.  0% power will always be available
#define MAX_IO3 100 // Set output % for upper limit for IO3

// pwm setting
#define PWM_FAN 5
#define PWM_HEAT 2
#define PWM_FREQ 3922
#define PWM_RESOLUTION 10 // 0-1024

extern int levelOT1;
extern int levelIO3;
extern bool pid_status;

extern double PID_output;
extern double pid_sv;
extern double pid_tune_output;

extern const uint32_t frequency;
extern const byte resolution;
extern const byte pwm_fan_out;
extern const byte pwm_heat_out;

extern uint8_t BLE_data_buffer_uint8[64];
extern char BLE_data_buffer_char[64];
const TickType_t xIntervel = 150 / portTICK_PERIOD_MS;

extern bool deviceConnected;
extern SemaphoreHandle_t xContrlDataMutex;

// forward declarations
class pidCmnd;
class io3Cmnd;
class ot1Cmnd;

// external declarations of class objects
extern Pwm pwm;
// extern ModbusIP mb;
extern pidCmnd pid;
extern io3Cmnd io3;
extern ot1Cmnd ot1;
extern BLECharacteristic *pTxCharacteristic;
// class declarations for commands

class pidCmnd : public CmndBase
{
public:
    pidCmnd();
    virtual boolean doCommand(CmndParser *pars);
};

class io3Cmnd : public CmndBase
{
public:
    io3Cmnd();
    virtual boolean doCommand(CmndParser *pars);
};

class ot1Cmnd : public CmndBase
{
public:
    ot1Cmnd();
    virtual boolean doCommand(CmndParser *pars);
};
#endif
