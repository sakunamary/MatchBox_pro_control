/*
   EEPROM Write

   Stores random values into the EEPROM.
   These values will stay in the EEPROM when the board is
   turned off and may be retrieved later by another sketch.
*/
#include <Arduino.h>
#include "config.h"
#include "Wire.h"
#include <MCP3424.h>
#include <ESP32Servo.h>
// #include "TypeK.h"
#include "DFRobot_AHT20.h"
#include "SparkFun_External_EEPROM.h" // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM

// Need this for the lower level access to set them up.
uint8_t address = 0x68;
int i, j;
double temp_cal_tmp[5];
double temp_;
long Voltage; // Array used to store results
#define LOCATION_SETTINGS 0
double BT_TEMP;
double AMB_TEMP;
#define R0 100
#define Rref 1000

const int HEAT_OUT_PIN = PWM_HEAT; // GPIO26
const int FAN_OUT_PIN = PWM_FAN;
const int frequency = PWM_FREQ;
const byte resolution = PWM_RESOLUTION; // pwm -0-1023

MCP3424 ADC_MCP3424(address); // Declaration of MCP3424 A2=0 A1=1 A0=0
DFRobot_AHT20 aht20;
ESP32PWM pwm_heat;
ESP32PWM pwm_fan;

pid_setting_t pid_parm = {
    .pid_CT = 1.5,     // double pid_CT;
    .p = 3.2,          // double p ;
    .i = 0.17,         // double i ;
    .d = 8.0,          // double d ;
    .BT_tempfix = 0.0, // double BT_tempfix;
    .ET_tempfix = 0.0  // double ET_tempfix;
};

ExternalEEPROM I2C_EEPROM;
// TypeK temp_K_cal;

void loadUserSettings();

void setup()
{

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    Serial.begin(BAUDRATE);

    Serial.println("start...\n");
    Serial.println("INIT PWM...\n");
    vTaskDelay(30000);
    //  Init pwm output

    pwm_heat.attachPin(HEAT_OUT_PIN, frequency, resolution); // 1KHz 8 bit
    pwm_fan.attachPin(FAN_OUT_PIN, frequency, resolution);   // 1KHz 8 bit
    pwm_heat.writeScaled(0.0);
    pwm_fan.writeScaled(0.3);

    Serial.println("INIT AHT20 and EEPROM...\n");
    // Prepare working .....
    Wire.begin();
    I2C_EEPROM.setMemoryType(64);

    ADC_MCP3424.NewConversion(); // New conversion is initiated
    aht20.begin();

    Serial.println("Pharse I:Sensor init\n");
    if (aht20.startMeasurementReady(/* crcEn = */ true))
    {
        AMB_TEMP = aht20.getTemperature_C();
    }
    Serial.println("Pharse I:OK\n");
    Serial.println("Pharse II:cal temp fix data in 3s ....\n");
    vTaskDelay(3000);
    Serial.println("Temp raw Reading ....\n");
    vTaskDelay(200);
    ADC_MCP3424.Configuration(1, 16, 1, 1); // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
    Voltage = ADC_MCP3424.Measure();        // Measure is stocked in array Voltage, note that the library will wait for a completed conversion that takes around 200 ms@18bits
    BT_TEMP = pid_parm.BT_tempfix + (((Voltage / 1000 * Rref) / ((3.3 * 1000) - Voltage / 1000) - R0) / (R0 * 0.0039083));

    Serial.printf("Temp raw:: AMB_TEMP:%4.2f;BT:%4.2f\n", AMB_TEMP, BT_TEMP);

    pid_parm.BT_tempfix = AMB_TEMP - BT_TEMP;

    Serial.printf("Temp fix::BT fix:%4.2f\n", pid_parm.BT_tempfix);
    Serial.println("Pharse II:Done\n");

    Serial.println("Pharse III: Write data into EEPROM...\n");

    // part I :init setting
    Serial.println("start EEPROM setting ...");
    if (!I2C_EEPROM.begin())
    {
        Serial.println("failed to initialise EEPROM");
        delay(1000000);
    }
    else
    {
        Serial.println("Initialed EEPROM,load data will be writen after 3s...");
        delay(3000);
        I2C_EEPROM.put(LOCATION_SETTINGS, pid_parm);
        Serial.println("EEPROM,load data for check after 3s...");

        delay(3000);
        loadUserSettings();
        Serial.printf("\nEEPROM value check ...\n");
        Serial.printf("pid_CT:%4.2f\n", pid_parm.pid_CT);
        Serial.printf("PID kp:%4.2f\n", pid_parm.p);
        Serial.printf("PID ki:%4.2f\n", pid_parm.i);
        Serial.printf("PID kd:%4.2f\n", pid_parm.d);
        Serial.printf("BT fix:%4.2f\n", pid_parm.BT_tempfix);
        Serial.printf("ET fix:%4.2f\n", pid_parm.ET_tempfix);
    }
}

void loop()
{
}

// Load the current settings from EEPROM into the settings struct
void loadUserSettings()
{

    // Check to see if EEPROM is blank. If the first four spots are zeros then we can assume the EEPROM is blank.
    uint32_t testRead = 0;
    if (I2C_EEPROM.get(LOCATION_SETTINGS, testRead) == 0) // EEPROM address to read, thing to read into
    {
        // At power on, settings are set to defaults within the struct.
        // So go record the struct as it currently exists so that defaults are set.
        I2C_EEPROM.put(LOCATION_SETTINGS, pid_parm);
        Serial.println("Default settings applied");
    }
    else
    {
        // Read current settings
        I2C_EEPROM.get(LOCATION_SETTINGS, pid_parm);
    }
}
