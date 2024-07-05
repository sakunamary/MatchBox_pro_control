/*
   EEPROM Write

   Stores random values into the EEPROM.
   These values will stay in the EEPROM when the board is
   turned off and may be retrieved later by another sketch.
*/

#include "config.h"
#include <Wire.h>
#include <MCP3424.h>
#include "DFRobot_AHT20.h"
#include "SparkFun_External_EEPROM.h" // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM

// Need this for the lower level access to set them up.
uint8_t address = 0x68;
int i, j;
double temp_cal_tmp[5];
double temp_;
long Voltage; // Array used to store results

double AMB_TEMP;
double BT_TEMP;
double ET_TEMP;

#define RNOMINAL 100
#define RREF 1000
#define LOCATION_SETTINGS 0

MCP3424 ADC_MCP3424(address); // Declaration of MCP3424 A2=0 A1=1 A0=0
DFRobot_AHT20 aht20;
ExternalEEPROM I2C_EEPROM;

pid_setting_t pid_parm = {
    2,    // uint16_t pid_CT;
    2.0,  // double p ;
    0.12, // double i ;
    5.0,  // double d ;
    0.0,  // uint16_t BT_tempfix;
    0.0   // uint16_t ET_tempfix;
};

void setup()
{
    Serial.begin(115200);
    Serial.println("start...\n");
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
    vTaskDelay(1000);
    ADC_MCP3424.Configuration(1, 16, 1, 1); // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
    for (i = 0; i < 5; i++)
    {
        vTaskDelay(50);
        Voltage = ADC_MCP3424.Measure();
        temp_cal_tmp[i] = pid_parm.BT_tempfix + (((Voltage / 1000 * RNOMINAL) / ((3.3 * 1000) - Voltage / 1000) - RREF) / (RREF * 0.0039083)); // CH3
        for (j = i + 1; j < 5; j++)
        {
            if (temp_cal_tmp[i] > temp_cal_tmp[j])
            {
                temp_ = temp_cal_tmp[i];
                temp_cal_tmp[i] = temp_cal_tmp[j];
                temp_cal_tmp[j] = temp_;
            }
        }
    }
    BT_TEMP = temp_cal_tmp[2]; // for bt temp more accuricy

    vTaskDelay(1000);
    ADC_MCP3424.Configuration(2, 16, 1, 1); // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
    for (i = 0; i < 5; i++)
    {
        vTaskDelay(50);
        Voltage = ADC_MCP3424.Measure();
        temp_cal_tmp[i] = pid_parm.BT_tempfix + (((Voltage / 1000 * RNOMINAL) / ((3.3 * 1000) - Voltage / 1000) - RREF) / (RREF * 0.0039083)); // CH3
        for (j = i + 1; j < 5; j++)
        {
            if (temp_cal_tmp[i] > temp_cal_tmp[j])
            {
                temp_ = temp_cal_tmp[i];
                temp_cal_tmp[i] = temp_cal_tmp[j];
                temp_cal_tmp[j] = temp_;
            }
        }
    }
    ET_TEMP = temp_cal_tmp[2]; // for bt temp more accuricy
    Serial.printf("Temp raw:: AMB_TEMP:%4.2d;BT:%4.2d; ET:%4.2d\n", AMB_TEMP, BT_TEMP, ET_TEMP);

    pid_parm.BT_tempfix = BT_TEMP - AMB_TEMP;
    pid_parm.ET_tempfix = ET_TEMP - AMB_TEMP;
    Serial.printf("Temp fix::BT fix:%4.2d; ET fix:%4.2d\n", pid_parm.BT_tempfix, pid_parm.ET_tempfix);
    Serial.println("Pharse II:OK\n");
    Serial.println("Pharse III: Write data into EEPROM...\n");

    if (!I2C_EEPROM.begin(sizeof(pid_parm)))
    {
        Serial.println("failed to initialise EEPROM");
        Serial.println("Pharse III: NOT OK\n");
    }
    else
    {
        Serial.println("Initialed EEPROM,new data will be writen after 3s...");
        vTaskDelay(3000);
        I2C_EEPROM.put(LOCATION_SETTINGS, pid_parm);
        Serial.println("EEPROM,load data for checking after 3s...");

        // Read current settings
        I2C_EEPROM.get(LOCATION_SETTINGS, pid_parm);

        Serial.printf("\nEEPROM value check ...\n");
        Serial.printf("pid_CT:%d\n", pid_parm.pid_CT);
        Serial.printf("PID kp:%4.2f\n", pid_parm.p);
        Serial.printf("PID ki:%4.2f\n", pid_parm.i);
        Serial.printf("PID kd:%4.2f\n", pid_parm.d);
        Serial.printf("BT fix:%4.2f\n", pid_parm.BT_tempfix);
        Serial.printf("ET fix:%4.2f\n", pid_parm.ET_tempfix);
        Serial.println("Pharse III: OK\n");
    }
}

void loop()
{
}
