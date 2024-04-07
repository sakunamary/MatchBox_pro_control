/*
   EEPROM Write

   Stores random values into the EEPROM.
   These values will stay in the EEPROM when the board is
   turned off and may be retrieved later by another sketch.
*/

#include "EEPROM.h"

typedef struct eeprom_settings
{
    uint16_t pid_CT;
    double p;
    double i;
    double d;
    uint16_t BT_tempfix;
    uint16_t ET_tempfix;
} pid_setting_t;

extern pid_setting_t;

extern pid_setting_t pid_parm;

void setup()
{
    Serial.begin(BAUDRATE);
    Serial.println("start...");
    if (!EEPROM.begin(sizeof(pid_parm)))
    {
        Serial.println("failed to initialise EEPROM");
        delay(1000000);
    }
    else
    {
        Serial.println("Initialed EEPROM,data will be writen after 3s...");
        delay(3000);
        EEPROM.get(0, pid_parm);

        pid_parm.pid_CT = 1500;
        pid_parm.p = 2.0;
        pid_parm.i = 0.12;
        pid_parm.d = 5;
        pid_parm.BT_tempfix = 0.0;
        pid_parm.ET_tempfix = -3.0;

        EEPROM.put(0, pid_parm);
        EEPROM.commit();
    }

    Serial.println(" bytes read from Flash . Values are:");

    for (int i = 0; i < sizeof(pid_parm); i++)
    {
        Serial.print(byte(EEPROM.read(i)));
        Serial.print(" ");
    }
}

void loop()
{
}
