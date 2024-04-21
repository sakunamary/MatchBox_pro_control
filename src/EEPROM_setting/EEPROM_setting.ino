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
    double BT_tempfix;
    double ET_tempfix;
} pid_setting_t;

pid_setting_t pid_parm;

void setup()
{
    Serial.begin(115200);
    Serial.println("start...");
    if (!EEPROM.begin(sizeof(pid_parm)))
    {
        Serial.println("failed to initialise EEPROM");
        delay(5000);
    }
    else
    {
        Serial.println("Initialed EEPROM,data will be writen after 3s...");
        delay(3000);
        EEPROM.get(0, pid_parm);

        pid_parm.pid_CT = 2000;
        pid_parm.p = 2.0;
        pid_parm.i = 0.12;
        pid_parm.d = 5.0;
        pid_parm.BT_tempfix = 0.0;
        pid_parm.ET_tempfix = 0.0;

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
