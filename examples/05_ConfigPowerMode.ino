#include "FineTuneBMX160.h"

using namespace FineTuneBMX160;
BMX160 bmx160;

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    Wire.begin();

    bmx160.begin();

    // Sensor Power Modes - WARNING: Certain functionalities are not supported when one or more sensors aren't in normal mode.
    // Check datasheet for more info
    bmx160.setAccelPowerMode(ACCEL::POWER_MODE::SUSPEND);
    bmx160.setGyroPowerMode(GYRO::POWER_MODE::SUSPEND);
    bmx160.setMagnInterfacePowerMode(MAGN_INTERFACE::POWER_MODE::SUSPEND);

    // Get ------------------------------------------------------------------------------
    ACCEL::POWER_MODE accel_mode;
    bmx160.getAccelPowerMode(accel_mode);
    Serial.print("Current/set accel mode: ");Serial.print((uint)accel_mode);
    Serial.print("/");Serial.println((int)ACCEL::POWER_MODE::SUSPEND);

    GYRO::POWER_MODE gyro_mode;
    bmx160.getGyroPowerMode(gyro_mode);
    Serial.print("Current/set gyro mode: ");Serial.print((uint)gyro_mode);
    Serial.print("/");Serial.println((int)GYRO::POWER_MODE::SUSPEND);

    MAGN_INTERFACE::POWER_MODE magn_mode;
    bmx160.getMagnInterfacePowerMode(magn_mode);
    Serial.print("Current/set magn mode: ");Serial.print((uint)magn_mode);
    Serial.print("/");Serial.println((int)MAGN_INTERFACE::POWER_MODE::SUSPEND);
}

void loop()
{
    while(true);
}
