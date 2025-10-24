#include "FineTuneBMX160.h"

using namespace FineTuneBMX160;
BMX160 bmx160;

void setup()
{
    Serial.begin(115200);
    while (!Serial);


    bmx160.begin();

    // Set ------------------------------------------------------------------
    bmx160.setAccelRange(ACCEL::RANGE::G2);
    bmx160.setGyroRange(GYRO::RANGE::DPS250);
    // Magn range not configurable

    // Get -------------------------------------------------------------------
    ACCEL::RANGE accel_range;
    bmx160.getAccelRange(accel_range);
    Serial.print("Current/set accel range: ");Serial.print((uint)accel_range);
    Serial.print("/");Serial.println((int)ACCEL::RANGE::G2);

    GYRO::RANGE gyro_range;
    bmx160.getGyroRange(gyro_range);
    Serial.print("Current/set gyro range: ");Serial.print((uint)gyro_range);
    Serial.print("/");Serial.println((int)GYRO::RANGE::DPS250);

}

void loop()
{
    while(true);
}
