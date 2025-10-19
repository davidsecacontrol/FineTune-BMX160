#include "FineTuneBMX160.h"

using namespace FineTuneBMX160;
BMX160 bmx160;

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    Wire.begin();

    bmx160.begin();

    // Set sensor ODRs - WARNING: Certain ODRs are only allowed under specific circumstances. -----------------------------------
    //Check datasheet for more info.
    bmx160.setAccelOdr(ACCEL::ODR::Hz100);
    bmx160.setGyroOdr(GYRO::ODR::Hz100);
    bmx160.setMagnInterfaceOdr(MAGN_INTERFACE::ODR::Hz100);


    // Get ------------------------------------------------------------------------------------------------
    ACCEL::ODR accel_odr;
    bmx160.getAccelOdr(accel_odr);
    Serial.print("Current/set accel ODR: ");Serial.print((uint)accel_odr);
    Serial.print("/");Serial.println((int)ACCEL::ODR::Hz100);

    GYRO::ODR gyro_odr;
    bmx160.getGyroOdr(gyro_odr);
    Serial.print("Current/set gyro ODR: ");Serial.print((uint)gyro_odr);
    Serial.print("/");Serial.println((int)GYRO::ODR::Hz100);

    MAGN_INTERFACE::ODR magn_odr;
    bmx160.getMagnInterfaceOdr(magn_odr);
    Serial.print("Current/set magn ODR: ");Serial.print((uint)magn_odr);
    Serial.print("/");Serial.println((int)MAGN_INTERFACE::ODR::Hz100);

}

void loop()
{
    while(true);
}
