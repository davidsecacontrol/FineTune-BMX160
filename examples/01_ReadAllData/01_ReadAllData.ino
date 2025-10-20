#include "FineTuneBMX160.h"

using namespace FineTuneBMX160;
BMX160 bmx160;

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    Wire.begin();

    bmx160.begin();
}

void loop()
{
    DataPacket accel, gyro, magn;
    bmx160.getAllData(accel, gyro, magn);

    Serial.print("Time: ");Serial.print(accel.sensortime * TIMESTAMPS::SENSITIVITY[0]);Serial.println("s");
    
    Serial.print("Accel [ ");Serial.print(accel.x);Serial.print(" , ");Serial.print(accel.y);Serial.print(" , ");Serial.print(accel.z);Serial.println(" ] m^2/s ");
    Serial.print("Gyro  [ ");Serial.print(gyro.x); Serial.print(" , ");Serial.print(gyro.y); Serial.print(" , ");Serial.print(gyro.z); Serial.println(" ] deg/s ");
    Serial.print("Magn  [ ");Serial.print(magn.x); Serial.print(" , ");Serial.print(magn.y); Serial.print(" , ");Serial.print(magn.z); Serial.println(" ] uT ");

    Serial.println("-------------------------------------");

    delay(1000);
}
