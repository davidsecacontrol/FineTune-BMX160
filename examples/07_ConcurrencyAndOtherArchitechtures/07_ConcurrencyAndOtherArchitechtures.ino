#include "FineTuneBMX160.h"

using namespace FineTuneBMX160;

// There are 4 architecture-dependent methods in the library:
// - bool readReg(const REGISTER reg, uint8_t const*const buffer, size_t length)
// - bool writeReg(REGISTER const *const reg, uint8_t *const buffer, size_t length)
// - void wait(unsigned long time)
// - bool isConnected()

// To port this library to other architectures, or allow concurrency simply override these definitions accordingly:
class UserDefinedTimer :public TimingClassTemplate{ 
    public:
        void wait(const uint32_t time) override{
            Serial.print("//Implement a concurrent delay here//\n");
            delay(time);
        };
};


BMX160_Template<UserDefinedTimer> bmx160;

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
