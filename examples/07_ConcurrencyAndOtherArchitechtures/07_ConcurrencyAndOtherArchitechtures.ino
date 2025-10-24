#include "FineTuneBMX160.h"

using namespace FineTuneBMX160;

// There are exactly 2 interfaces the library depends on, and both are public in "interfaces.h":
// - TimingInterface: Controls how the library handles delays. Default implementation uses ArduinoBlockingDelay which uses delay(time);
// - CommunicationInterface: Controls how the library communicates with the sensor. Default implementation uses ArduinoI2CCommunication which uses Arduino's I2C library, "Wire.h"

// To change any of these interfaces, inherit from the interface and implement your own version:

class TimingImpl_concurrency :public TimingInterface{ 
    protected:

    void wait(unsigned long time) override{
        Serial.print("//Implement a concurrent delay here//\n");
        delay(time);
    };
};
// Now, there are 2 possibilities: Select implementation by commenting in/out (manual/automated) next line
#define MANUAL_MODE

#ifdef MANUAL_MODE
// 1 -> Use BMX160_Base and manually instantiate and set each interface  -> Avg: Can use constructor parameters for interfaces
TimingImpl_concurrency concurrent_timer;
ArduinoI2CCommunication manual_comms(I2C_ADDRESS);
BMX160_Base bmx160;
#else
// 2-> Use BMX160_Template that already templated this -> Cons: User-defined parameters for interfaces must be templated
BMX160_Template<TimingImpl_concurrency,ArduinoI2CCommunication_Template<I2C_ADDRESS>> bmx160;
#endif

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    #ifdef MANUAL_MODE
        bmx160.setTimingInterface(concurrent_timer);
        bmx160.setCommunicationInterface(manual_comms);
    #endif

    
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
