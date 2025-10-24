#include "FineTuneBMX160.h"

using namespace FineTuneBMX160;

// All protected methods and variables of this library can be upgraded to public. This way, unimpletented BMX160 functionalities like FIFO buffer or interrupts can still be used.

class public_imu: public BMX160{ 
    public:
        using BMX160::writeReg;
        using BMX160::readReg;
};
public_imu bmx160;

// Alternatively, the base class BMX160_Base can be used. In that case, the interfaces must be manually created and assigned via BMX160_Base::setTimingInterface(...) &  BMX160_Base::setCommunicationInterface(...)
// ---------------------------------------

void setup()
{
    Serial.begin(115200);
    while (!Serial);


    bmx160.begin();

    // Manually setting ACCEL::RANGE to G2 and checking
    bmx160.writeReg(REGISTER::ACC_RANGE,3);
    uint8_t accel_range;
    bmx160.readReg(REGISTER::ACC_RANGE,accel_range);
    Serial.print("Manually setting accel range (current/expected): ");Serial.print(accel_range);
    Serial.print("/");Serial.println((int)ACCEL::RANGE::G2);
}

void loop()
{
    while(true);
}
