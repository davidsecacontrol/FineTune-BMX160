#include "FineTuneBMX160.h"

using namespace FineTune;

BMX160::BMX160(arduino::TwoWire& Wire, uint8_t address) : Wire(Wire), address{address} {};

I2C_STATUS BMX160::writeReg(const uint8_t reg, const uint8_t byte)
{
    this->Wire.beginTransmission(this->address);
    this->Wire.write(reg);
    this->Wire.write(&byte, UINT8_C(1));
    return static_cast<I2C_STATUS> (this->Wire.endTransmission());
}

I2C_STATUS BMX160::readReg(const uint8_t reg, uint8_t& buffer)
{
    // Send register to read
    this->Wire.beginTransmission(this->address);
    this->Wire.write(&reg, UINT8_C(1));
    I2C_STATUS status = static_cast<I2C_STATUS>(this->Wire.endTransmission(false)); // Error to be addressed, "false" to not release bus

    if(status != I2C_STATUS::SUCCESS){
        return status;
    }

    // Read the result
    this->Wire.requestFrom(this->address, UINT8_C(1));
    buffer = this->Wire.read();
    return static_cast<I2C_STATUS> (this->Wire.endTransmission());
}

// Add error codes https://docs.arduino.cc/language-reference/en/functions/communication/this->Wire/endTransmission/
I2C_STATUS BMX160::isConnected()
{
    this->Wire.beginTransmission(this->address);
    return static_cast<I2C_STATUS> (this->Wire.endTransmission());
}