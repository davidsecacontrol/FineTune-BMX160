#include "FineTuneBMX160.h"

FTBMX160::FTBMX160(arduino::TwoWire& Wire, uint8_t address) : Wire(Wire), address{address} {};

uint8_t FTBMX160::writeReg(const uint8_t reg, const uint8_t byte)
{
    this->Wire.beginTransmission(this->address);
    this->Wire.write(reg);
    this->Wire.write(&byte, UINT8_C(1));
    return this->Wire.endTransmission();
}

uint8_t FTBMX160::readReg(const uint8_t reg)
{
    // Send register to read
    this->Wire.beginTransmission(this->address);
    this->Wire.write(&reg, UINT8_C(1));
    this->Wire.endTransmission(false); // Error to be addressed, "false" to not release bus

    // Read the result
    this->Wire.requestFrom(this->address, UINT8_C(1));
    uint8_t byte = this->Wire.read();
    this->Wire.endTransmission();
    return byte;
}

// Add error codes https://docs.arduino.cc/language-reference/en/functions/communication/this->Wire/endTransmission/
uint8_t FTBMX160::isConnected()
{
    this->Wire.beginTransmission(this->address);
    return this->Wire.endTransmission();
}