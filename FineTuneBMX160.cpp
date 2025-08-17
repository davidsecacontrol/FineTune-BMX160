#include "FineTuneBMX160.h"

using namespace FineTuneBMX160;

// Utility functions:----------------
inline void CopyBufferToDataPacket(BMX160DataPacket& packet, uint8_t* buffer, float conversionFactor);

// ---------------------------------


BMX160::BMX160(arduino::TwoWire &Wire, uint8_t address) : Wire(Wire), address{address} {};

 I2C_STATUS BMX160::begin(){
    I2C_STATUS state = writeReg(UINT8_C(0x7E),UINT8_C(0x11)); // Turn on accel
    delay(10);

    if(state != I2C_STATUS::SUCCESS){
        return state;
    }

    state = writeReg(UINT8_C(0x7E),UINT8_C(0x15)); // Turn on accel
    delay(100);

    if(state != I2C_STATUS::SUCCESS){
        return state;
    }

    state = writeReg(UINT8_C(0x7E),UINT8_C(0x18)); // Turn on accel
    delay(10);
    
    if(state != I2C_STATUS::SUCCESS){
        return state;
    }
    return state;
}


I2C_STATUS BMX160::getAllData(BMX160DataPacket &accel, BMX160DataPacket &gyro, BMX160DataPacket &magn)
{
    uint8_t buffer[20];
    I2C_STATUS state = this->readReg(UINT8_C(0x04),buffer,20);

    if(state != I2C_STATUS::SUCCESS){
        return state;
    }

    CopyBufferToDataPacket(accel,&buffer[14],ACCEL_Gs_PER_LSB*9.80665f);
    CopyBufferToDataPacket(gyro ,&buffer[8],GYRO_DEG_S_PER_LSB);
    CopyBufferToDataPacket(magn ,&buffer[0],MAGN_uT_PER_LSB);
    
 

    return state;
}


I2C_STATUS BMX160::writeReg(const uint8_t reg, const uint8_t byte)
{
    this->Wire.beginTransmission(this->address);
    this->Wire.write(reg);
    this->Wire.write(&byte, 1);
    return static_cast<I2C_STATUS>(this->Wire.endTransmission());
}

I2C_STATUS BMX160::readReg(const uint8_t reg, uint8_t& buffer)
{
    return this->readReg(reg, &buffer, 1);
}

I2C_STATUS BMX160::readReg(const uint8_t reg, uint8_t *const buffer, int length)
{
    // Send register to read
    this->Wire.beginTransmission(this->address);
    this->Wire.write(&reg, 1);
    I2C_STATUS status = static_cast<I2C_STATUS>(this->Wire.endTransmission(false)); // Error to be addressed, "false" to not release bus

    if (status != I2C_STATUS::SUCCESS)
    {
        return status;
    }

    // Read the result
    this->Wire.requestFrom(this->address, length);
    for (uint8_t i = 0; i < length; i++)
    {
        buffer[i] = this->Wire.read();
    }

    return static_cast<I2C_STATUS>(this->Wire.endTransmission());
}

// Add error codes https://docs.arduino.cc/language-reference/en/functions/communication/this->Wire/endTransmission/
I2C_STATUS BMX160::isConnected()
{
    this->Wire.beginTransmission(this->address);
    return static_cast<I2C_STATUS>(this->Wire.endTransmission());
}

// Other utility functions

inline void CopyBufferToDataPacket(BMX160DataPacket& packet, uint8_t* buffer, float conversionFactor){
    packet.x = ((int16_t)((uint16_t) buffer[1] << 8 | buffer[0])) * conversionFactor;
    packet.y = ((int16_t)((uint16_t) buffer[3] << 8 | buffer[2])) * conversionFactor;
    packet.z = ((int16_t)((uint16_t) buffer[5] << 8 | buffer[4])) * conversionFactor;
}