#include "FineTuneBMX160.h"

using namespace FineTuneBMX160;

// Utility functions:----------------
inline void CopyBufferToDataPacket(DataPacket &packet, uint8_t *buffer, float conversionFactor);

// ---------------------------------

constexpr float G_TO_MS2 = 9.80665f;

namespace SENSITIVITY
{
    constexpr float ACCEL[] = { 
        1.0f / 16384,
        1.0f / 8192,
        1.0f / 4096,
        1.0f / 2048
    };
    
    constexpr float GYRO[] = {  
        1.0f/16.4f,
        1.0f/32.8f,
        1.0f/65.6f,
        1.0f/131.2f
    };

    constexpr float MAGN[] = {
        0.3f
    };
}

namespace MASK{
    constexpr uint8_t ACCEL_RANGE[] = {
        0b00000011,
        0b00000101,
        0b00001000,
        0b00001100
    };
    constexpr uint8_t GYRO_RANGE[] = {
        0b00000000,
        0b00000001,
        0b00000010,
        0b00000011,
        0b00000100
    };
}

BMX160::BMX160(arduino::TwoWire &Wire, uint8_t address) : Wire(Wire), address{address} {};

I2C_STATUS BMX160::begin()
{
    I2C_STATUS state = writeReg(UINT8_C(0x7E), UINT8_C(0x11)); // Turn on accel
    delay(10);

    if (state != I2C_STATUS::SUCCESS)
    {
        return state;
    }

    state = writeReg(UINT8_C(0x7E), UINT8_C(0x15)); // Turn on accel
    delay(100);

    if (state != I2C_STATUS::SUCCESS)
    {
        return state;
    }

    state = writeReg(UINT8_C(0x7E), UINT8_C(0x18)); // Turn on accel
    delay(10);

    if (state != I2C_STATUS::SUCCESS)
    {
        return state;
    }
    return state;
}

I2C_STATUS BMX160::setAccelRange(RANGE::ACCEL range){
    I2C_STATUS state = writeReg(UINT8_C(0x41),MASK::ACCEL_RANGE[(int) range]);
    
    if(state != I2C_STATUS::SUCCESS){
        return state;
    }
    this->accelerometer_range = range;

    // Recommended to perform a read to remove all stall data
    DataPacket buffer;
    return getAllData(buffer,buffer,buffer);

}

I2C_STATUS BMX160::setGyroRange(RANGE::GYRO range){

    I2C_STATUS state = writeReg(UINT8_C(0x43),MASK::GYRO_RANGE[(int) range]);

    if(state != I2C_STATUS::SUCCESS){
        return state;
    }
    this->gyroscope_range = range;

    // Recommended to perform a read to remove all stall data
    DataPacket buffer;
    return getAllData(buffer,buffer,buffer);

}


I2C_STATUS BMX160::getAllData(DataPacket &accel, DataPacket &gyro, DataPacket &magn)
{
    uint8_t buffer[20];
    I2C_STATUS state = this->readReg(UINT8_C(0x04), buffer, 20);

    if (state != I2C_STATUS::SUCCESS)
    {
        return state;
    }

    CopyBufferToDataPacket(accel, &buffer[14], SENSITIVITY::ACCEL[(int) this->accelerometer_range]*G_TO_MS2);
    CopyBufferToDataPacket(gyro,  &buffer[8] , SENSITIVITY::GYRO[(int) this->gyroscope_range]);
    CopyBufferToDataPacket(magn,  &buffer[0] , SENSITIVITY::MAGN[(int)this->magnetorquer_range]);

    return state;
}

I2C_STATUS BMX160::writeReg(const uint8_t reg, const uint8_t byte)
{
    this->Wire.beginTransmission(this->address);
    this->Wire.write(reg);
    this->Wire.write(&byte, 1);
    return static_cast<I2C_STATUS>(this->Wire.endTransmission());
}

I2C_STATUS BMX160::readReg(const uint8_t reg, uint8_t &buffer)
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

inline void CopyBufferToDataPacket(DataPacket &packet, uint8_t *buffer, float conversionFactor)
{
    packet.x = ((int16_t)((uint16_t)buffer[1] << 8 | buffer[0])) * conversionFactor;
    packet.y = ((int16_t)((uint16_t)buffer[3] << 8 | buffer[2])) * conversionFactor;
    packet.z = ((int16_t)((uint16_t)buffer[5] << 8 | buffer[4])) * conversionFactor;
}