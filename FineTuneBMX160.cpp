/**
 * @file FineTuneBMX160.cpp
 * @author your name (you@domain.com)
 * @brief Library source
 * @version 0.1
 * @date 2025-08-18
 * 
 * 
 */
#include "FineTuneBMX160.h"

using namespace FineTuneBMX160;

// Utility functions:----------------

/**
 * @brief Receives accel/magn/gyro data in uint8_t format and converts it into float values
 * 
 * @param packet Output float data
 * @param buffer Input uint8_t data
 * @param conversionFactor Sensitivity of LSB
 */
inline void CopyBufferToDataPacket(DataPacket &packet, uint8_t *buffer, float conversionFactor);

// ---------------------------------

/** @brief Conversion factor from Gs to m^2/s */
constexpr float G_TO_MS2 = 9.80665f;


namespace SENSITIVITY
{
    /** @brief Accelerometer sensitivity presets*/
    constexpr float ACCEL[] = {
        1.0f / 16384,
        1.0f / 8192,
        1.0f / 4096,
        1.0f / 2048};

    /** @brief Gyroscope sensisitivy presets*/
    constexpr float GYRO[] = {
        1.0f / 16.4f,
        1.0f / 32.8f,
        1.0f / 65.6f,
        1.0f / 131.2f};

    /** @brief Magnetometer sensitivity presets*/
    constexpr float MAGN[] = {
        0.3f};
}

namespace MASK
{
    /** @brief Masks for setting accelerometer range*/
    constexpr uint8_t ACCEL_RANGE[] = {
        0b00000011,
        0b00000101,
        0b00001000,
        0b00001100
    };

    /** @brief Masks for setting gyroscope range*/
    constexpr uint8_t GYRO_RANGE[] = {
        0b00000000,
        0b00000001,
        0b00000010,
        0b00000011,
        0b00000100
    };

    /** @brief Masks for setting accelerometer power mode*/
    constexpr uint8_t ACCEL_MPU[] = {
    
    };

    /** @brief Masks for setting gyroscope power mode*/
    constexpr uint8_t GYRO_MPU[] = {

    };

    /** @brief Masks for setting magnetometer power mode*/
    constexpr uint8_t MAGN_MPU[] = {

    };
}

BMX160::BMX160(arduino::TwoWire &Wire, uint8_t address) : Wire(Wire), address{address} {};

I2C_STATUS BMX160::begin()
{
    I2C_STATUS state = writeReg(REGISTER::CMD, UINT8_C(0x11)); // Turn on accel
    delay(10);

    if (state != I2C_STATUS::SUCCESS)
    {
        return state;
    }

    state = writeReg(REGISTER::CMD, UINT8_C(0x15)); // Turn on gyro
    delay(100);

    if (state != I2C_STATUS::SUCCESS)
    {
        return state;
    }

    /*
    state = writeReg(REGISTER::CMD, UINT8_C(0x18)); // Turn on magn
    delay(10);
    */
    return state;
}

I2C_STATUS BMX160::setAccelRange(RANGE::ACCEL range)
{
    I2C_STATUS state = writeReg(REGISTER::ACC_RANGE, MASK::ACCEL_RANGE[static_cast<size_t>(range)]);

    if (state != I2C_STATUS::SUCCESS)
    {
        return state;
    }
    this->accelerometer_range = range;

    // Recommended to perform a read to remove all stall data
    DataPacket buffer;
    return getAllData(buffer, buffer, buffer);
}

I2C_STATUS BMX160::setGyroRange(RANGE::GYRO range)
{

    I2C_STATUS state = writeReg(REGISTER::GYR_RANGE, MASK::GYRO_RANGE[static_cast<size_t>(range)]);

    if (state != I2C_STATUS::SUCCESS)
    {
        return state;
    }
    this->gyroscope_range = range;

    // Recommended to perform a read to remove all stall data
    DataPacket buffer;
    return getAllData(buffer, buffer, buffer);
}

I2C_STATUS BMX160::getAllData(DataPacket &accel, DataPacket &gyro, DataPacket &magn)
{
    uint8_t buffer[20];
    I2C_STATUS state = this->readReg(REGISTER::DATA_0, buffer, 20);

    if (state != I2C_STATUS::SUCCESS)
    {
        return state;
    }

    CopyBufferToDataPacket(accel, &buffer[14], SENSITIVITY::ACCEL[static_cast<size_t>(this->accelerometer_range)] * G_TO_MS2);
    CopyBufferToDataPacket(gyro, &buffer[8], SENSITIVITY::GYRO[static_cast<size_t>(this->gyroscope_range)]);
    CopyBufferToDataPacket(magn, &buffer[0], SENSITIVITY::MAGN[static_cast<size_t>(this->magnetorquer_range)]);

    return state;
}

I2C_STATUS BMX160::writeReg(const REGISTER reg, const uint8_t byte)
{
    this->Wire.beginTransmission(this->address);
    this->Wire.write(static_cast<uint8_t>(reg));
    this->Wire.write(&byte, 1);
    return static_cast<I2C_STATUS>(this->Wire.endTransmission());
}

I2C_STATUS BMX160::readReg(const REGISTER reg, uint8_t &buffer)
{
    return this->readReg(reg, &buffer, 1);
}

I2C_STATUS BMX160::readReg(const REGISTER reg, uint8_t *const buffer, size_t length)
{
    // Send register to read
    this->Wire.beginTransmission(this->address);
    this->Wire.write(reinterpret_cast<const uint8_t *>(&reg), 1);
    I2C_STATUS status = static_cast<I2C_STATUS>(this->Wire.endTransmission(false)); // Error to be addressed, "false" to not release bus

    if (status != I2C_STATUS::SUCCESS)
    {
        return status;
    }

    // Read the result
    this->Wire.requestFrom(this->address, length);
    for (size_t i = 0; i < length; i++)
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
    packet.x = static_cast<int16_t>(static_cast<uint16_t>(buffer[1]) << 8 | buffer[0]) * conversionFactor;
    packet.y = static_cast<int16_t>(static_cast<uint16_t>(buffer[3]) << 8 | buffer[2]) * conversionFactor;
    packet.z = static_cast<int16_t>(static_cast<uint16_t>(buffer[5]) << 8 | buffer[4]) * conversionFactor;

}