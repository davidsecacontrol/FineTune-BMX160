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
        0.3f
    };
    /** @brief Temperature sensor sensitivity presets */
    constexpr float TEMP[] = {
        1.0f/512.0f
    };
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
        0b00010001,
        0b00010010,
        0b00010000
    };

    /** @brief Masks for setting gyroscope power mode*/
    constexpr uint8_t GYRO_MPU[] = {
        0b00010101,
        0b00010111,
        0b00010100
    };

    /** @brief Masks for setting magnetometer power mode*/
    constexpr uint8_t MAGN_MPU[] = {

    };
}

BMX160::BMX160(arduino::TwoWire &Wire, uint8_t address) : Wire(Wire), address{address} {};

bool BMX160::begin()
{
    if(!this->setAccelPowerMode(POWER_MODE::ACCEL::NORMAL)) // Turn on accel
    {
        return false;
    }

    if(!this->setGyroPowerMode(POWER_MODE::GYRO::NORMAL)) // Turn on gyro
    {
        return false;
    }

    /*
    state = writeReg(REGISTER::CMD, UINT8_C(0x18)); // Turn on magn
    delay(10);
    */
    this->state = ERROR_CODE::ALL_OK;
    
    return true;
}

bool BMX160::setAccelRange(RANGE::ACCEL range)
{
    if(!this->writeReg(REGISTER::ACC_RANGE, MASK::ACCEL_RANGE[static_cast<size_t>(range)]))
    {
        return false;
    }

    this->accelerometer_range = range;

    // Recommended to perform a read to remove all stall data
    DataPacket buffer;
    if(!this->getAllData(buffer, buffer, buffer)){
        return false;
    }
    return true;
}

bool BMX160::setGyroRange(RANGE::GYRO range)
{

    if(!this->writeReg(REGISTER::GYR_RANGE, MASK::GYRO_RANGE[static_cast<size_t>(range)]))
    {
        return false;
    }
    this->gyroscope_range = range;

    // Recommended to perform a read to remove all stall data
    DataPacket buffer;
    if(!this->getAllData(buffer, buffer, buffer)){
        return false;
    }
    return true;
}

bool BMX160::setAccelPowerMode(POWER_MODE::ACCEL power_mode){
    if(!this->writeReg(REGISTER::CMD,MASK::ACCEL_MPU[static_cast<size_t>(power_mode)])){
        return false;
    }
    this->accelerometer_power_mode = power_mode;

    switch(power_mode){
        case POWER_MODE::ACCEL::SUSPEND:
            delay(1); // Takes 300us to reset MPU
            break;
        default:
            delay(5); // Takes Max 3.8 + 0.3 ms to turn on, whichever mode
    }

    return true;
}

bool BMX160::setGyroPowerMode(POWER_MODE::GYRO power_mode){
    if(!this->writeReg(REGISTER::CMD,MASK::GYRO_MPU[static_cast<size_t>(power_mode)])){
        return false;
    }

    this->gyroscope_power_mode = power_mode;

    switch(power_mode){
        case POWER_MODE::GYRO::SUSPEND:
            delay(1); // Takes 300us to reset MPU
            break;
        default:
            delay(81); // Takes Max 80 + 0.3 ms to turn on, whichever mode
    }

    return true;
}


bool BMX160::getAllData(DataPacket &accel, DataPacket &gyro, DataPacket &magn)
{
    uint8_t buffer[20];
    if(!this->readReg(REGISTER::DATA_0, buffer, 20)){
        return false;
    };

    CopyBufferToDataPacket(accel, &buffer[14], SENSITIVITY::ACCEL[static_cast<size_t>(this->accelerometer_range)] * G_TO_MS2);
    CopyBufferToDataPacket(gyro, &buffer[8], SENSITIVITY::GYRO[static_cast<size_t>(this->gyroscope_range)]);
    CopyBufferToDataPacket(magn, &buffer[0], SENSITIVITY::MAGN[static_cast<size_t>(this->magnetorquer_range)]);

    return true;
}

bool BMX160::getTemp(float& temp){
    uint8_t buffer[2];

    if(!this->readReg(REGISTER::TEMPERATURE_0,buffer,2)){
        return false;
    }
    /*
    // RAW READ
    constexpr char translation[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    Serial.print("Temp raw read: 0x");Serial.print(translation[buffer[1]>>4]);Serial.print(translation[buffer[1]&0x0F]);
    Serial.print(translation[buffer[0]>>4]);Serial.print(translation[buffer[0]&0x0F]);
    Serial.print(",");Serial.println( static_cast<int16_t>((static_cast<uint16_t>(buffer[0]) << 8) + buffer[1]) );
    */

    constexpr float OFFSET = 23.0f;

    uint16_t raw_data = (static_cast<uint16_t>(buffer[1]) << 8) + buffer[0];

    if(raw_data == 0x8000){ // From datasheet
        this->state = ERROR_CODE::INVALID_TEMPERATURE_MEASUREMENT;
        return false;
    }

    temp = static_cast<int16_t>(raw_data) * SENSITIVITY::TEMP[0]+OFFSET;

    return true;
}

bool BMX160::writeReg(const REGISTER reg, const uint8_t byte)
{
    this->Wire.beginTransmission(this->address);
    this->Wire.write(static_cast<uint8_t>(reg));
    this->Wire.write(&byte, 1);
    this->state = static_cast<ERROR_CODE>(this->Wire.endTransmission());

    return this->state == ERROR_CODE::ALL_OK;
}

bool BMX160::readReg(const REGISTER reg, uint8_t &buffer)
{
    return this->readReg(reg, &buffer, 1);

}

bool BMX160::readReg(const REGISTER reg, uint8_t *const buffer, size_t length)
{
    // Send register to read
    this->Wire.beginTransmission(this->address);
    this->Wire.write(reinterpret_cast<const uint8_t *>(&reg), 1);
    this->state = static_cast<ERROR_CODE>(this->Wire.endTransmission(false)); // Error to be addressed, "false" to not release bus

    if (this->state != ERROR_CODE::ALL_OK)
    {
        return false;
    }

    // Read the result
    this->Wire.requestFrom(this->address, length);
    for (size_t i = 0; i < length; i++)
    {
        buffer[i] = this->Wire.read();
    }
    this->state = static_cast<ERROR_CODE>(this->Wire.endTransmission());

    return this->state == ERROR_CODE::ALL_OK;
}

bool BMX160::isConnected()
{
    this->Wire.beginTransmission(this->address);
    this->state = static_cast<ERROR_CODE>(this->Wire.endTransmission());
    return this->state == ERROR_CODE::ALL_OK;
}

// Other utility functions

inline void CopyBufferToDataPacket(DataPacket &packet, uint8_t *buffer, float conversionFactor)
{
    packet.x = static_cast<int16_t>(static_cast<uint16_t>(buffer[1]) << 8 | buffer[0]) * conversionFactor;
    packet.y = static_cast<int16_t>(static_cast<uint16_t>(buffer[3]) << 8 | buffer[2]) * conversionFactor;
    packet.z = static_cast<int16_t>(static_cast<uint16_t>(buffer[5]) << 8 | buffer[4]) * conversionFactor;
}