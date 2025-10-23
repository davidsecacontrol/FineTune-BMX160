/**
 * @file FineTuneBMX160.cpp
 * @author David Secades (davidsecacontrol@gmail.com)
 * @brief Library source
 * @version 1.0.0
 * @date 2025-10-20
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

BMX160::BMX160(uint8_t address) : address{address} {};

bool BMX160::begin()
{

  this->state = ERROR_CODE::UNINITIALIZED ;  

    if(!this->softReset()){ // Reset IMU
        return false;
    }
    
    if (!this->setAccelPowerMode(ACCEL::POWER_MODE::NORMAL)) 
    {
        return false;
    }

    if (!this->setGyroPowerMode(GYRO::POWER_MODE::NORMAL)) 
    {
        return false;
    }

    if (!this->setMagnInterfacePowerMode(MAGN_INTERFACE::POWER_MODE::NORMAL))
    {
        return false;
    }

    this->state = ERROR_CODE::ALL_OK;

    return true;
}

bool BMX160::setAccelRange(const ACCEL::RANGE range)
{
    if( (range != ACCEL::RANGE::G2) && (range != ACCEL::RANGE::G4  ) && 
        (range != ACCEL::RANGE::G8) && (range != ACCEL::RANGE::G16 ) )
    {
        this->state = ERROR_CODE::INVALID_RANGE_SETTING;
        return false;
    }

    if (!this->writeReg(REGISTER::ACC_RANGE, static_cast<uint8_t>(range)))
    {
        return false;
    }

    this->accelerometer_range = range;

    switch (range)
    {
    case ACCEL::RANGE::G2:
        this->accelerometer_sensitivity = ACCEL::SENSITIVITY[0];
        break;
    case ACCEL::RANGE::G4:
        this->accelerometer_sensitivity = ACCEL::SENSITIVITY[1];
        break;
    case ACCEL::RANGE::G8:
        this->accelerometer_sensitivity = ACCEL::SENSITIVITY[2];
        break;
    case ACCEL::RANGE::G16:
        this->accelerometer_sensitivity = ACCEL::SENSITIVITY[3];
        break;
    default: 
        this->state = ERROR_CODE::INVALID_RANGE_SETTING;
        return false; 
    }

    // Recommended to perform a read to remove all stall data
    DataPacket buffer;
    if (!this->getAllData(buffer, buffer, buffer))
    {
        return false;
    }
    return true;
}

bool BMX160::getAccelRange(ACCEL::RANGE& range){
    
    uint8_t byte;
    if(!this->readReg(REGISTER::ACC_RANGE,byte)){
        return false;
    }
    byte &= 0b00001111; // Mask for <3:0>

    range = static_cast<ACCEL::RANGE>(byte);  

    // Datasheet: If a setting does not constitute one of the allowed ranges, interpret it as G2
    if(range != ACCEL::RANGE::G2 && range != ACCEL::RANGE::G4 && range != ACCEL::RANGE::G8 && range != ACCEL::RANGE::G16){
        range = ACCEL::RANGE::G2;
    }

    return true;
}


bool BMX160::setGyroRange(const GYRO::RANGE range)
{

 if ( ( range != GYRO::RANGE::DPS2000 ) && ( range != GYRO::RANGE::DPS1000 ) && 
      ( range != GYRO::RANGE::DPS500  ) && ( range != GYRO::RANGE::DPS250  ) && 
      ( range != GYRO::RANGE::DPS150  ) )  
    {
        this->state = ERROR_CODE::INVALID_RANGE_SETTING;
        return false;
    }

    if (!this->writeReg(REGISTER::GYR_RANGE, static_cast<uint8_t>(range)))
    {
        return false;
    }
    this->gyroscope_range = range;

    this->gyroscope_sensitivity = GYRO::SENSITIVITY[static_cast<size_t>(this->gyroscope_range)]; // ONly allowed as mask goes from 0 and up 1 by 1

    // Recommended to perform a read to remove all stall data
    DataPacket buffer;
    if (!this->getAllData(buffer, buffer, buffer))
    {
        return false;
    }
    return true;
}

bool BMX160::getGyroRange(GYRO::RANGE& range){
    
    uint8_t byte;
    if(!this->readReg(REGISTER::GYR_RANGE,byte)){
        return false;
    }
    byte &= 0b00000111; // Mask for <2:0>

    range = static_cast<GYRO::RANGE>(byte);  

    if( ( range != GYRO::RANGE::DPS2000 ) && ( range != GYRO::RANGE::DPS1000 ) && 
      ( range != GYRO::RANGE::DPS500  ) && ( range != GYRO::RANGE::DPS250  ) && 
      ( range != GYRO::RANGE::DPS150  ) )  
    {
        this->state = ERROR_CODE::INVALID_RANGE_SETTING;
        return false;
    }
    return true;
}

bool BMX160::setAccelPowerMode(const ACCEL::POWER_MODE power_mode)
{
    if (power_mode !=  (ACCEL::POWER_MODE::NORMAL) && (power_mode !=  ACCEL::POWER_MODE::LOW_POWER) && 
       (power_mode !=  ACCEL::POWER_MODE::SUSPEND) )
    {
        this->state = ERROR_CODE::INVALID_POWER_SETTING;
        return false;
    }

    if (!this->writeReg(REGISTER::CMD, static_cast<uint8_t>(power_mode)))
    {
        return false;
    }
    this->accelerometer_power_mode = power_mode;

    switch (power_mode)
    {
    case ACCEL::POWER_MODE::NORMAL:  
    case ACCEL::POWER_MODE::LOW_POWER:
        this->wait(5); // Takes Max 3.8 + 0.3 ms to turn on, whichever mode
       break;
    case ACCEL::POWER_MODE::SUSPEND:
        this->wait(1); // Takes 300us to reset MPU
        break;
    default:
        this->state = ERROR_CODE::INVALID_POWER_SETTING;
        return false;
    }

    return true;
}

bool BMX160::getAccelPowerMode(ACCEL::POWER_MODE& power_mode){
    uint8_t byte;
    if(!this->readReg(REGISTER::PMU_STATUS,byte)){
        return false;
    }
    
    switch((byte & 0b00110000) >> 4){
        case 0:
            power_mode = ACCEL::POWER_MODE::SUSPEND;
        break;
        case 1:
            power_mode = ACCEL::POWER_MODE::NORMAL;
        break;
        case 2:
            power_mode = ACCEL::POWER_MODE::LOW_POWER;
        break;

        default:
            this->state = ERROR_CODE::INVALID_POWER_SETTING;
            return false;
    }
    return true;
}

bool BMX160::setGyroPowerMode(const GYRO::POWER_MODE power_mode)
{
    if( (power_mode != GYRO::POWER_MODE::NORMAL       ) &&  
        (power_mode != GYRO::POWER_MODE::FAST_STARTUP ) &&
        (power_mode != GYRO::POWER_MODE::SUSPEND      ) ) 
    {
        this->state = ERROR_CODE::INVALID_POWER_SETTING;
        return false;
    }

    if (!this->writeReg(REGISTER::CMD, static_cast<uint8_t>(power_mode)))
    {
        return false;
    }

    this->gyroscope_power_mode = power_mode;

    switch (power_mode)
    {
    case GYRO::POWER_MODE::SUSPEND:
        this->wait(1); // Takes 300us to reset MPU
        break;
    default:
        this->wait(81); // Takes Max 80 + 0.3 ms to turn on, whichever mode
    }

    return true;
}

bool BMX160::getGyroPowerMode(GYRO::POWER_MODE& power_mode){
    uint8_t byte;
    if(!this->readReg(REGISTER::PMU_STATUS,byte)){
        return false;
    }
    
    switch((byte & 0b00001100) >> 2){
        case 0:
            power_mode = GYRO::POWER_MODE::SUSPEND;
        break;
        case 1:
            power_mode = GYRO::POWER_MODE::NORMAL;
        break;
        case 3:
            power_mode = GYRO::POWER_MODE::FAST_STARTUP;
        break;

        default:
            this->state = ERROR_CODE::INVALID_POWER_SETTING;
            return false;
    }

    return true;
}

bool BMX160::getMagnInterfacePowerMode(MAGN_INTERFACE::POWER_MODE& power_mode){
    uint8_t byte;
    if(!this->readReg(REGISTER::PMU_STATUS,byte)){
        return false;
    }
    switch(byte & 0b00000011){
        case 0:
            power_mode = MAGN_INTERFACE::POWER_MODE::SUSPEND;
        break;
        case 1:
            power_mode = MAGN_INTERFACE::POWER_MODE::NORMAL;
        break;
        case 2:
            power_mode = MAGN_INTERFACE::POWER_MODE::LOW_POWER;
        break;

        default:
            this->state = ERROR_CODE::INVALID_POWER_SETTING;
            return false;
    }

    return true;
}

bool BMX160::setMagnInterfacePowerMode(const MAGN_INTERFACE::POWER_MODE power_mode)
{
    if( (power_mode != MAGN_INTERFACE::POWER_MODE::LOW_POWER ) &&
        (power_mode != MAGN_INTERFACE::POWER_MODE::NORMAL    ) &&
        (power_mode != MAGN_INTERFACE::POWER_MODE::SUSPEND   ) )
    {
        this->state = ERROR_CODE::INVALID_POWER_SETTING;
        return false;   
    }
    // All power mode changes require NORMAL mode first
    if (!this->writeReg(REGISTER::CMD, static_cast<uint8_t>(MAGN_INTERFACE::POWER_MODE::NORMAL)))
    {
        return false;
    }
    this->wait(1); // 0.5 + 0.3 ms

    if (power_mode == MAGN_INTERFACE::POWER_MODE::SUSPEND)
    {
        if (!this->writeReg(REGISTER::MAG_IF_0, 0x80))
        { // Switch interface to setup mode
            return false;
        }

        if (!this->MagnIndirectWrite(MAGN::REGISTER::POWER_MODE, static_cast<uint8_t>(MAGN::POWER_MODE::SLEEP)))
        {
            return false;
        }
        if (!this->writeReg(REGISTER::CMD, static_cast<uint8_t>(MAGN_INTERFACE::POWER_MODE::SUSPEND)))
        {
            return false;
        }
        this->wait(1); // 0.5 + 0.3 ms
    }
    else
    {

        if (!this->writeReg(REGISTER::MAG_IF_0, 0x80))
        { // Switch interface to setup mode
            return false;
        }
        if (!this->MagnIndirectWrite(MAGN::REGISTER::POWER_MODE, static_cast<uint8_t>(MAGN::POWER_MODE::SLEEP)))
        {
            return false;
        }
        if (!this->MagnIndirectWrite(MAGN::REGISTER::REPXY, static_cast<uint8_t>(MAGN::PRESETS::REPXY::REGULAR)))
        {
            return false;
        }
        if (!this->MagnIndirectWrite(MAGN::REGISTER::REPZ, static_cast<uint8_t>(MAGN::PRESETS::REPZ::REGULAR)))
        {
            return false;
        }
        // Prepare interface for data mode
        if (!this->MagnIndirectWrite(static_cast<MAGN::REGISTER>(0x4C), 0x02))
        {
            return false;
        }
        if (!this->writeReg(REGISTER::MAG_IF_1, 0x42))
        {
            return false;
        }
        // Set odr and return to power mode
        if (!this->writeReg(REGISTER::MAG_CONF, static_cast<uint8_t>(this->magnetometer_interface_odr)))
        {
            return false;
        }
        if (!this->writeReg(REGISTER::MAG_IF_0, 0x03))
        { // Read 6 values
            return false;
        }
        if (!this->writeReg(REGISTER::CMD, static_cast<uint8_t>(power_mode)))
        {
            return false;
        }
        this->wait(1);
    }

    this->magnetometer_interface_power_mode = power_mode;

    return true;
}

bool BMX160::MagnIndirectWrite(MAGN::REGISTER reg, uint8_t data)
{
    if (!this->writeReg(REGISTER::MAG_IF_3, data))
    {
        return false;
    }
    if (!this->writeReg(REGISTER::MAG_IF_2, static_cast<uint8_t>(reg)))
    {
        return false;
    }
    if (!this->waitForMagn())
    {
        return false;
    }

    return true;
}

bool BMX160::MagnIndirectRead(MAGN::REGISTER reg, uint8_t &buffer)
{
    if (!this->writeReg(REGISTER::MAG_IF_1, static_cast<uint8_t>(reg)))
    {
        return false;
    }
    if (!this->waitForMagn())
    {
        return false;
    }
    if (!this->readReg(REGISTER::DATA_0, buffer))
    {
        return false;
    }

    return true;
}

bool BMX160::waitForMagn()
{
    bool waiting = true;
    uint8_t read_data;

    do
    {

        if (!this->readReg(REGISTER::STATUS, read_data))
        {
            return false;
        }

        waiting = (read_data & 0b00000010) != 0; // Wait until write operation is done

    } while (waiting);
    return true;
}

bool BMX160::getAllData(DataPacket &accel, DataPacket &gyro, DataPacket &magn)
{
    if (this->accelerometer_power_mode != ACCEL::POWER_MODE::NORMAL &&
        this->gyroscope_power_mode != GYRO::POWER_MODE::NORMAL &&
        this->magnetometer_interface_power_mode != MAGN_INTERFACE::POWER_MODE::NORMAL)
    {

        this->state = ERROR_CODE::NO_BURST_READING_DATA_WHEN_ALL_SUSPENDED_OR_LOW_POWER;
        return false;
    }
    uint8_t buffer[23]; // DATA from 0 to 19,  TIME from 20 to 22
    if (!this->readReg(REGISTER::DATA_0, buffer, 23))
    {
        return false;
    };

    CopyBufferToDataPacket(accel, &buffer[14], this->accelerometer_sensitivity * G_TO_MS2);
    CopyBufferToDataPacket(gyro, &buffer[8], this->gyroscope_sensitivity);
    CopyBufferToDataPacket(magn, &buffer[0], this->magnetometer_sensitivity);

    uint32_t time = (static_cast<uint32_t>(buffer[22]) << 16) | (static_cast<uint32_t>(buffer[21]) << 8) | buffer[20];

    accel.sensortime = time;
    gyro.sensortime = time;
    magn.sensortime = time;

    return true;
}

bool BMX160::getTemp(float &temp)
{
    uint8_t buffer[2];

    if (!this->readReg(REGISTER::TEMPERATURE_0, buffer, 2))
    {
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

    if (raw_data == 0x8000)
    { // From datasheet
        this->state = ERROR_CODE::INVALID_TEMPERATURE_MEASUREMENT;
        return false;
    }

    temp = static_cast<int16_t>(raw_data) * TEMP_SENSOR::SENSITIVITY[0] + OFFSET;

    return true;
}

bool BMX160::setAccelOdr(const ACCEL::ODR odr)
{
    // Check if correct odr -----------------------------
    if (odr > ACCEL::ODR::Hz1600 || odr < ACCEL::ODR::Hz25_over_32)
    { // ODR code outside of defined values
        this->state = ERROR_CODE::INVALID_ODR_SETTING;
        return false;
    }

    if (this->accelerometer_power_mode == ACCEL::POWER_MODE::NORMAL)
    {
        if (odr > ACCEL::ODR::Hz1600 || odr < ACCEL::ODR::Hz25_over_2)
        { // ODR not allowed in normal mode
            this->state = ERROR_CODE::INVALID_ODR_SETTING;
            return false;
        }
    }

    // Transform from ACCEL::ODR to mask -------------------------------------------------------
    uint8_t mask = 0b00100000; // 0 (no undersampling) 010 (normal mode) 0000 (odr, to be filled)

    mask = mask | static_cast<uint8_t>(odr);

    // Write to IMU -------------------------------------------------------------------------------
    if (!writeReg(REGISTER::ACC_CONF, mask))
    {
        return false;
    }

    // Check if error flag is set (occurs if ODR is not allowed) ----------------------------------

    uint8_t byte_read = 0;
    if (!readReg(REGISTER::ERR_REG, byte_read))
    {
        return false;
    }

    byte_read = (byte_read & 0b00011110) >> 1; // Masking the error
    if (byte_read != 0)
    {
        this->state = ERROR_CODE::ERR_REG;
        return false;
    }

    // All good -> update local state -------------------------------------------------------
    this->accelerometer_odr = odr;

    return true;
}

bool BMX160::getAccelOdr(ACCEL::ODR &odr)
{

    uint8_t byte;
    if (!readReg(REGISTER::ACC_CONF, byte))
    {
        return false;
    }

    byte = byte & 0b00001111; // Mask for only the odr bits
    odr = static_cast<ACCEL::ODR>(byte);

    if (odr > ACCEL::ODR::Hz1600 || odr < ACCEL::ODR::Hz25_over_32)
    { // ODR codeo utside of defined values
        this->state = ERROR_CODE::INVALID_ODR_SETTING;
        return false;
    }

    return true;
}

bool BMX160::setGyroOdr(const GYRO::ODR odr)
{
    // Check if correct odr -----------------------------
    if (odr > GYRO::ODR::Hz3200 || odr < GYRO::ODR::Hz25)
    { // ODR codeoutside of defined values
        this->state = ERROR_CODE::INVALID_ODR_SETTING;
        return false;
    }

    // Transform from GYRO::ODR to mask -------------------------------------------------------
    uint8_t mask = 0b00100000; // 0 (reserved, datasheet specifies "00", prob an error) 010 (normal mode) 0000 (odr, to be filled)

    mask = mask | static_cast<uint8_t>(odr);

    // Write to IMU -------------------------------------------------------------------------------
    if (!writeReg(REGISTER::GYR_CONF, mask))
    {
        return false;
    }

    // Check if error flag is set (occurs if ODR is not allowed) ----------------------------------

    uint8_t byte_read = 0;
    if (!readReg(REGISTER::ERR_REG, byte_read))
    {
        return false;
    }

    byte_read = (byte_read & 0b00011110) >> 1; // Masking the error
    if (byte_read != 0)
    {
        this->state = ERROR_CODE::ERR_REG;
        return false;
    }

    // All good -> update local state -------------------------------------------------------
    this->gyroscope_odr = odr;

    return true;
}

bool BMX160::getGyroOdr(GYRO::ODR &odr)
{
    uint8_t byte;
    if (!readReg(REGISTER::GYR_CONF, byte))
    {
        return false;
    }

    byte = byte & 0b00001111; // Mask for only the odr bits
    odr = static_cast<GYRO::ODR>(byte);

    if (odr > GYRO::ODR::Hz1600 || odr < GYRO::ODR::Hz25)
    { // ODR codeoutside of defined values
        this->state = ERROR_CODE::INVALID_ODR_SETTING;
        return false;
    }
    return true;
}

bool BMX160::setMagnInterfaceOdr(const MAGN_INTERFACE::ODR odr)
{
    // Check if correct odr -----------------------------
    // See table 11 for allowed ODR w.r.t. preset -> using regular preset
    if (odr > MAGN_INTERFACE::ODR::Hz100 || odr < MAGN_INTERFACE::ODR::Hz25_over_2)
    { // ODR codeoutside of defined values
        this->state = ERROR_CODE::INVALID_ODR_SETTING;
        return false;
    }

    // Check if error flag is set (occurs if ODR is not allowed) ----------------------------------
    REGISTER regs[8] = {
        REGISTER::CMD,      // Set magn interface to normal
        REGISTER::MAG_IF_0, // Start read mode <7>= 0b1 with minimum delay <3:0> = 0b0000
        REGISTER::MAG_IF_3, // Prepare magn interface for data mode
        REGISTER::MAG_IF_2, // -
        REGISTER::MAG_IF_1, // -
        REGISTER::MAG_CONF, // Set odr
        REGISTER::MAG_IF_0, // Set magn interface to data mode
        REGISTER::CMD,      // Set magn interface power mode to desired one
    };

    uint8_t buffer[8] = {
        static_cast<uint8_t>(MAGN_INTERFACE::POWER_MODE::NORMAL),
        UINT8_C(0x80),
        UINT8_C(0x02),
        UINT8_C(0x4C),
        UINT8_C(0x42),
        static_cast<uint8_t>(odr),
        static_cast<uint8_t>(this->magnetometer_interface_data_size),
        static_cast<uint8_t>(this->magnetometer_interface_power_mode)};
    // Write to IMU -------------------------------------------------------------------------------
    if (!writeReg(regs, buffer, 8))
    {
        return false;
    }
    uint8_t byte_read = 0;
    if (!readReg(REGISTER::ERR_REG, byte_read))
    {
        return false;
    }

    byte_read = (byte_read & 0b00011110) >> 1; // Masking the error code
    if (byte_read != 0)
    {
        this->state = ERROR_CODE::ERR_REG;
        return false;
    }

    // All good -> update local state -------------------------------------------------------
    this->magnetometer_interface_odr = odr;

    return true;
}

bool BMX160::getMagnInterfaceOdr(MAGN_INTERFACE::ODR &odr)
{
    uint8_t byte;
    if (!readReg(REGISTER::MAG_CONF, byte))
    {
        return false;
    }

    byte = byte & 0b00001111; // Mask for only the odr bits
    odr = static_cast<MAGN_INTERFACE::ODR>(byte);

    if (odr > MAGN_INTERFACE::ODR::Hz100 || odr < MAGN_INTERFACE::ODR::Hz25_over_2)
    { // ODR codeoutside of defined values
        this->state = ERROR_CODE::INVALID_ODR_SETTING;
        return false;
    }
    
    return true;
}

bool BMX160::getErrorRegister(uint8_t &error_code)
{
    if (!readReg(REGISTER::ERR_REG, error_code))
    {
        return false;
    }
    return true;
}

bool BMX160::softReset(){
    if(!writeReg(REGISTER::CMD,UINT8_C(0xB6))){
        return false;
    }
    return true;
}


bool BMX160::writeReg(const REGISTER reg, const uint8_t byte)
{
    return this->writeReg(&reg,&byte,1);
}

bool BMX160::writeReg(REGISTER const *const regs, uint8_t const*const buffer, size_t length)
{
    // Note that the begin/end transmission bufefr size is 32 bytes. Just in case, a new transmission is made for each one. This can be made faster by uniting all these into a single begin/end

    bool wait_per_write =
        this->accelerometer_power_mode != ACCEL::POWER_MODE::NORMAL &&
        this->gyroscope_power_mode != GYRO::POWER_MODE::NORMAL &&
        this->magnetometer_interface_power_mode != MAGN_INTERFACE::POWER_MODE::NORMAL;

    for (size_t i = 0; i < length; i++)
    {
        Wire.beginTransmission(this->address);
        Wire.write(static_cast<uint8_t>(regs[i]));
        Wire.write(&buffer[i], 1);
        this->state = static_cast<ERROR_CODE>(Wire.endTransmission());
        if (wait_per_write)
        {
            this->wait(1); // It is required to wait 0.4 ms before writes if all sensors suspended / low power
        }
    }

    return this->state == ERROR_CODE::ALL_OK;
}

bool BMX160::readReg(const REGISTER reg, uint8_t &buffer)
{
    return this->readReg(reg, &buffer, 1);
}

bool BMX160::readReg(const REGISTER reg, uint8_t *const buffer, size_t length)
{
    // Send register to read
    Wire.beginTransmission(this->address);
    Wire.write(reinterpret_cast<const uint8_t *>(&reg), 1);
    this->state = static_cast<ERROR_CODE>(Wire.endTransmission(false)); // Error to be addressed, "false" to not release bus

    if (this->state != ERROR_CODE::ALL_OK)
    {
        return false;
    }

    // Read the result
    Wire.requestFrom(this->address, length);
    for (size_t i = 0; i < length; i++)
    {
        buffer[i] = Wire.read();
    }
    this->state = static_cast<ERROR_CODE>(Wire.endTransmission());

    return this->state == ERROR_CODE::ALL_OK;
}

bool BMX160::isConnected()
{
    Wire.beginTransmission(this->address);
    this->state = static_cast<ERROR_CODE>(Wire.endTransmission());
    return this->state == ERROR_CODE::ALL_OK;
}

// Other utility functions

inline void CopyBufferToDataPacket(DataPacket &packet, uint8_t *buffer, float conversionFactor)
{
    packet.x = static_cast<int16_t>(static_cast<uint16_t>(buffer[1]) << 8 | buffer[0]) * conversionFactor;
    packet.y = static_cast<int16_t>(static_cast<uint16_t>(buffer[3]) << 8 | buffer[2]) * conversionFactor;
    packet.z = static_cast<int16_t>(static_cast<uint16_t>(buffer[5]) << 8 | buffer[4]) * conversionFactor;
}

bool BMX160::getChipID(uint8_t &chip_id)
{
    if (!this->readReg(REGISTER::CHIP_ID, chip_id))
    {
        return false;
    }

    return true;
}

void BMX160::wait(unsigned long time)
{
    delay(time);
}