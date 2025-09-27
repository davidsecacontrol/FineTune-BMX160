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



BMX160::BMX160(arduino::TwoWire &Wire, uint8_t address) : Wire(Wire), address{address} {};

bool BMX160::begin()
{
    if(!this->setAccelPowerMode(ACCEL::POWER_MODE::NORMAL)) // Turn on accel
    {
        return false;
    }

    if(!this->setGyroPowerMode(GYRO::POWER_MODE::NORMAL)) // Turn on gyro
    {
        return false;
    }
    
    if(!this->setMagnInterfacePowerMode(MAGN_INTERFACE::POWER_MODE::NORMAL)){
        return false;
    }
    
    this->state = ERROR_CODE::ALL_OK;
    
    return true;
}

bool BMX160::setAccelRange(ACCEL::RANGE range)
{
    if(!this->writeReg(REGISTER::ACC_RANGE, static_cast<uint8_t>(range)))
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

bool BMX160::setGyroRange(GYRO::RANGE range)
{

    if(!this->writeReg(REGISTER::GYR_RANGE, static_cast<uint8_t>(range)))
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

bool BMX160::setAccelPowerMode(ACCEL::POWER_MODE power_mode){
    if(!this->writeReg(REGISTER::CMD,static_cast<uint8_t>(power_mode))){
        return false;
    }
    this->accelerometer_power_mode = power_mode;

    switch(power_mode){
        case ACCEL::POWER_MODE::SUSPEND:
            this->wait(1); // Takes 300us to reset MPU
            break;
        default:
            this->wait(5); // Takes Max 3.8 + 0.3 ms to turn on, whichever mode
    }

    return true;
}

bool BMX160::setGyroPowerMode(GYRO::POWER_MODE power_mode){
    if(!this->writeReg(REGISTER::CMD,static_cast<uint8_t>(power_mode))){
        return false;
    }

    this->gyroscope_power_mode = power_mode;

    switch(power_mode){
        case GYRO::POWER_MODE::SUSPEND:
            this->wait(1); // Takes 300us to reset MPU
            break;
        default:
            this->wait(81); // Takes Max 80 + 0.3 ms to turn on, whichever mode
    }

    return true;
}

bool BMX160::setMagnInterfacePowerMode(MAGN_INTERFACE::POWER_MODE power_mode){
    // All power mode changes require NORMAL mode first
    if(!this->writeReg(REGISTER::CMD,static_cast<uint8_t>(MAGN_INTERFACE::POWER_MODE::NORMAL))){
        return false;
    }
    this->wait(1); // 0.5 + 0.3 ms

    if(power_mode == MAGN_INTERFACE::POWER_MODE::SUSPEND){
        constexpr size_t length = 4;
        REGISTER regs[length] = {
            REGISTER::MAG_IF_0,     // Start read mode <7>= 0b1 with minimum delay <3:0> = 0b0000
            REGISTER::MAG_IF_3,     // Indirect write 0x00 (magn suspend mode) 
            REGISTER::MAG_IF_2,     // to indirect address 0x4B (magn power mode)
            REGISTER::CMD           // Set magn interface into suspend mode
        };
        uint8_t buffer[length] = {
            UINT8_C(0x80),
            static_cast<uint8_t>(MAGN::POWER_MODE::SUSPEND),
            static_cast<uint8_t>(MAGN::REGISTER::POWER_MODE),
            static_cast<uint8_t>(MAGN_INTERFACE::POWER_MODE::SUSPEND)
        };

        if(!writeReg(regs,buffer,length)){
            return false;
        }
        this->wait(1); // 0.5 + 0.3 ms

    }else{
        constexpr size_t length = 13;
        // Carry out initial setup
        REGISTER regs[length] = {
            REGISTER::MAG_IF_0,     // Start read mode <7>= 0b1 with minimum delay <3:0> = 0b0000
            REGISTER::MAG_IF_3,     // Indirect write 0x00 (magn suspend mode) 
            REGISTER::MAG_IF_2,     // to indirect address 0x4B (magn power mode)
            REGISTER::MAG_IF_3,     // Set REPXY regular preset
            REGISTER::MAG_IF_2,     // -
            REGISTER::MAG_IF_3,     // Set REPZ to regular preset
            REGISTER::MAG_IF_2,     // -
            REGISTER::MAG_IF_3,     // Prepare magn interface for data mode
            REGISTER::MAG_IF_2,     // -
            REGISTER::MAG_IF_1,     // -
            REGISTER::MAG_CONF,     // Set odr
            REGISTER::MAG_IF_0,     // Set magn interface to data mode
            REGISTER::CMD,          // Set magn interface power mode to desired one
        };

        uint8_t buffer[length] = {
            UINT8_C(0x80),
            static_cast<uint8_t>(MAGN::POWER_MODE::SLEEP),
            static_cast<uint8_t>(MAGN::REGISTER::POWER_MODE),
            static_cast<uint8_t>(MAGN::PRESETS::REPXY::REGULAR),
            static_cast<uint8_t>(MAGN::REGISTER::REPXY),
            static_cast<uint8_t>(MAGN::PRESETS::REPZ::REGULAR),
            static_cast<uint8_t>(MAGN::REGISTER::REPZ),
            UINT8_C(0x02),
            UINT8_C(0x4C),
            UINT8_C(0x42),
            static_cast<uint8_t>(this->magnetometer_interface_odr),
            static_cast<uint8_t>(this->magnetometer_interface_data_size),
            static_cast<uint8_t>(power_mode)
        };
        
        if(!writeReg(regs,buffer,length)){
            return false;
        }
        this->wait(1); // 0.5 + 0.3 ms
    }

    this->magnetometer_interface_power_mode = power_mode;

    return true;
}
    


bool BMX160::getAllData(DataPacket &accel, DataPacket &gyro, DataPacket &magn)
{
    if(this->accelerometer_power_mode          != ACCEL::POWER_MODE::NORMAL &&
       this->gyroscope_power_mode              != GYRO::POWER_MODE::NORMAL  &&
       this->magnetometer_interface_power_mode != MAGN_INTERFACE::POWER_MODE::NORMAL){

        this->state = ERROR_CODE::NO_BURST_READING_DATA_WHEN_ALL_SUSPENDED_OR_LOW_POWER;
        return false;
    } 
    uint8_t buffer[23]; // DATA from 0 to 19,  TIME from 20 to 22
    if(!this->readReg(REGISTER::DATA_0, buffer, 23)){
        return false;
    };

    CopyBufferToDataPacket(accel, &buffer[14], ACCEL::SENSITIVITY[static_cast<size_t>(this->accelerometer_range)] * G_TO_MS2);
    CopyBufferToDataPacket(gyro, &buffer[8], GYRO::SENSITIVITY[static_cast<size_t>(this->gyroscope_range)]);
    CopyBufferToDataPacket(magn, &buffer[0], MAGN::SENSITIVITY[static_cast<size_t>(this->magnetorquer_range)]);
    Serial.print("Buffer magn: [");
    for(size_t i = 0; i < 8; i++){
        Serial.print(buffer[i]);Serial.print(",");
    }
    Serial.println("]");

    uint32_t time =  (static_cast<uint32_t>(buffer[22]) << 16) | (static_cast<uint32_t>(buffer[21]) << 8) | buffer[20];

    accel.sensortime = time;
    gyro.sensortime = time;
    magn.sensortime = time;

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

    temp = static_cast<int16_t>(raw_data) * TEMP_SENSOR::SENSITIVITY[0]+OFFSET;

    return true;
}


bool BMX160::setAccelOdr(const ACCEL::ODR odr){
    // Check if correct odr -----------------------------
    if(odr > ACCEL::ODR::Hz1600 || odr < ACCEL::ODR::Hz25_over_32){ // ODR codeoutside of defined values
        this->state = ERROR_CODE::INVALID_ODR_SETTING;
        return false;
    }
    
    if(this->accelerometer_power_mode == ACCEL::POWER_MODE::NORMAL){
        if(odr > ACCEL::ODR::Hz1600 || odr < ACCEL::ODR::Hz25_over_2){  // ODR not allowed in normal mode
            this->state = ERROR_CODE::INVALID_ODR_SETTING;
            return false;
        }
    }
    
    // Transform from ACCEL::ODR to mask -------------------------------------------------------
    uint8_t mask = 0b00100000; // 0 (no undersampling) 010 (normal mode) 0000 (odr, to be filled)

    mask = mask | static_cast<uint8_t>(odr);

    // Write to IMU -------------------------------------------------------------------------------
    if(!writeReg(REGISTER::ACC_CONF,mask)){
        return false;
    }

    // Check if error flag is set (occurs if ODR is not allowed) ----------------------------------

    uint8_t byte_read = 0;
    if(!readReg(REGISTER::ERR_REG,byte_read)){
        return false;
    }

    byte_read = (byte_read & 0b00011110) >> 1; // Masking the error
    if(byte_read != 0){
        this->state = ERROR_CODE::ERR_REG;
        return false;
    }

    // All good -> update local state -------------------------------------------------------
    this->accelerometer_odr = odr;

    return true;
}




bool BMX160::getAccelOdr(ACCEL::ODR& odr){
    
    uint8_t byte;
    if(!readReg(REGISTER::ACC_CONF,byte)){
        return false;
    }

    byte = byte & 0b00001111; // Mask for only the odr bits
    odr = static_cast<ACCEL::ODR>(byte); 

    return true;
}

bool BMX160::setGyroOdr(const GYRO::ODR odr){
    // Check if correct odr -----------------------------
    if(odr > GYRO::ODR::Hz3200 || odr < GYRO::ODR::Hz25){ // ODR codeoutside of defined values
        this->state = ERROR_CODE::INVALID_ODR_SETTING;
        return false;
    }
    

    
    // Transform from GYRO::ODR to mask -------------------------------------------------------
    uint8_t mask = 0b00100000; // 0 (reserved, datasheet specifies "00", prob an error) 010 (normal mode) 0000 (odr, to be filled)

    mask = mask | static_cast<uint8_t>(odr);

    // Write to IMU -------------------------------------------------------------------------------
    if(!writeReg(REGISTER::GYR_CONF,mask)){
        return false;
    }

    // Check if error flag is set (occurs if ODR is not allowed) ----------------------------------

    uint8_t byte_read = 0;
    if(!readReg(REGISTER::ERR_REG,byte_read)){
        return false;
    }

    byte_read = (byte_read & 0b00011110) >> 1; // Masking the error
    if(byte_read != 0){
        this->state = ERROR_CODE::ERR_REG;
        return false;
    }

    // All good -> update local state -------------------------------------------------------
    this->gyroscope_odr = odr;

    return true;
}

bool BMX160::getGyroOdr(GYRO::ODR& odr){
    uint8_t byte;
    if(!readReg(REGISTER::GYR_CONF,byte)){
        return false;
    }

    byte = byte & 0b00001111; // Mask for only the odr bits
    odr = static_cast<GYRO::ODR>(byte); 

    return true;
}

bool BMX160::setMagnInterfaceOdr(const MAGN_INTERFACE::ODR odr){
    // Check if correct odr -----------------------------
    // See table 11 for allowed ODR w.r.t. preset -> using regular preset
    if(odr > MAGN_INTERFACE::ODR::Hz100 || odr < MAGN_INTERFACE::ODR::Hz25_over_2){ // ODR codeoutside of defined values
        this->state = ERROR_CODE::INVALID_ODR_SETTING;
        return false;
    }
    


    // Check if error flag is set (occurs if ODR is not allowed) ----------------------------------
    REGISTER regs[8] = {
        REGISTER::CMD,          // Set magn interface to normal
        REGISTER::MAG_IF_0,     // Start read mode <7>= 0b1 with minimum delay <3:0> = 0b0000
        REGISTER::MAG_IF_3,     // Prepare magn interface for data mode
        REGISTER::MAG_IF_2,     // -
        REGISTER::MAG_IF_1,     // -
        REGISTER::MAG_CONF,     // Set odr
        REGISTER::MAG_IF_0,     // Set magn interface to data mode
        REGISTER::CMD,          // Set magn interface power mode to desired one
    };

    uint8_t buffer[8] = {
        static_cast<uint8_t>(MAGN_INTERFACE::POWER_MODE::NORMAL), 
        UINT8_C(0x80),
        UINT8_C(0x02),
        UINT8_C(0x4C),
        UINT8_C(0x42),
        static_cast<uint8_t>(odr),
        static_cast<uint8_t>(this->magnetometer_interface_data_size),
        static_cast<uint8_t>(this->magnetometer_interface_power_mode)
    };
        // Write to IMU -------------------------------------------------------------------------------
    if(!writeReg(regs,buffer,8)){
        return false;
    }
    uint8_t byte_read = 0;
    if(!readReg(REGISTER::ERR_REG,byte_read)){
        return false;
    }

    byte_read = (byte_read & 0b00011110) >> 1; // Masking the error code
    if(byte_read != 0){
        this->state = ERROR_CODE::ERR_REG;
        return false;
    }

    // All good -> update local state -------------------------------------------------------
    this->magnetometer_interface_odr = odr;

    return true;   
}

bool BMX160::getMagnInterfaceOdr(MAGN_INTERFACE::ODR& odr){
    uint8_t byte;
    if(!readReg(REGISTER::MAG_CONF,byte)){
        return false;
    }

    byte = byte & 0b00001111; // Mask for only the odr bits
    odr = static_cast<MAGN_INTERFACE::ODR>(byte); 

    return true;
}


bool BMX160::getErrorRegister(uint8_t& error_code){
    if(!readReg(REGISTER::ERR_REG,error_code)){
        return false;
    }
    return true;
}


bool BMX160::writeReg(const REGISTER reg, const uint8_t byte)
{
    this->Wire.beginTransmission(this->address);
    this->Wire.write(static_cast<uint8_t>(reg));
    this->Wire.write(&byte, 1);
    this->state = static_cast<ERROR_CODE>(this->Wire.endTransmission());

    if(this->accelerometer_power_mode          != ACCEL::POWER_MODE::NORMAL &&
       this->gyroscope_power_mode              != GYRO::POWER_MODE::NORMAL  &&
       this->magnetometer_interface_power_mode != MAGN_INTERFACE::POWER_MODE::NORMAL){

        this->wait(1); // IIt is required to wait 0.4 ms before writes
    }
    
    return this->state == ERROR_CODE::ALL_OK;
}

bool BMX160::writeReg(REGISTER const * const regs, uint8_t *const buffer, size_t length){
    // Note that the begin/end transmission bufefr size is 32 bytes. Just in case, a new transmission is made for each one. This can be made faster by uniting all these into a single begin/end
    
    bool wait_per_write = 
        this->accelerometer_power_mode          != ACCEL::POWER_MODE::NORMAL &&
        this->gyroscope_power_mode              != GYRO::POWER_MODE::NORMAL  &&
        this->magnetometer_interface_power_mode != MAGN_INTERFACE::POWER_MODE::NORMAL;

    for(size_t i = 0; i < length; i++){ 
        this->Wire.beginTransmission(this->address);
        this->Wire.write(static_cast<uint8_t>(regs[i]));
        this->Wire.write(&buffer[i], 1);
        this->state = static_cast<ERROR_CODE>(this->Wire.endTransmission());
        if(wait_per_write){
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

bool BMX160::getChipID(uint8_t& chip_id){
    if(!this->readReg(REGISTER::CHIP_ID,chip_id)){
        return false;
    }

    return true;
}

void BMX160::wait(unsigned long time)
{
    delay(time);
}