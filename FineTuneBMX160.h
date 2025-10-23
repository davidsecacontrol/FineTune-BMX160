/**
 * @file FineTuneBMX160.h
 * @author David Secades (davidsecacontrol@gmail.com)
 * @brief Library header
 * @version 1.0.0
 * @date 2025-10-20
 *
 *
 */
#ifndef FINETUNE_BMX160
#define FINETUNE_BMX160

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

#include "registers.h"
#include "masks.h"

namespace FineTuneBMX160
{
    /** @brief Wire I2C error identifiers*/
    enum struct ERROR_CODE : uint8_t // DO NOT CHANGE ORDER OF I2C ELEMENTS
    {
        ALL_OK = UINT8_C(0),
        I2C_TOO_LONG_FOR_BUFFER = UINT8_C(1),
        I2C_NACK_ON_ADDRESS = UINT8_C(2),
        I2C_NACK_ON_DATA_TRANSMISSION = UINT8_C(3),
        I2C_OTHER = UINT8_C(4),
        I2C_TIMEOUT = UINT8_C(5),
        UNINITIALIZED = UINT8_C(6),
        INVALID_TEMPERATURE_MEASUREMENT = UINT8_C(7),
        NO_BURST_READING_DATA_WHEN_ALL_SUSPENDED_OR_LOW_POWER = UINT8_C(8),
        INVALID_ODR_SETTING = UINT8_C(9),
        ERR_REG = UINT8_C(10),
        INVALID_RANGE_SETTING = UINT8_C(11),
        INVALID_POWER_SETTING = UINT8_C(12)
    };

    constexpr uint32_t MAX_I2C_CLOCK_FREQUENCY = 1000000; ///< Maximum I2C clock frequency allowed for BMX160
    constexpr uint8_t CHIP_I2C_ADDRESS = 0x68;   ///< Standard I2C address for BMX160
    constexpr uint8_t CHIP_ID = 216;    ///< Chip ID for BMX160

    /** @brief Single sensor measurement */
    typedef struct
    {
        float x;             ///< x-axis
        float y;             ///< y-axis
        float z;             ///< z-axis
        uint32_t sensortime; ///< Local sensor time, 24 bits with wrapping, 39us/LSB
    } DataPacket;


    class TimingClassTemplate {
        public:
            /**
             * @brief Timing function, wait for time miliseconds.
             * 
             * @param time Delay time in miliseconds
             */
            virtual void wait(const uint32_t time);
    };

    class ArduinoBlockingDelay : public TimingClassTemplate {
        public:
        void wait(const uint32_t time) override {
            delay(time);
        }
    };

    class ComunicationProtocolTemplate {
        public:
        /**
         * @brief Returns true if IMU acknowledges conenction
         *
         * @return bool success/fail status
         */

        virtual bool isConnected();
        /**
         * @brief Sends a write command through the I2C protocol
         *
         * @param reg Reister to write to
         * @param byte 8-bit value to write
         * @return bool success/fail status
         */
        virtual uint8_t writeReg(const uint8_t reg, const uint8_t byte){
            return this->writeReg(&reg,&byte,1);
        }

        /**
         * @brief Sends a burst of write commands through the I2C protocol.
         *
         * @param reg Arrays of registers to write to
         * @param buffer Arrays of 8-bit values to write to
         * @param length Length of buffer and regs arrays
         * @return bool success/fail status
         */
        virtual uint8_t writeReg(uint8_t const *const reg, uint8_t const*const buffer, size_t length);

        /**
         * @brief Requests one byte through the I2C protocol
         *
         * @param reg Register to read from
         * @param buffer 8-bit buffer for storing read value
         * @return bool success/fail status
         */
        virtual uint8_t readReg(const uint8_t reg, uint8_t &buffer){
            return this->readReg(reg, &buffer, 1);
        }

        /**
         * @brief Requests a variable number of bytes through the I2C protocol
         *
         * @param reg Register to read from
         * @param buffer Pointer to 8-bit buffer of length length
         * @param length Length of buffer buffer
         * @return bool success/fail status
         */
        virtual uint8_t readReg(const uint8_t reg, uint8_t *const buffer, size_t length);
    };

    template<uint8_t I2C_address>
    class I2C_Protocol : public ComunicationProtocolTemplate{
        public:
        uint8_t readReg(const uint8_t reg, uint8_t &buffer) override {
            return this->readReg(reg, &buffer, 1);
        }
        uint8_t writeReg(const uint8_t reg, const uint8_t byte) override {
            return this->writeReg(&reg,&byte,1);
        }
        bool isConnected()
        {
            Wire.beginTransmission(I2C_address);
            return !Wire.endTransmission();
        }
        uint8_t writeReg(uint8_t const *const reg, uint8_t const*const buffer, size_t length) override {
            // Note that the begin/end transmission bufefr size is 32 bytes. Just in case, a new transmission is made for each one. This can be made faster by uniting all these into a single begin/end
            uint8_t state = 0; 
            for (size_t i = 0; i < length; i++)
            {
                Wire.beginTransmission(I2C_address);
                Wire.write(reg[i]);
                Wire.write(&buffer[i], 1);
                state = Wire.endTransmission();
                if(state != 0){ // I2C error
                    return state;
                }
            }

            return state;
        }


        uint8_t readReg(const uint8_t reg, uint8_t *const buffer, size_t length) override {
            uint8_t state = 0;
            // Send register to read
            Wire.beginTransmission(I2C_address);
            Wire.write(&reg, 1);
            state = Wire.endTransmission(false); // Error to be addressed, "false" to not release bus
            if (state != 0)
            {
                return state;
            }

            // Read the result
            Wire.requestFrom(I2C_address, length);
            for (size_t i = 0; i < length; i++)
            {
                buffer[i] = Wire.read();
            }
            state= Wire.endTransmission();
            return state;
        }
    };



    /**
     * @brief Sensor API. All communication with sensor should happen throuh this library
     *
     */
    template<class TimingClassTemplate, class ComunicationProtocolTemplate>
    class BMX160_Template
    {
    public:
        /**
         * @brief BMX60 error status. If different from ALL_OK handle error and clear this variable
         *
         */
        ERROR_CODE state = ERROR_CODE::UNINITIALIZED;


        BMX160_Template()  = default;

        // --------------------------------------------------------------------------

        /**
         * @brief Power up accelerometer, gyroscope and magnetometer(WIP)
         *
         * @return bool success/fail status
         */
        bool begin();

        /**
         * @brief Set specific accelerometer data range
         *
         * @param range Desired range
         * @return bool success/fail status
         */
        bool setAccelRange(const ACCEL::RANGE range);

        /**
         * @brief Copies to range the current range for the accelerometer
         *
         * @param range
         * @return bool success/fail status
         */
        bool getAccelRange(ACCEL::RANGE& range);

        /**
         * @brief Set specific gyroscope data range
         *
         * @param range Desired range
         * @return bool success/fail status
         */
        bool setGyroRange(const GYRO::RANGE range);

        /**
         * @brief Copies to range the current range for the gyroscope
         *
         * @param range
         * @return bool success/fail status
         */
        bool getGyroRange(GYRO::RANGE& range);

        /**
         * @brief Set the accelerometer power mode.
         *
         * @param power_mode
         * @return bool success/fail status
         */
        bool setAccelPowerMode(const ACCEL::POWER_MODE power_mode);

        /**
         * @brief Copies to power_mode the current power mode of the accelerometer
         *
         * @param power_mode
         * @return bool success/fail status
         */
        bool getAccelPowerMode(ACCEL::POWER_MODE& power_mode);

        /**
         * @brief Set the gyroscope power mode.
         *
         * @param power_mode
         * @return bool success/fail status
         */
        bool setGyroPowerMode(const GYRO::POWER_MODE power_mode);

        /**
         * @brief Copies to power_mode the current power mode of the gyroscope
         *
         * @param power_mode
         * @return bool success/fail status
         */
        bool getGyroPowerMode(GYRO::POWER_MODE& power_mode);

        /**
         * @brief Set the magnetometer power mode. This must be run after soft reset for the magn to work properly
         *
         *
         * @param power_mode
         * @return bool success/fail status
         */
        bool setMagnInterfacePowerMode(const MAGN_INTERFACE::POWER_MODE power_mode);

            /**
         * @brief Copies to power_mode the current power mode of the magnetometer interface
         *
         * @param power_mode
         * @return bool success/fail status
         */
        bool getMagnInterfacePowerMode(MAGN_INTERFACE::POWER_MODE& power_mode);

        /**
         * @brief Reads latest sensor data
         *
         * @param accel Object to store accelerometer data
         * @param gyro  Object to store gyroscope data
         * @param magn  Object to store magnetometer data
         * @return bool success/fail status
         */
        bool getAllData(DataPacket &accel, DataPacket &gyro, DataPacket &magn);

        /**
         * @brief Reads the sensor temperature data. Updates:
         * 
         * - All sensors suspended - never 
         * 
         * - Gyro normal mode - every 10ms 
         * 
         * - Gyro suspended / fastmode - every 1.28s alligning with SENSORTIME<15>
         *
         * @param temp Read temperature value
         * @return bool success/fail status
         */
        bool getTemp(float &temp);


        /**
         * @brief Retrieves the chip's id. Correct when == 216
         *
         * @param chip_id Variable to store read value
         * @return bool success/fail status
         */
        bool getChipID(uint8_t &chip_id);

        /**
         * @brief Sets data rate for the accelerometer. Note that in normal mode only 32/2Hz until 1600Hz are allowed. Oversampling and low_power mode not implemented
         *
         * @param odr Unsigned integer used to compute frequency
         * @return bool success/fail status
         */
        bool setAccelOdr(const ACCEL::ODR odr);

        /**
         * @brief Copies to odr the current sampling frequency for the accelerometer
         *
         * @param odr
         * @return bool success/fail status
         */
        bool getAccelOdr(ACCEL::ODR &odr);

        /**
         * @brief Sets data rate for the gyroscope. Note that only 25Hz until 3200Hz are allowed. Oversampling is not implemented
         *
         * @param odr Unsigned integer used to compute frequency
         * @return bool success/fail status
         */
        bool setGyroOdr(const GYRO::ODR odr);

        /**
         * @brief Copies to odr the current sampling frequency for the gyroscope
         *
         * @param odr
         * @return bool success/fail status
         */
        bool getGyroOdr(GYRO::ODR &odr);

        /**
         * @brief Sets data rate for the magnetometer interface. Note that only 25/32Hz until 800Hz are allowed. Oversampling is not implemented
         *
         * @param odr Unsigned integer used to compute frequency
         * @return bool success/fail status
         */
        bool setMagnInterfaceOdr(const MAGN_INTERFACE::ODR odr);

        /**
         * @brief Copies to odr the current sampling frequency for the magnetometer interface
         *
         * @param odr
         * @return bool success/fail status
         */
        bool getMagnInterfaceOdr(MAGN_INTERFACE::ODR &odr);

        /** 
         * @brief Copies the BMX160 error code 0x02<4:1> to error_code
         * 
         * @param error_code Variable to copy the error code to
         * @return bool success/fail status
         */
        bool getErrorRegister(uint8_t &error_code);
        
        /**
         * @brief Trigers a reboot of the whole BMX160
         * 
         * @return bool success/fail status
         */
        bool softReset();
        
    protected:
        TimingClassTemplate timer;
        ComunicationProtocolTemplate CommInterface;

        ACCEL::RANGE accelerometer_range = ACCEL::RANGE::G2; ///< Current accelerometer range
        GYRO::RANGE gyroscope_range = GYRO::RANGE::DPS2000;  ///< Current gyroscope range

        float accelerometer_sensitivity = ACCEL::SENSITIVITY[0]; ///< Current accelerometer sensitivity
        float gyroscope_sensitivity = GYRO::SENSITIVITY[0]; ///< Current gyroscope sensitivity
        float magnetometer_sensitivity = MAGN::SENSITIVITY[0];  ///< Current magnetometer sensitivity
        // IF ALL 3 (not interface) == SUSPEND -> DO NOT:  ADD RULES AND CHECK IF TRUE
        // - burst write                    IMPLEMENTED (not supported)
        // - Write without a 0.4 ms wait    IMPLEMENETED
        // - burst read on FIFO_DATA        IMPLEMENTED
        // IF ALL 3 (not interface) == SUSPEND / LOW_POWER -> DO NOT:
        // - read the FIFO
        ACCEL::POWER_MODE accelerometer_power_mode = ACCEL::POWER_MODE::SUSPEND; ///< Current accelerometer power mode
        GYRO::POWER_MODE gyroscope_power_mode = GYRO::POWER_MODE::SUSPEND;  ///< Current gyroscope power mode
        MAGN_INTERFACE::POWER_MODE magnetometer_interface_power_mode = MAGN_INTERFACE::POWER_MODE::SUSPEND; ///< Current magnetometer interface power mode

        ACCEL::ODR accelerometer_odr = ACCEL::ODR::Hz100; ///< Current accelerometer odr setting
        GYRO::ODR gyroscope_odr = GYRO::ODR::Hz100; ///< Current gyroscope odr setting
        MAGN_INTERFACE::ODR magnetometer_interface_odr = MAGN_INTERFACE::ODR::Hz100; ///< Current magnetometer odr setting

        MAGN_INTERFACE::DATA_SIZE magnetometer_interface_data_size = MAGN_INTERFACE::DATA_SIZE::XYZ_RHALL; ///< Current values to copy to BMX160 memory




        /**
         * @brief Continuously checks register 0x1B (STATUS) for bit <1> mag_man_op = 0.
         *
         * @return bool success/fail status
         */
        bool waitForMagn();

        /**
         * @brief Uses indirect writing to interface with the magnetometer sensor BMM150 directly. CAREFUL! MAGNETOMETER MUST BE SET INTO MANUAL MODE FIRST!
         *
         * @param reg BMM150 register to write to
         * @param data 8-bit value to write
         * @return bool success/fail status
         */
        bool MagnIndirectWrite(MAGN::REGISTER reg, uint8_t data);

        /**
         * @brief Uses indirect reading to interface with the magnetometer sensor BMM150 directly. CAREFUL! MAGNETOMETER MUST BE SET INTO MANUAL MODE FIRST!
         *
         * @param reg BMM150 register to read from
         * @param buffer 8-bit buffer for storing read value
         * @return bool success/fail status
         */
        bool MagnIndirectRead(MAGN::REGISTER reg, uint8_t &buffer);
    };

    #include"FineTuneBMX160.tpp"

    using BMX160 = BMX160_Template<ArduinoBlockingDelay,I2C_Protocol<CHIP_I2C_ADDRESS>>;
}






#endif