/**
 * @file FineTuneBMX160.h
 * @author David Secades (dasecacontrol@gmail.com)
 * @brief Library header
 * @version 0.1
 * @date 2025-08-18
 *
 *
 */
#ifndef FINETUNE_BMX160
#define FINETUNE_BMX160

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <stdint.h>
namespace FineTuneBMX160
{

    /** @brief BMX160 register bank idenitifiers*/
    enum struct REGISTER : uint8_t
    {
        CHIP_ID = UINT8_C(0x00),
        ERR_REG = UINT8_C(0x02),
        PMU_STATUS = UINT8_C(0x03),
        DATA_0 = UINT8_C(0x04),
        DATA_1 = UINT8_C(0x05),
        DATA_2 = UINT8_C(0x06),
        DATA_3 = UINT8_C(0x07),
        DATA_4 = UINT8_C(0x08),
        DATA_5 = UINT8_C(0x09),
        DATA_6 = UINT8_C(0x0A),
        DATA_7 = UINT8_C(0x0B),
        DATA_8 = UINT8_C(0x0C),
        DATA_9 = UINT8_C(0x0D),
        DATA_10 = UINT8_C(0x0E),
        DATA_11 = UINT8_C(0x0F),
        DATA_12 = UINT8_C(0x10),
        DATA_13 = UINT8_C(0x11),
        DATA_14 = UINT8_C(0x12),
        DATA_15 = UINT8_C(0x13),
        DATA_16 = UINT8_C(0x14),
        DATA_17 = UINT8_C(0x15),
        DATA_18 = UINT8_C(0x16),
        DATA_19 = UINT8_C(0x17),
        SENSOR_TIME_0 = UINT8_C(0x18),
        SENSOR_TIME_1 = UINT8_C(0x19),
        SENSOR_TIME_2 = UINT8_C(0x1A),
        STATUS = UINT8_C(0x1B),
        INT_STATUS_0 = UINT8_C(0x1C),
        INT_STATUS_1 = UINT8_C(0x1D),
        INT_STATUS_2 = UINT8_C(0x1E),
        INT_STATUS_3 = UINT8_C(0x1F),
        TEMPERATURE_0 = UINT8_C(0x20),
        TEMPERATURE_1 = UINT8_C(0x21),
        FIFO_LENGTH_0 = UINT8_C(0x22),
        FIFO_LENGTH_1 = UINT8_C(0x23),
        FIFO_DATA = UINT8_C(0x24),
        ACC_CONF = UINT8_C(0x40),
        ACC_RANGE = UINT8_C(0x41),
        GYR_CONF = UINT8_C(0x42),
        GYR_RANGE = UINT8_C(0x43),
        MAG_CONF = UINT8_C(0x44),
        FIFO_DOWNS = UINT8_C(0x45),
        FIFO_CONFIG_0 = UINT8_C(0x46),
        FIFO_CONFIG_1 = UINT8_C(0x47),
        MAG_IF_0 = UINT8_C(0x4C),
        MAG_IF_1 = UINT8_C(0x4D),
        MAG_IF_2 = UINT8_C(0x4E),
        MAG_IF_3 = UINT8_C(0x4F),
        INT_EN_0 = UINT8_C(0x50),
        INT_EN_1 = UINT8_C(0x51),
        INT_EN_2 = UINT8_C(0x52),
        INT_OUT_CTRL = UINT8_C(0x53),
        INT_LATCH = UINT8_C(0x54),
        INT_MAP_0 = UINT8_C(0x55),
        INT_MAP_1 = UINT8_C(0x56),
        INT_MAP_2 = UINT8_C(0x57),
        INT_DATA_0 = UINT8_C(0x58),
        INT_DATA_1 = UINT8_C(0x59),
        INT_LOWHIGH_0 = UINT8_C(0x5A),
        INT_LOWHIGH_1 = UINT8_C(0x5B),
        INT_LOWHIGH_2 = UINT8_C(0x5C),
        INT_LOWHIGH_3 = UINT8_C(0x5D),
        INT_LOWHIGH_4 = UINT8_C(0x5E),
        INT_MOTION_0 = UINT8_C(0x5F),
        INT_MOTION_1 = UINT8_C(0x60),
        INT_MOTION_2 = UINT8_C(0x61),
        INT_MOTION_3 = UINT8_C(0x62),
        INT_TAP_0 = UINT8_C(0x63),
        INT_TAP_1 = UINT8_C(0x64),
        INT_ORIENT_0 = UINT8_C(0x65),
        INT_ORIENT_1 = UINT8_C(0x66),
        INT_FLAT_0 = UINT8_C(0x67),
        INT_FLAT_1 = UINT8_C(0x68),
        FOC_CONF = UINT8_C(0x69),
        CONF = UINT8_C(0x6A),
        IF_CONF = UINT8_C(0x6B),
        PMU_TRIGGER = UINT8_C(0x6C),
        SELFT_TEST = UINT8_C(0x6D),
        NV_CONF = UINT8_C(0x70),
        OFFSET_0 = UINT8_C(0x71),
        OFFSET_1 = UINT8_C(0x72),
        OFFSET_2 = UINT8_C(0x73),
        OFFSET_3 = UINT8_C(0x74),
        OFFSET_4 = UINT8_C(0x75),
        OFFSET_5 = UINT8_C(0x76),
        OFFSET_6 = UINT8_C(0x77),
        STEP_CNT_0 = UINT8_C(0x78),
        STEP_CNT_1 = UINT8_C(0x79),
        STEP_CONF_0 = UINT8_C(0x7A),
        STEP_CONF_1 = UINT8_C(0x7B),
        CMD = UINT8_C(0x7E)
    };

    /** @brief Wire I2C error identifiers*/
    enum struct ERROR_CODE : uint8_t  // DO NOT CHANGE ORDER OF I2C ELEMENTS
    {
        ALL_OK = UINT8_C(0),
        I2C_TOO_LONG_FOR_BUFFER = UINT8_C(1),
        I2C_NACK_ON_ADDRESS = UINT8_C(2),
        I2C_NACK_ON_DATA_TRANSMISSION = UINT8_C(3),
        I2C_OTHER = UINT8_C(4),
        I2C_TIMEOUT = UINT8_C(5),
        UNINITIALIZED = UINT8_C(6)
    };


    namespace RANGE
    {
        /** @brief Allowed accelerometer ranges */
        enum struct ACCEL : int
        {
            G2 = 0,
            G4 = 1,
            G8 = 2,
            G16 = 3
        };

        /** @brief Allowed gyroscope ranges */
        enum struct GYRO : int
        {
            DPS2000 = 0,
            DPS1000 = 1,
            DPS500 = 2,
            DPS250 = 3,
            DPS150 = 4
        };

        /** @brief Allowed magnetometer range*/
        enum struct MAGN : int
        {
            uT0_3 = 0
        };
    }

    namespace POWER_MODE
    {
        /** @brief Allowed accelerometer power modes */
        enum struct ACCEL : int
        {
            NORMAL = 0,
            LOW_POWER = 1,
            SUSPEND = 2
        };

        /** @brief Allowed gyroscope power modes */
        enum struct GYRO : int
        {
            NORMAL = 0,
            FAST_STARTUP = 1,
            SUSPEND = 2
        };

        /** @brief Allowed magnetometer power modes*/
        enum struct MAGN : int
        {
            FORCE = 0,
            SLEEP = 1,
            SUSPEND = 2

        };
        enum struct MAGN_INTERFACE : int
        {
            NORMAL = 0,
            LOW_POWER = 1,
            SUSPEND = 2
        };
    }



    /** @brief Single sensor measurement */
    typedef struct
    {
        float x; ///< x-axis
        float y; ///< y-axis
        float z; ///< z-axis
    } DataPacket;

    /**
     * @brief Sensor API. All communication with sensor should happen throuh this library
     * 
     */
    class BMX160
    {
    public:

        ERROR_CODE state = ERROR_CODE::UNINITIALIZED;

        // Initializers --------------------------------------------------------------
        BMX160() = default;

        /**
         * @brief Constructor with specific Wire instance or device address
         *
         * @param Wire Wire instance
         * @param address Device address
         */
        BMX160(arduino::TwoWire &Wire, uint8_t address = UINT8_C(0x68));
        // --------------------------------------------------------------------------

        /**
         * @brief Power up accelerometer, gyroscope and magnetometer(WIP)
         *
         * @return uint8_t FineTuneBMX160::I2CStatus to identify failed transmission
         */
        [[nodiscard]] bool begin();

        /**
         * @brief Set specific accelerometer data range
         *
         * @param range Desired range
         * @return uint8_t FineTuneBMX160::I2CStatus to identify failed transmission
         */
        [[nodiscard]] bool setAccelRange(RANGE::ACCEL range);

        /**
         * @brief Set specific gyroscope data range
         *
         * @param range Desired range
         * @return uint8_t FineTuneBMX160::I2CStatus to identify failed transmission
         */
        [[nodiscard]] bool setGyroRange(RANGE::GYRO range);


        /**
         * @brief Set the accelerometer power mode. 
         * 
         * @param power_mode 
         * @return uint8_t FineTuneBMX160::I2CStatus to identify failed transmission 
         */
        [[nodiscard]] bool setAccelPowerMode(POWER_MODE::ACCEL power_mode);

        /**
         * @brief Set the gyroscope power mode. 
         * 
         * @param power_mode 
         * @return uint8_t FineTuneBMX160::I2CStatus to identify failed transmission 
         */
        [[nodiscard]] bool setGyroPowerMode(POWER_MODE::GYRO power_mode);

        /**
         * @brief Set the magnetometer power mode
         * 
         * 
         * @param power_mode 
         * @return uint8_t FineTuneBMX160::I2CStatus to identify failed transmission 
         */
        [[nodiscard]] bool setMagnPowerMode(POWER_MODE::MAGN power_mode);



        /**
         * @brief Reads latest sensor data
         *
         * @param accel Object to store accelerometer data
         * @param gyro  Object to store gyroscope data
         * @param magn  Object to store magnetometer data
         * @return uint8_t FineTuneBMX160::I2CStatus to identify failed transmission
         */
        [[nodiscard]] bool getAllData(DataPacket &accel, DataPacket &gyro, DataPacket &magn);

        /**
         * @brief Sends a write command through the I2C protocol
         *
         * @param reg Reister to write to
         * @param byte 8-bit value to write
         * @return uint8_t FineTuneBMX160::I2CStatus to identify failed transmission
         */
        [[nodiscard]] bool writeReg(const REGISTER reg, const uint8_t byte);

        /**
         * @brief Requests one byte through the I2C protocol
         *
         * @param reg Register to read from
         * @param buffer 8-bit buffer for storing read value
         * @return uint8_t FineTuneBMX160::I2CStatus to identify failed transmission
         */
        [[nodiscard]] bool readReg(const REGISTER reg, uint8_t &buffer);

        /**
         * @brief Requests a variable number of bytes through the I2C protocol
         *
         * @param reg Register to read from
         * @param buffer Pointer to 8-bit buffer of length length
         * @param length Length of buffer buffer
         * @return uint8_t FineTuneBMX160::I2CStatus to identify failed transmission
         */
        [[nodiscard]] bool readReg(const REGISTER reg, uint8_t *const buffer, size_t length);

        /**
         * @brief Returns true if IMU acknowledges conenction
         *
         * @return uint8_t FineTuneBMX160::I2CStatus to identify failed transmission
         */
        [[nodiscard]] bool isConnected();

    protected:
        arduino::TwoWire &Wire = Wire;         ///< Communication object to employ
        const uint8_t address = UINT8_C(0x68); ///< Sensor address

        RANGE::ACCEL accelerometer_range = RANGE::ACCEL::G2; ///< Current accelerometer range
        RANGE::GYRO gyroscope_range = RANGE::GYRO::DPS2000;  ///< Current gyroscope range
        RANGE::MAGN magnetorquer_range = RANGE::MAGN::uT0_3; ///< Current magnetometer range

        // IF ALL 3 (not interface) == SUSPEND -> DO NOT:  ADD RULES AND CHECK IF TRUE
        // - burst write
        // - Write without a 0.4 ms wait
        // - burst read on FIFO_DATA
        // IF ALL 3 (not interface) == SUSPEND / LOW_POWER -> DO NOT:
        // - read the FIFO
        POWER_MODE::ACCEL accelerometer_power_mode = POWER_MODE::ACCEL::SUSPEND;
        POWER_MODE::GYRO gyroscope_power_mode = POWER_MODE::GYRO::SUSPEND;
        POWER_MODE::MAGN magnetometer_power_mode = POWER_MODE::MAGN::SUSPEND;
        POWER_MODE::MAGN_INTERFACE magnetometer_interface_power_mode = POWER_MODE::MAGN_INTERFACE::SUSPEND;
        
    };

}

#endif