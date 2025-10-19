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
        ERR_REG = UINT8_C(10)
    };

    namespace ACCEL
    {

        /** @brief Allowed accelerometer ranges. Values represent mask */
        enum struct RANGE : uint8_t
        {
            G2 = 0b00000011,
            G4 = 0b00000101,
            G8 = 0b00001000,
            G16 = 0b00001100
        };

        /** @brief Allowed accelerometer power modes. Values represent the mask */
        enum struct POWER_MODE : uint8_t
        {
            NORMAL = 0b00010001,
            LOW_POWER = 0b00010010,
            SUSPEND = 0b00010000
        };

        enum struct ODR : uint8_t
        {
            Hz25_over_32 = UINT8_C(1),
            Hz25_over_16 = UINT8_C(2),
            Hz25_over_8 = UINT8_C(3),
            Hz25_over_4 = UINT8_C(4),
            Hz25_over_2 = UINT8_C(5),
            Hz25 = UINT8_C(6),
            Hz50 = UINT8_C(7),
            Hz100 = UINT8_C(8),
            Hz200 = UINT8_C(9),
            Hz400 = UINT8_C(10),
            Hz800 = UINT8_C(11),
            Hz1600 = UINT8_C(12)
        };

        /** @brief Accelerometer sensitivity presets*/
        constexpr float SENSITIVITY[] = {
            1.0f / 16384,
            1.0f / 8192,
            1.0f / 4096,
            1.0f / 2048};
    }

    namespace GYRO
    {

        /** @brief Allowed gyroscope ranges. Values represent mask */
        enum struct RANGE : uint8_t
        {
            DPS2000 = 0b00000000,
            DPS1000 = 0b00000001,
            DPS500 = 0b00000010,
            DPS250 = 0b00000011,
            DPS150 = 0b00000100
        };

        /** @brief Allowed gyroscope power modes. Values represent mask */
        enum struct POWER_MODE : uint8_t
        {
            NORMAL = 0b00010101,
            FAST_STARTUP = 0b00010111,
            SUSPEND = 0b00010100
        };

        enum struct ODR : uint8_t
        {
            Hz25 = UINT8_C(6),
            Hz50 = UINT8_C(7),
            Hz100 = UINT8_C(8),
            Hz200 = UINT8_C(9),
            Hz400 = UINT8_C(10),
            Hz800 = UINT8_C(11),
            Hz1600 = UINT8_C(12),
            Hz3200 = UINT8_C(13)
        };

        /** @brief Gyroscope sensisitivy presets*/
        constexpr float SENSITIVITY[] = {
            1.0f / 16.4f,
            1.0f / 32.8f,
            1.0f / 65.6f,
            1.0f / 131.2f,
            1.0f / 262.4f
        };
    }

    /**
     * @brief Embedded magnetometer chip inside the BMX160. Can be used via indirect addressing
     *
     */
    namespace MAGN
    {
        /** @brief Magnetometer addresses for indirect addressing */
        enum struct REGISTER : uint8_t
        {
            POWER_MODE = UINT8_C(0x4B),
            REPXY = UINT8_C(0x51),
            REPZ = UINT8_C(0x52)
        };

        namespace PRESETS
        {
            enum struct REPXY : uint8_t
            {
                LOW_POWER = UINT8_C(0x01),
                REGULAR = UINT8_C(0x04),
                ENHANCED_REGULAR = UINT8_C(0x07),
                HIGH_ACCURACY = UINT8_C(0x17)
            };

            enum struct REPZ : uint8_t
            {
                LOW_POWER = UINT8_C(0x02),
                REGULAR = UINT8_C(0x0E),
                ENHANCED_REGULAR = UINT8_C(0x1A),
                HIGH_ACCURACY = UINT8_C(0x52)
            };
        }

        /** @brief Allowed magnetometer range*/
        enum struct RANGE : int
        {
            uT0_3 = 0
        };

        /** @brief Allowed magnetometer power modes*/
        enum struct POWER_MODE : int
        {
            FORCE = 0x02, // Actually not specified in datasheet, shouldn't be employed
            SLEEP = 0x01,
            SUSPEND = 0x00
        };

        /** @brief Magnetometer sensitivity presets*/
        constexpr float SENSITIVITY[] = {
            0.3f};
    }

    /**
     * @brief Interface used for synchronising and using the embedded magnetometer same way as the other sensors.
     *
     */
    namespace MAGN_INTERFACE
    {
        /** @brief Allowed magnetometer interface power modes*/
        enum struct POWER_MODE : int8_t
        {
            NORMAL = 0b00011001,
            LOW_POWER = 0b00011010,
            SUSPEND = 0b00011000
        };

        /** @brief Allwed magnetometer ODRs. ODR settings may depend on perset mode (see Table 11 in datasheet) */
        enum struct ODR : uint8_t
        {
            Hz25_over_32 = UINT8_C(1),
            Hz25_over_16 = UINT8_C(2),
            Hz25_over_8 = UINT8_C(3),
            Hz25_over_4 = UINT8_C(4),
            Hz25_over_2 = UINT8_C(5),
            Hz25 = UINT8_C(6),
            Hz50 = UINT8_C(7),
            Hz100 = UINT8_C(8),
            Hz200 = UINT8_C(9),
            Hz400 = UINT8_C(10),
            Hz800 = UINT8_C(11)
        };

        /** @brief Selected values for reading  */
        enum struct DATA_SIZE : uint8_t
        {
            LSB_X = UINT8_C(0x00),
            X = UINT8_C(0x01),
            XYZ = UINT8_C(0x02),
            XYZ_RHALL = UINT8_C(0x03)
        };

        /** @brief Read offset after data ready, 2.5ms resolution. Recommended setting: 0x00<<2 */
        enum struct READ_OFFSET : uint8_t // Unused, minimum offset selected (0x00)
        {
            ms0 = UINT8_C(0x00 << 2),
            ms2_5 = UINT8_C(0x01 << 2),
            ms5 = UINT8_C(0x02 << 2),
            ms7_5 = UINT8_C(0x03 << 2),
            ms10 = UINT8_C(0x04 << 2),
            ms12_5 = UINT8_C(0x05 << 2),
            ms15 = UINT8_C(0x06 << 2),
            ms17_5 = UINT8_C(0x07 << 2),
            ms20 = UINT8_C(0x08 << 2),
            ms22_5 = UINT8_C(0x09 << 2),
            ms25 = UINT8_C(0x0A << 2),
            ms27_5 = UINT8_C(0x0B << 2),
            ms30 = UINT8_C(0x0C << 2),
            ms32_5 = UINT8_C(0x0D << 2),
            ms35 = UINT8_C(0x0E << 2),
            ms37_5 = UINT8_C(0x0F << 2),
        };

    }

    namespace TIMESTAMPS
    {
        /** @brief Timestamps sensitivity presets in seconds */
        constexpr float SENSITIVITY[] = {
            0.000039f};
    }

    namespace TEMP_SENSOR
    {
        /** @brief Temperature sensor sensitivity presets in ºC*/
        constexpr float SENSITIVITY[] = {
            1.0f / 512.0f};
    }

    /** @brief Single sensor measurement */
    typedef struct
    {
        float x;             ///< x-axis
        float y;             ///< y-axis
        float z;             ///< z-axis
        uint32_t sensortime; ///< Local sensor time, 24 bits with wrapping, 39us/LSB
    } DataPacket;

    /**
     * @brief Sensor API. All communication with sensor should happen throuh this library
     *
     */
    class BMX160
    {
    public:
        /**
         * @brief BMX60 error status. If different from ALL_OK handle error and clear this variable
         *
         */
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
         * @return bool success/fail status
         */
        bool begin();

        /**
         * @brief Set specific accelerometer data range
         *
         * @param range Desired range
         * @return bool success/fail status
         */
        bool setAccelRange(ACCEL::RANGE range);

        /**
         * @brief Set specific gyroscope data range
         *
         * @param range Desired range
         * @return bool success/fail status
         */
        bool setGyroRange(GYRO::RANGE range);

        /**
         * @brief Set the accelerometer power mode.
         *
         * @param power_mode
         * @return bool success/fail status
         */
        bool setAccelPowerMode(ACCEL::POWER_MODE power_mode);

        /**
         * @brief Set the gyroscope power mode.
         *
         * @param power_mode
         * @return bool success/fail status
         */
        bool setGyroPowerMode(GYRO::POWER_MODE power_mode);

        /**
         * @brief Set the magnetometer power mode. This must be run after soft reset for the magn to work properly
         *
         *
         * @param power_mode
         * @return bool success/fail status
         */
        bool setMagnInterfacePowerMode(MAGN_INTERFACE::POWER_MODE power_mode);

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
         * @brief Reads the sensor temperature data. Updated never (all sensors suspended) every 10ms +- (gyro enabled) or 1.28s lining with bit 15 of sensortime (gyro suspended / fast mode)
         *
         * @param temp Read temperature value
         * @return bool success/fail status
         */
        bool getTemp(float &temp);

        /**
         * @brief Returns true if IMU acknowledges conenction
         *
         * @return bool success/fail status
         */
        bool isConnected();

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
         * @brief Copies to odr the chosen sampling frequency for the accelerometer
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
         * @brief Copies to odr the chosen sampling frequency for the gyroscope
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
         * @brief Copies to odr the chosen sampling frequency for the magnetometer interface
         *
         * @param odr
         * @return bool success/fail status
         */
        bool getMagnInterfaceOdr(MAGN_INTERFACE::ODR &odr);

        bool getErrorRegister(uint8_t &error_code);

    protected:
        arduino::TwoWire &Wire = Wire;         ///< Communication object to employ
        const uint8_t address = UINT8_C(0x68); ///< Sensor address

        ACCEL::RANGE accelerometer_range = ACCEL::RANGE::G2; ///< Current accelerometer range
        GYRO::RANGE gyroscope_range = GYRO::RANGE::DPS2000;  ///< Current gyroscope range
        MAGN::RANGE magnetorquer_range = MAGN::RANGE::uT0_3; ///< Current magnetometer range
        float accelerometer_sensitivity = ACCEL::SENSITIVITY[0];
        float gyroscope_sensitivity = GYRO::SENSITIVITY[0];
        float magnetometer_sensitivity = MAGN::SENSITIVITY[0];
        // IF ALL 3 (not interface) == SUSPEND -> DO NOT:  ADD RULES AND CHECK IF TRUE
        // - burst write                    IMPLEMENTED (not supported)
        // - Write without a 0.4 ms wait    IMPLEMENETED
        // - burst read on FIFO_DATA        IMPLEMENTED
        // IF ALL 3 (not interface) == SUSPEND / LOW_POWER -> DO NOT:
        // - read the FIFO
        ACCEL::POWER_MODE accelerometer_power_mode = ACCEL::POWER_MODE::SUSPEND;
        GYRO::POWER_MODE gyroscope_power_mode = GYRO::POWER_MODE::SUSPEND;
        MAGN::POWER_MODE magnetometer_power_mode = MAGN::POWER_MODE::SUSPEND;
        MAGN_INTERFACE::POWER_MODE magnetometer_interface_power_mode = MAGN_INTERFACE::POWER_MODE::SUSPEND;
        ACCEL::ODR accelerometer_odr = ACCEL::ODR::Hz100;
        GYRO::ODR gyroscope_odr = GYRO::ODR::Hz100;
        MAGN_INTERFACE::ODR magnetometer_interface_odr = MAGN_INTERFACE::ODR::Hz100;
        MAGN_INTERFACE::DATA_SIZE magnetometer_interface_data_size = MAGN_INTERFACE::DATA_SIZE::XYZ_RHALL;
        /**
         * @brief Waiting function the library will employ. Can be overwritten with a derived class
         *
         */
        virtual void wait(unsigned long time);

        /**
         * @brief Sends a write command through the I2C protocol
         *
         * @param reg Reister to write to
         * @param byte 8-bit value to write
         * @return bool success/fail status
         */
        bool writeReg(const REGISTER reg, const uint8_t byte);

        /**
         * @brief Sends a burst of write commands through the I2C protocol.
         *
         * @param reg Arrays of registers to write to
         * @param buffer Arrays of 8-bit values to write to
         * @param length Length of buffer and regs arrays
         * @return bool success/fail status
         */
        bool writeReg(REGISTER const *const reg, uint8_t *const buffer, size_t length);

        /**
         * @brief Requests one byte through the I2C protocol
         *
         * @param reg Register to read from
         * @param buffer 8-bit buffer for storing read value
         * @return bool success/fail status
         */
        bool readReg(const REGISTER reg, uint8_t &buffer);

        /**
         * @brief Requests a variable number of bytes through the I2C protocol
         *
         * @param reg Register to read from
         * @param buffer Pointer to 8-bit buffer of length length
         * @param length Length of buffer buffer
         * @return bool success/fail status
         */
        bool readReg(const REGISTER reg, uint8_t *const buffer, size_t length);

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

}

#endif