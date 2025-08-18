#ifndef FINETUNE_BMX160
#define FINETUNE_BMX160

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <stdint.h>
namespace FineTuneBMX160
{
    enum struct REGISTER: uint8_t{
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
        OFFSET_0 =  UINT8_C(0x71),
        OFFSET_1 =  UINT8_C(0x72),
        OFFSET_2 =  UINT8_C(0x73),
        OFFSET_3 =  UINT8_C(0x74),
        OFFSET_4 =  UINT8_C(0x75),
        OFFSET_5 =  UINT8_C(0x76),
        OFFSET_6 =  UINT8_C(0x77),
        STEP_CNT_0 = UINT8_C(0x78),
        STEP_CNT_1 = UINT8_C(0x79),
        STEP_CONF_0 = UINT8_C(0x7A),
        STEP_CONF_1 = UINT8_C(0x7B),
        CMD = UINT8_C(0x7E)
    };

    enum struct I2C_STATUS : uint8_t
    {
        SUCCESS = UINT8_C(0),
        ERR_TOO_LONG_FOR_BUFFER = UINT8_C(1),
        ERR_NACK_ON_ADDRESS = UINT8_C(2),
        ERR_NACK_ON_DATA_TRANSMISSION = UINT8_C(3),
        ERR_OTHER = UINT8_C(4),
        ERR_TIMEOUT = UINT8_C(5)
    };

    namespace RANGE
    {
        enum struct ACCEL : int
        {
            G2 = 0,
            G4 = 1,
            G8 = 2,
            G16= 3
        };

        enum struct GYRO : int
        {
            DPS2000 = 0,
            DPS1000 = 1,
            DPS500 = 2,
            DPS250 = 3,
            DPS150 = 4
        };
        enum struct MAGN : int
        {
            uT0_3 = 0
        };
    }

    typedef struct
    {
        float x;
        float y;
        float z;
    } DataPacket;

    class BMX160
    {
    public:
        // Initializers
        BMX160() = default;
        BMX160(arduino::TwoWire &Wire, uint8_t address = UINT8_C(0x68));

        [[nodiscard]] I2C_STATUS begin();

        [[nodiscard]] I2C_STATUS setAccelRange(RANGE::ACCEL range);
        [[nodiscard]] I2C_STATUS  setGyroRange(RANGE::GYRO range);

        [[nodiscard]] I2C_STATUS getAllData(DataPacket &accel, DataPacket &gyro, DataPacket &magn);
        /**
         * @brief Sends a write command through the I2C protocol
         *
         * @param reg Reister to write to
         * @param byte 8-bit value to write
         * @return uint8_t FineTuneBMX160::I2CStatus to identify failed transmission
         */
        [[nodiscard]] I2C_STATUS writeReg(const REGISTER reg, const uint8_t byte);

        /**
         * @brief Requests one byte through the I2C protocol
         *
         * @param reg Register to read from
         * @param buffer 8-bit buffer for storing read value
         * @return I2C_STATUS
         */
        [[nodiscard]] I2C_STATUS readReg(const REGISTER reg, uint8_t &buffer);

        /**
         * @brief Requests a variable number of bytes through the I2C protocol
         *
         * @param reg Register to read from
         * @param buffer Pointer to 8-bit buffer of length length
         * @param length Length of buffer buffer
         * @return uint8_t FineTuneBMX160::I2CStatus to identify failed transmission
         */
        [[nodiscard]] I2C_STATUS readReg(const REGISTER reg, uint8_t *const buffer, size_t length);

        /**
         * @brief Returns true if IMU acknowledges conenction
         *
         * @return uint8_t FineTuneBMX160::I2CStatus to identify failed transmission
         */
        [[nodiscard]] I2C_STATUS isConnected();

    protected:
        arduino::TwoWire &Wire = Wire;
        const uint8_t address = UINT8_C(0x68);

        // Default values after reset
        RANGE::ACCEL accelerometer_range = RANGE::ACCEL::G2;
        RANGE::GYRO gyroscope_range = RANGE::GYRO::DPS2000;
        RANGE::MAGN magnetorquer_range = RANGE::MAGN::uT0_3;
    };

}

#endif