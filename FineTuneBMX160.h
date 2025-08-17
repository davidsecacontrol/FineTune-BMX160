#ifndef FINETUNE_BMX160
#define FINETUNE_BMX160

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <stdint.h>
namespace FineTuneBMX160
{
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
        [[nodiscard]] I2C_STATUS writeReg(const uint8_t reg, const uint8_t byte);

        /**
         * @brief Requests one byte through the I2C protocol
         *
         * @param reg Register to read from
         * @param buffer 8-bit buffer for storing read value
         * @return I2C_STATUS
         */
        [[nodiscard]] I2C_STATUS readReg(const uint8_t reg, uint8_t &buffer);

        /**
         * @brief Requests a variable number of bytes through the I2C protocol
         *
         * @param reg Register to read from
         * @param buffer Pointer to 8-bit buffer of length length
         * @param length Length of buffer buffer
         * @return uint8_t FineTuneBMX160::I2CStatus to identify failed transmission
         */
        [[nodiscard]] I2C_STATUS readReg(const uint8_t reg, uint8_t *const buffer, int length);

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