#ifndef FINETUNE_BMX160
#define FINETUNE_BMX160

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <stdint.h>

namespace FineTune
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

    class BMX160
    {
    public:
        // Initializers
        BMX160() = default;
        BMX160(arduino::TwoWire &Wire, uint8_t address = UINT8_C(0x68));

        /**
         * @brief Sends a write command through the I2C protocol
         *
         * @param reg Reister to write to
         * @param byte 8-bit value to write
         * @return uint8_t FineTune::I2CStatus to identify failed transmission
         */
        [[nodiscard]] I2C_STATUS writeReg(const uint8_t reg, const uint8_t byte);

        /**
         * @brief Requests a single byte through the I2C protocol
         *
         * @param reg Register to read from
         * @param buffer 8-bit variable to write to
         * @return uint8_t FineTune::I2CStatus to identify failed transmission
         */
        [[nodiscard]] I2C_STATUS readReg(const uint8_t reg, uint8_t &buffer);

        /**
         * @brief Returns true if IMU acknowledges conenction
         *
         * @return uint8_t FineTune::I2CStatus to identify failed transmission
         */
        [[nodiscard]] I2C_STATUS isConnected();

        // protected:
        arduino::TwoWire &Wire = Wire;
        const uint8_t address = UINT8_C(0x68);
    };

}

#endif