#ifndef FINETUNE_BMX160
#define FINETUNE_BMX160

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <stdint.h>

class FTBMX160
{
public:
    FTBMX160() = default;
    FTBMX160(arduino::TwoWire& Wire, uint8_t address = UINT8_C(0x68));

    /**
     * @brief Sends a write command through the I2C protocol
     * 
     * @param reg Reister to write to
     * @param byte 8-bit value to write
     * @return uint8_t Status code of Wire.endTrasmission()
     */
    uint8_t writeReg(const uint8_t reg, const uint8_t byte);

    /**
     * @brief Requests a single byte through the I2C protocol
     * 
     * @param reg Register to read from
     * @return uint8_t read byte
     */
    uint8_t readReg(const uint8_t reg);

    /**
     * @brief Returns true if IMU acknowledges conenction
     * 
     * @return uint8_t Status code of Wire.endTrasmission()
     */
    uint8_t isConnected();

//protected:
    arduino::TwoWire& Wire = Wire;
    const uint8_t address= UINT8_C(0x68);
};



#endif