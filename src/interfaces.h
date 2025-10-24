#include <stdint.h>
#include <Arduino.h>
/**
 * @brief Timing interface. Can be used to control how FineTuneBMX160 handles timing.
 * 
 */
class TimingInterface{
    public:
    /**
     * @brief function must return after time_ms has elapsed.
     * 
     * @param time_ms 
     */
    virtual void wait(uint32_t time_ms) = 0;
};

/**
 * @brief Specific implementation of @ref TimingInterface that uses the blocking delay. Used as default implementation.
 * 
 */
class ArduinoBlockingTiming : public TimingInterface {
    public:
    /**
     * @brief Arduino blocking delay function.
     * 
     * @param time_ms 
     */
    void wait(uint32_t time_ms) override {
        delay(time_ms);
    }
};


/**
 * @brief Communication protocol iterface.  Can be used to control what protocol BMX160 uses. For consistency, provide required parameters in constructor and start the communication in begin(). Unsuccessful communication should store the error, and after getLastError() it should be cleared out.
 * 
 */
class CommunicationInterface {
    public:

    /**
     * @brief The communication protocol is initialized here. For example, wire.begin() for arduino's I2C.
     * 
     * @return bool success/fail status 
     */
    virtual bool begin() = 0;


    /**
     * @brief Writes an array of registers @p reg and writes @p buffer content of size @p size to them. Returns true if succesful. If failed, stores the error locally and returns false.
     * 
     * @param reg First register address to write to
     * @param buffer Buffer with data to write 
     * @param size Size of @p buffer and number of registers to write
     * @return bool success/fail status 
     */
    virtual bool writeReg(uint8_t const * const reg, uint8_t const * const buffer, const uint8_t  size) = 0;

    /**
     * @brief Reads a total of @p size registers starting from @p reg into @p buffer. Returns true if succesful. If failed, stores the error locally and returns false.
     * 
     * @param reg First register address to read from
     * @param buffer Buffer to store register data
     * @param size Size of @p buffer and number of registers to read
     * @return bool success/fail status 
     */
    virtual bool readReg(const uint8_t reg, uint8_t * const buffer, const uint8_t  size) = 0;

    /**
     * @brief Returns the last error (or lack_of, 0) from the interface implementation. Each implementation can define its own error enum to represet the error code. Once this funciton is called, the error is removed unless it is not fixed.
     * 
     * @return int Error code. 
     */
    virtual int getLastError() = 0;

    // Optional:

    /**
     * @brief Equivalent to @ref CommunicationInterface::writeReg(const uint8_t, uint8_t* const, const uint8_t)
     * 
     * @param reg Register address to write to
     * @param buffer Buffer with data to write 
     * @return bool success/fail status 
     */
    virtual bool writeReg(const uint8_t reg, uint8_t& buffer) {
        return writeReg(&reg,&buffer,1);
    }

    /**
     * @brief Equivalent to @ref CommunicationInterface::readReg(const uint8_t, uint8_t* const, const uint8_t)
     * 
     * @param reg Register address to read from
     * @param buffer Buffer to store register data
     * @return bool success/fail status 
     */
    virtual bool readReg(const uint8_t reg, uint8_t& buffer){
        return readReg(reg,&buffer,1);
    }
};


