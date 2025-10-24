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
     * @brief Returns whether the sensor is detected.
     * 
     * @return bool true/false for detected/not detected
     */
    virtual bool isConnected() = 0;


    /**
     * @brief Returns the last error (or lack_of, 0) from the interface implementation. Each implementation can define its own error enum to represet the error code. Once this funciton is called, the error is removed unless it is not fixed.
     * 
     * @return int Error code. 
     */
    virtual int getLastError() = 0;

    // Optional:

    /**
     * @brief Writes to a register address @p reg and writes @p buffer content to them. Returns true if succesful. If failed, stores the error locally and returns false.
     * 
     * @param reg Register address to write to
     * @param buffer Buffer with data to write 
     * @return bool success/fail status 
     */
    virtual bool writeReg(const uint8_t reg, const uint8_t& buffer)  = 0;

    /**
     * @brief Reads the contents of register @p reg and copies them into @p buffer. Returns true if succesful. If failed, stores the error locally and returns false.
     * 
     * @param reg Register address to read from
     * @param buffer Buffer to store register data
     * @return bool success/fail status 
     */
    virtual bool readReg(const uint8_t reg, uint8_t& buffer) = 0;
};


class ArduinoI2CCommunication : public CommunicationInterface {
    uint8_t address;

    enum struct ERROR_CODE :uint8_t
    {
        ALL_OK = UINT8_C(0),
        I2C_TOO_LONG_FOR_BUFFER = UINT8_C(1),
        I2C_NACK_ON_ADDRESS = UINT8_C(2),
        I2C_NACK_ON_DATA_TRANSMISSION = UINT8_C(3),
        I2C_OTHER = UINT8_C(4),
        I2C_TIMEOUT = UINT8_C(5)
    };
    ERROR_CODE error_code = ERROR_CODE::ALL_OK;

    ArduinoI2CCommunication(uint8_t address) : address(address) {};

    ~ArduinoI2CCommunication() {
        Wire.end();
    }

    bool begin() override {
        Wire.begin();
    }

    bool isConnected() override {
        Wire.beginTransmission(this->address);
        this->error_code = static_cast<ERROR_CODE>(Wire.endTransmission());
        return this->error_code == ERROR_CODE::ALL_OK;
    }

    int getLastError() override {
        return static_cast<int>(this->error_code);
    }

    bool writeReg(const uint8_t reg, const uint8_t& buffer) override {

        Wire.beginTransmission(this->address);
        Wire.write(reg);
        Wire.write(&buffer, 1);
        this->error_code = static_cast<ERROR_CODE>(Wire.endTransmission());

        return this->error_code == ERROR_CODE::ALL_OK;
    }

    bool readReg(const uint8_t reg, uint8_t& buffer) override {
        // Send register to read
        Wire.beginTransmission(this->address);
        Wire.write(&reg, 1);
        this->error_code = static_cast<ERROR_CODE>(Wire.endTransmission(false)); // Error to be addressed, "false" to not release bus

        if(this->error_code != ERROR_CODE::ALL_OK){
            return false;
        }

        // Read the result
        Wire.requestFrom(this->address, 1);
        buffer = Wire.read();
        this->error_code = static_cast<ERROR_CODE>(Wire.endTransmission());

        return this->error_code == ERROR_CODE::ALL_OK;
    }

};