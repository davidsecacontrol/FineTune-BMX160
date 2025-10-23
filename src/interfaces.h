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



class ComunicationInterface {

};