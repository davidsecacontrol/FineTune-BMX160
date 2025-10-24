#include "FineTuneBMX160.h"

using namespace FineTuneBMX160;

// NOT RECOMMENDED: This definiton derives a class that promotes certain methods to public. Intended for those who wish to expand the library / interface directly with the IMU.
// Used here for forcing an error
class public_imu: public BMX160{ 
    public:
        using BMX160::writeReg;
};
public_imu bmx160;
// ---------------------------------------

void setup()
{
    Serial.begin(115200);
    while (!Serial);


    // All library functions return a bool for success / failure:

    // Optional: Check connection and chip ID --------------------------------------------------------------
    while(!bmx160.isConnected()){
        Serial.print("IMU not detected.");Serial.print("I2C error: ");Serial.println((int)bmx160.state);
        delay(5000);
    }

    uint8_t chip_id;
    bmx160.getChipID(chip_id);
    Serial.print("Chip ID: ");Serial.println(chip_id);
    // ---------------------------------------------------------------------------------------------------

    if(!bmx160.begin()){ // Error spotted - for I2C errors or mistakes caught by the library
        // Recover from error
    }

    // This setter will fail:
    if(!bmx160.setMagnInterfaceOdr(MAGN_INTERFACE::ODR::Hz25_over_4)){
        Serial.print("Current/expected error (caught by library): ");Serial.print((int)bmx160.state);
        Serial.print("/");Serial.println((int)ERROR_CODE::INVALID_ODR_SETTING); 
    }
    
    // Some setters have very specific edge-cases that may not be defined. The setter will still return false, but the eror code will be ERROR_CODE::ERR_REG. This references to the IMU's internal error register:
    bmx160.writeReg(REGISTER::GYR_CONF,0); // This results in an error code
    uint8_t error_code;
    bmx160.getErrorRegister(error_code); 
    Serial.print("BMX160 ERR_REG (forced error code): 0b");Serial.println(error_code,BIN);

    // If there is a communication error, it can be obtained from getCommit will be reflected by ERROR_CODE::COMMUNICATION_INTERFACE_ERROR
    Serial.println("Triggering communication error...");
    // Forcing such an error by reassigning the commsImplementation with an incorrect address:
    ArduinoI2CCommunication comms_wrong(0x00);
    bmx160.setCommunicationInterface(comms_wrong);
    bmx160.isConnected();
    int comms_error;
    bmx160.getCommunicationInterfaceError(comms_error);
    Serial.print("BMX160 Error code: ");Serial.print((uint8_t)bmx160.state);Serial.print("/");Serial.println((uint8_t)ERROR_CODE::COMMUNICATION_INTERFACE_ERROR);
    Serial.print("Comms error code: ");Serial.print(comms_error);Serial.print("/");Serial.println((uint8_t)ArduinoI2CCommunication::ERROR_CODE::I2C_NACK_ON_ADDRESS);
}

void loop()
{
    while(true);
}
