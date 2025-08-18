#include <Arduino.h>
#include "FineTuneBMX160.h"

FineTuneBMX160::BMX160 bmx160(Wire, UINT8_C(0x68));

[[maybe_unused]] FineTuneBMX160::I2C_STATUS _; // For constiously dropping error checking

void setup()
{
  Serial.begin(9600);
  while (!Serial)
  {
    delay(50);
  };

  Wire.begin();
  delay(1000);
  Wire.setClock(1000000); // Optional, sets I2C to maximum BMX160 can handle

  // Check connection (optional) ---------------------------------
  FineTuneBMX160::I2C_STATUS status = bmx160.isConnected();
  if (status == FineTuneBMX160::I2C_STATUS::SUCCESS)
  {
    Serial.println("IMU found");
  }
  else
  {
    Serial.print("IMU not found. Wire error code:");
    Serial.print(static_cast<uint8_t>(status));
    Serial.print("    blocking...");
    while (true);
  }

  // Initialize library and turn on accel & gyro -----------------
  _ = bmx160.begin();

  // Show chip id (optional) ------------------------------------------------
  uint8_t chip_id = UINT8_C(8);
  _ = bmx160.readReg(FineTuneBMX160::REGISTER::CHIP_ID, chip_id);
  Serial.print("Chip id: ");Serial.println(chip_id);


  // Select desired ranges (optional) ---------------------------------------
  _ = bmx160.setAccelRange(FineTuneBMX160::RANGE::ACCEL::G4);
  _ = bmx160.setGyroRange(FineTuneBMX160::RANGE::GYRO::DPS1000);


  
}

void loop()
{


  FineTuneBMX160::DataPacket accel, gyro, magn;

  // Read all data (magn not implemented yet) -------------------------------
  _ = bmx160.getAllData(accel, gyro, magn);


  Serial.print("Accel [ ");Serial.print(accel.x);Serial.print(" , ");Serial.print(accel.y);
  Serial.print(" , ");Serial.print(accel.z);Serial.println(" ] m^2/s ");

  Serial.print("Gyro  [ ");Serial.print(gyro.x);Serial.print(" , ");Serial.print(gyro.y);
  Serial.print(" , ");Serial.print(gyro.z);Serial.println(" ] deg/s ");

  Serial.print("Magn  [ ");Serial.print(magn.x);Serial.print(" , ");Serial.print(magn.y);
  Serial.print(" , ");Serial.print(magn.z);Serial.println(" ] uT ");

  Serial.println("-------------------------------------");


  delay(1000);

}
