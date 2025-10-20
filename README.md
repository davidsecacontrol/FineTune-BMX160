# FineTune BMX160 Sensor Library

A statically allocated STL-free driver for interfacing with the [BMX160 9-axis sensor](https://www.mouser.com/pdfdocs/BST-BMX160-DS000-11.pdf?srsltid=AfmBOorUoEEuLLeUnl63Qi5JP0psxd0zj9lsFmA3bqbn5dXef2W4PHS2).

## Usage
This library is plug-and-play. First, declare the IMU:
```
FineTuneBMX160::BMX160 bmx160;
```
And then initialize it:
```
Wire.begin();
bmx160.begin();
```
Sensor data can now be read:
```
FineTuneBMX160::DataPacket accel, gyro, magn;
bmx160.getAllData(accel, gyro, magn);
```

For other functionalities, check the library examples.

## Features
- Read time stamped **accelerometer**, **gyroscope**, **magnetometer** and **temperature** data
- Configure all **measurement ranges**
- Configure all **power modes**
- Configure all **ODRs**. Notice certain power modes don't allow all ODRs, check datasheet.
- **Error handling** by all functions returning success/failure + IMU state variable 
- Indirect addressing of the inner [BMM150 magnetometer sensor](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm150-ds001.pdf). Notice the interface must be manually switched from Setup / Data mode if this feature is directly used.

# Unimplemented
- Undersampling
- FIFO buffer
- Interrupts
- Device self-tests
- Offest compensation
- Certain low-power configurations




