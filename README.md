# FineTune BMX160 Sensor Library

A statically allocated library for interfacing with the [BMX160 9-axis sensor](https://www.mouser.com/pdfdocs/BST-BMX160-DS000-11.pdf?srsltid=AfmBOorUoEEuLLeUnl63Qi5JP0psxd0zj9lsFmA3bqbn5dXef2W4PHS2) intended for 3D orientation algorithms. 
Currently under active development.

## Usage
For a basic usage of the library, check the example [GetAllData.ino](examples/GetAllData.ino)

## Features
- Read time stamped **accelerometer**, **gyroscope**, **magnetometer** and **temperature** data
- Configure all **measurement ranges**
- Configure all **power modes**
- Configure all **ODRs**. Notice certain power modes don't allow all ODRs, check datasheet.
- **Error handling** by all functions returning success/failure + IMU state variable 
- Indirect addressing of the inner [BMM150 magnetometer sensor](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm150-ds001.pdf). Notice the interface must be manually switched from Setup / Data mode if this feature is directly used. 
- Advanced features (undersampling, digital filter FIFO, etc.) coming soon

  ## Roadmap
  For an updated development plan, check [this ClickUp board](https://sharing.clickup.com/90151493180/b/h/5-90158959475-2/683fd04fea24ccc). Current state and better tracking comming soon.



