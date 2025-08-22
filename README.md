# FineTune BMX160 Sensor Library

A statically allocated library for interfacing with the [BMX160 9-axis sensor](https://www.mouser.com/pdfdocs/BST-BMX160-DS000-11.pdf?srsltid=AfmBOorUoEEuLLeUnl63Qi5JP0psxd0zj9lsFmA3bqbn5dXef2W4PHS2) intended for 3D orientation algorithms. 
Currently under active development.

## Usage
For a basic usage of the library, check the example [GetAllData.ino](examples/GetAllData.ino)

## Features
- Read time stamped **accelerometer**, **gyroscope** and **temperature** data
- Configure **measurement ranges**
- Configure **power modes**
- **Error handling** by all functions returning success/failure
- Magnetometer and advanced features (undersampling, digital filter FIFO, etc.) coming soon

  ## Roadmap
  For an updated development plan, check [this ClickUp board](https://sharing.clickup.com/90151493180/b/h/5-90158959475-2/683fd04fea24ccc)



