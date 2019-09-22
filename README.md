# lsm9ds1-spin
--------------

This is a P8X32A/Propeller driver object for the STMicroelectronics LSM9DS1 9-DoF IMU.

## Salient Features

* 3-Axis Accelerometer in raw or calculated output (milli-G's)
* 3-Axis Gyroscope in raw or calculated output (milli-degrees per second)
* 3-Axis Magnetometer in raw or calculated output (milli-Gauss)
* Temperature in raw or calculated output (milli-degrees C/F)

## Requirements

* 1 additional core/cog for the PASM SPI driver

## Limitations

* Very early in development - may malfunction, or outright fail to build
* Some data may be imprecise because of fixed-point math implementation (unverified)
* Interrupt lines not currently supported

## TODO

- [ ] Code cleanup
- [ ] Implement 3W and 4W SPI variants
- [ ] Implement I2C driver variant
