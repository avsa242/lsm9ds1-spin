# lsm9ds1-spin
--------------

This is a P8X32A/Propeller, P2X8C4M64P/Propeller 2 driver object for the STMicroelectronics LSM9DS1 9-DoF IMU.

## Salient Features

* 3-Wire SPI connection at up to 1MHz (P1), _TBD_ (P2)
* Accelerometer data in raw or calculated output (micro-G's)
* Gyroscope data in raw or calculated output (micro-degrees per second)
* Magnetometer data in raw or calculated output (micro-Gauss)
* Temperature data in calculated output (centi-degrees C)
* Common functions for all three sensors: Full-scale, Output data rate, Per-axis Output enable, Flag indicating new data available, perform on-chip calibration, or write manually derived values, data endianness, soft-reset, read interrupts state
* Additional Accelerometer functionality: Set high-resolution mode
* Additional Gyroscope functionality: Set data output high-pass filter cut-off freq, set inactivity: duration, threshold, sleep on/off, low-power mode, sleep mode
* Additional Magnetometer functionality: Built-in self-test, performance mode, flag indicating measurement overflow/saturation

## Requirements

* P1/SPIN1: 1 additional core/cog for the PASM SPI driver
* P2/SPIN2: N/A

## Compiler compatibility

* P1/SPIN1: OpenSpin (tested with 1.00.81)
* P2/SPIN2: FastSpin (tested with 4.1.0-beta)

## Limitations

* Very early in development - may malfunction, or outright fail to build
* Magnetometer data low-confidence (I have no way to verify this type of sensor, currently)
* Interrupt I/O pins not currently used in the driver (interrupt status can be read through API, however)

## TODO

- [ ] Indicate default values in methods
- [ ] Modify some methods to take scaled parameters, rather than raw register values
- [ ] Add ability to set temperature scale
- [ ] Rename methods to the sensor.imu API - WIP
- [x] Implement 3W SPI driver
- [ ] Implement 4W SPI driver
- [ ] Implement I2C driver
