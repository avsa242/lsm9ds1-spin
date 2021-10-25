# lsm9ds1-spin
--------------

This is a P8X32A/Propeller, P2X8C4M64P/Propeller 2 driver object for the STMicroelectronics LSM9DS1 9-DoF IMU.

**IMPORTANT**: This software is meant to be used with the [spin-standard-library](https://github.com/avsa242/spin-standard-library) (P8X32A) or [p2-spin-standard-library](https://github.com/avsa242/p2-spin-standard-library) (P2X8C4M64P). Please install the applicable library first before attempting to use this code, otherwise you will be missing several files required to build the project.

## Salient Features

* 3-Wire or 4-wire SPI connection at 1MHz (P1), up to 10MHz (P2)
* I2C connection at up to 400kHz
* Accelerometer data in raw or calculated output (micro-G's)
* Gyroscope data in raw or calculated output (micro-degrees per second)
* Magnetometer data in raw or calculated output (micro-Gauss)
* Temperature data in calculated output (centi-degrees C)
* Common functions for all three sensors: Full-scale, Output data rate, Per-axis Output enable, Flag indicating new data available, perform on-chip calibration, or write manually derived values, data endianness, soft-reset, read interrupts state
* Additional Accelerometer functionality: Set high-resolution mode
* Additional Gyroscope functionality: Set data output high-pass filter cut-off freq, set inactivity: duration, threshold, sleep on/off, low-power mode, sleep mode
* Additional Magnetometer functionality: Built-in self-test, performance mode, flag indicating measurement overflow/saturation

## Requirements

P1/SPIN1:
* spin-standard-library
* P1/SPIN1: 1 additional core/cog for the PASM I2C or SPI engine, as applicable

P2/SPIN2:
* p2-spin-standard-library

## Compiler compatibility

* P1/SPIN1: OpenSpin (tested with 1.00.81)
* P2/SPIN2: FlexSpin (tested with 5.9.3-beta)
* ~~BST~~ (incompatible - no preprocessor)
* ~~Propeller Tool~~ (incompatible - no preprocessor)
* ~~PNut~~ (incompatible - no preprocessor)

## Limitations

* TBD

