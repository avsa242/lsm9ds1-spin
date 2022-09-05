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
* 1 extra core/cog for the PASM I2C or SPI engine (none, if bytecode engine is used)
* sensor.imu.common.spinh (provided by spin-standard-library)
* sensor.temp.common.spinh (provided by spin-standard-library)

P2/SPIN2:
* p2-spin-standard-library
* sensor.imu.common.spin2h (provided by p2-spin-standard-library)
* sensor.temp.common.spin2h (provided by p2-spin-standard-library)

## Compiler Compatibility

| Processor | Language | Compiler               | Backend     | Status                |
|-----------|----------|------------------------|-------------|-----------------------|
| P1        | SPIN1    | FlexSpin (5.9.14-beta) | Bytecode    | OK                    |
| P1        | SPIN1    | FlexSpin (5.9.14-beta) | Native code | OK                    |
| P1        | SPIN1    | OpenSpin (1.00.81)     | Bytecode    | Untested (deprecated) |
| P2        | SPIN2    | FlexSpin (5.9.14-beta) | NuCode      | FTBFS                 |
| P2        | SPIN2    | FlexSpin (5.9.14-beta) | Native code | OK                    |
| P1        | SPIN1    | Brad's Spin Tool (any) | Bytecode    | Unsupported           |
| P1, P2    | SPIN1, 2 | Propeller Tool (any)   | Bytecode    | Unsupported           |
| P1, P2    | SPIN1, 2 | PNut (any)             | Bytecode    | Unsupported           |

## Limitations

* Alternate I2C address bits of Accel/Gyro module and Mag module are currently linked (i.e., if address bit 1 is set on driver startup, both modules use that alternate address bit)


