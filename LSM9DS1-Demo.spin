{
----------------------------------------------------------------------------------------------------
    Filename:       LSM9DS1-Demo.spin
    Description:    Demo of the LSM9DS1 driver
        * 9DoF data output
    Author:         Jesse Burt
    Started:        Aug 12, 2017
    Updated:        Jan 19, 2025
    Copyright (c) 2025 - See end of file for terms of use.
----------------------------------------------------------------------------------------------------
}

' Uncomment the two lines below to use the driver in SPI mode
'#define LSM9DS1_SPI
'#pragma exportdef(LSM9DS1_SPI)

' Uncomment the two lines below to use the driver with a bytecode-based SPI engine
'#define LSM9DS1_SPI_BC
'#pragma exportdef(LSM9DS1_SPI_BC)

' Uncomment these two use the driver with a bytecode-based I2C engine
'#define LSM9DS1_I2C_BC
'#pragma exportdef(LSM9DS1_I2C_BC)


CON

    _clkmode    = xtal1+pll16x
    _xinfreq    = 5_000_000


OBJ

    time:   "time"
    ser:    "com.serial.terminal.ansi" | SER_BAUD=115_200

    { to use 3-wire SPI, set MOSI and MISO to the same pin }
    sensor: "sensor.imu.9dof.lsm9ds1" | {I2C}SCL=28, SDA=29, I2C_FREQ=400_000, I2C_ADDR=0, ...
                                        {SPI}CS_AG=0, CS_M=1, SCK=2, MOSI=3, MISO=4


PUB main()

    setup()

    ' preset settings: enable measurements, set output data rates
    sensor.preset_active()

    repeat
        ser.pos_xy(0, 3)
        show_accel_data()
        show_gyro_data()
        show_mag_data()
        if ( ser.getchar_noblock() == "c" )
        ' press 'c' during the demo to "calibrate," or eliminate the DC offset the sensor data
        '   may have
            cal_accel()
            cal_gyro()
            cal_mag()



PUB cal_accel()
' Calibrate the accelerometer
    ser.pos_xy(0, 3)
    ser.str(@"Calibrating accelerometer...")
    ser.clear_ln()
    sensor.calibrate_accel()


PUB cal_gyro()
' Calibrate the gyroscope
    ser.pos_xy(0, 4)
    ser.str(@"Calibrating gyroscope...")
    ser.clear_ln()
    sensor.calibrate_gyro()


PUB cal_mag()
' Calibrate the magnetometer
    ser.pos_xy(0, 5)
    ser.str(@"Calibrating magnetometer...")
    ser.clear_ln()
    sensor.calibrate_mag()


PUB show_accel_data() | axis, a[3]
' Display accelerometer data
    repeat                                      ' wait for measurement to be ready
    until sensor.accel_data_rdy()

    longfill(@a, 0, 3)                          ' init array to 0
    sensor.accel_g(@a[sensor.X_AXIS], @a[sensor.Y_AXIS], @a[sensor.Z_AXIS])

    ser.str(@"Accel (g):  ")
    repeat axis from sensor.X_AXIS to sensor.Z_AXIS
        show_mill_as_decimal(a[axis])
    ser.newline()


PUB show_gyro_data() | axis, g[3]
' Display gyroscope data
    repeat
    until sensor.gyro_data_rdy()

    longfill(@g, 0, 3)
    sensor.gyro_dps(@g[sensor.X_AXIS], @g[sensor.Y_AXIS], @g[sensor.Z_AXIS])

    ser.str(@"Gyro (dps): ")
    repeat axis from sensor.X_AXIS to sensor.Z_AXIS
        show_mill_as_decimal(g[axis])
    ser.newline()


PUB show_mag_data() | axis, m[3]
' Display magnetometer data
    repeat
    until sensor.mag_data_rdy()

    longfill(@m, 0, 3)
    sensor.mag_gauss(@m[sensor.X_AXIS], @m[sensor.Y_AXIS], @m[sensor.Z_AXIS])

    ser.str(@"Mag (Gs):  ")
    repeat axis from sensor.X_AXIS to sensor.Z_AXIS
        show_mill_as_decimal(m[axis])
    ser.newline()


PUB show_mill_as_decimal(v)
' Show a value in millionths as a fractional decimal number
'   (e.g., 1_234_567 would be shown as "1.234567")
'   v: value in millionths
    ser.printf(@"%c%d.%06.6d     ", (v < 0) ? "-" : " ", ...    ' sign
                                    ||(v / 1_000_000), ...      ' whole
                                    ||(v // 1_000_000) )        ' fractional


PUB setup()

    ser.start()
    time.msleep(30)
    ser.clear()
    ser.strln(@"Serial terminal started")

    if ( sensor.start() )
        ser.strln(@"LSM9DS1 driver started")
    else
        ser.strln(@"LSM9DS1 driver failed to start - halting")
        repeat


DAT
{
Copyright 2025 Jesse Burt

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
}

