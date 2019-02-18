{
    --------------------------------------------
    Filename: LSM9DS1-Test.spin
    Author: Jesse Burt
    Description: Test harness for LSM9DS1 driver
    Copyright (c) 2019
    Started Aug 12, 2017
    Updated Feb 18, 2019
    See end of file for terms of use.
    --------------------------------------------
}

CON

    _clkmode    = cfg#_clkmode
    _xinfreq    = cfg#_xinfreq

    SCL_PIN     = 5
    SDIO_PIN    = 6
    CS_AG_PIN   = 7
    CS_M_PIN    = 8
    INT_AG_PIN  = 9
    INT_M_PIN   = 10

    COL_REG     = 0
    COL_SET     = 12
    COL_READ    = 24
    COL_PF      = 40
    DEBUG_LED   = cfg#LED1

OBJ

    cfg     : "core.con.boardcfg.flip"
    ser     : "com.serial.terminal"
    time    : "time"
    imu     : "sensor.imu.tri.lsm9ds1"
    int     : "string.integer"

VAR

    byte _ser_cog, _imu_cog
    byte _max_cols
    byte _test_row

PUB Main

    Setup
    ser.Clear

    imu.CalibrateAG
    repeat
        ser.Position (0, 0)
        AccelRaw
        ser.Position (0, 1)
        GyroRaw
        time.MSleep (100)

PUB AccelRaw | ax, ay, az

    imu.ReadAccel (@ax, @ay, @az)
    ax := (ax * 1000) / 16384
    ay := (ay * 1000) / 16384
    az := (az * 1000) / 16384
    ser.Str (int.DecPadded (ax, 7))
    ser.Str (int.DecPadded (ay, 7))
    ser.Str (int.DecPadded (az, 7))

PUB GyroRaw | gx, gy, gz

    imu.ReadGyro (@gx, @gy, @gz)
    ser.Str (int.DecPadded (gx, 7))
    ser.Str (int.DecPadded (gy, 7))
    ser.Str (int.DecPadded (gz, 7))

PUB Setup

    repeat until _ser_cog := ser.Start (115_200)
    ser.Clear
    ser.Str (string("Serial terminal started", ser#NL))

    if _imu_cog := imu.Start (SCL_PIN, SDIO_PIN, CS_AG_PIN, CS_M_PIN, INT_AG_PIN, INT_M_PIN)
        if imu.whoAmI == imu#WHO_AM_I
            ser.Str (string("LSM9DS1 driver started", ser#NL))
            _max_cols := 4
            waitkey
            return
    ser.Str (string("Unable to start LSM9DS1 driver", ser#NL))
    imu.Stop
    time.MSleep (1)
    ser.Stop
    repeat

PUB waitkey

    ser.Str (string("Press any key", ser#NL))
    ser.CharIn

PUB flash

    dira[DEBUG_LED] := 1
    repeat
        !outa[DEBUG_LED]
        time.MSleep (100)

DAT
{
    --------------------------------------------------------------------------------------------------------
    TERMS OF USE: MIT License

    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
    associated documentation files (the "Software"), to deal in the Software without restriction, including
    without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
    following conditions:

    The above copyright notice and this permission notice shall be included in all copies or substantial
    portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
    LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
    --------------------------------------------------------------------------------------------------------
}
