{
    --------------------------------------------
    Filename: LSM9DS1-Test.spin
    Author: Jesse Burt
    Description: Test harness for LSM9DS1 driver
    Copyright (c) 2019
    Started Aug 12, 2017
    Updated Feb 10, 2019
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

    byte  _ser_cog, _imu_cog
    byte  _max_cols

PUB Main

    Setup

    BLE(1)
    BDU(1)
    H_LACTIVE(1)
    ODR(1)
    FS(1)
    LP_MODE(1)
    OUT_TEMP(5)
    flash


PUB OUT_TEMP(reps) | read

    repeat reps
        read := imu.Temperature
        Message (string("OUT_TEMP"), read, read)

PUB LP_MODE(reps) | tmp, read

    repeat reps
        repeat tmp from 0 to 1
            imu.GyroLowPower (tmp)
            read := imu.GyroLowPower (-2)
            Message (string("LP_MODE"), tmp, read)

PUB FS(reps) | tmp, read

    repeat reps
        repeat tmp from 1 to 4
            if tmp == 3
                next
            imu.GyroScale (lookup(tmp: 245, 500, 0, 2000))
            read := imu.GyroScale (-2)
            Message (string("FS"), lookup(tmp: 245, 500, 0, 2000), read)

PUB ODR(reps) | tmp, read

    repeat reps
        repeat tmp from 1 to 7
            imu.AGDataRate (lookup(tmp: 0, 14{.9}, 59{.5}, 119, 238, 476, 952))
            read := imu.AGDataRate (-2)
            Message (string("ODR"), lookup(tmp: 0, 14{.9}, 59{.5}, 119, 238, 476, 952), read)

PUB H_LACTIVE(reps) | tmp, read

    repeat reps
        repeat tmp from 0 to 1
            imu.IntLevel (tmp)
            read := imu.IntLevel (-2)
            Message (string("H_LACTIVE"), tmp, read)

PUB BDU(reps) | tmp, read

    repeat reps
        repeat tmp from 0 to 1
            imu.BlockUpdate (tmp)
            read := imu.BlockUpdate (-2)
            Message (string("BDU"), tmp, read)

PUB BLE(reps) | tmp, read

    repeat reps
        repeat tmp from 0 to 1
            imu.Endian (tmp)
            read := imu.Endian (-2)
            Message (string("BLE"), tmp, read)

PUB SW_RESET(reps) | tmp    'XXX

    tmp := imu.SWReset
    ser.Str (string(ser#NL, "SW Reset sent to IMU", ser#NL))
    ser.Hex (tmp, 2)

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


PUB TrueFalse(num)

    case num
        0: ser.Str (string("FALSE"))
        -1: ser.Str (string("TRUE"))
        OTHER: ser.Str (string("???"))

PUB Message(field, arg1, arg2)

    ser.PositionX ( COL_REG)
    ser.Str (field)

    ser.PositionX ( COL_SET)
    ser.Str (string("SET: "))
    ser.Dec (arg1)

    ser.PositionX ( COL_READ)
    ser.Str (string("   READ: "))
    ser.Dec (arg2)

    ser.PositionX (COL_PF)
    PassFail (arg1 == arg2)
    ser.NewLine

PUB PassFail(num)

    case num
        0: ser.Str (string("FAIL"))
        -1: ser.Str (string("PASS"))
        OTHER: ser.Str (string("???"))

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
