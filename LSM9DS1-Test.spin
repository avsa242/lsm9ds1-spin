{
    --------------------------------------------
    Filename: LSM9DS1-Test.spin
    Author: Jesse Burt
    Description: Test harness for LSM9DS1 driver
    Copyright (c) 2019
    Started Feb 9, 2019
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
    COL_SET     = COL_REG+20
    COL_READ    = COL_REG+32
    COL_PF      = COl_REG+48
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

    BLE(1)
    BDU(1)
    H_LACTIVE(1)
    ODR(1)
    FS(1)
    LP_MODE(1)
    OUT_TEMP(5)
    IG_XL(1)
    IG_G(1)
    IG_INACT(1)
    TDA(1)
    GDA(1)
    XLDA(1)
    FS_XL(1)
    HR(1)
    SLEEP_G(1)
    FIFO_EN (1)
    SLEEP_ON_INACT_EN (1)
    ACT_THS (1)
    ACT_DUR (1)
    FMODE(1)
    flash

PUB FMODE(reps) | tmp, read

    _test_row := 21
    repeat reps
        repeat tmp from 0 to 6
            case tmp
                2, 5:
                    next
                OTHER:
            imu.FIFOMode (tmp)
            read := imu.FIFOMode (-2)
            Message (string("FMODE"), tmp, read)

PUB ACT_DUR(reps) | tmp, read

    _test_row := 20
    repeat reps
        repeat tmp from 0 to 255
            imu.GyroActivityDur (tmp)
            read := imu.GyroActivityDur (-2)
            Message (string("ACT_DUR"), tmp, read)

PUB ACT_THS(reps) | tmp, read

    _test_row := 19
    repeat reps
        repeat tmp from 0 to 127
            imu.GyroActivityThr (tmp)
            read := imu.GyroActivityThr (-2)
            Message (string("ACT_THS"), tmp, read)

PUB SLEEP_ON_INACT_EN(reps) | tmp, read

    _test_row := 18
    repeat reps
        repeat tmp from 0 to 1
            imu.GyroInactiveSleep (tmp)
            read := imu.GyroInactiveSleep (-2)
            Message (string("SLEEP_ON_INACT_EN"), tmp, read)

PUB FIFO_EN(reps) | tmp, read

    _test_row := 17
    repeat reps
        repeat tmp from 0 to 1
            imu.FIFO (tmp)
            read := imu.FIFO (-2)
            Message (string("FIFO_EN"), tmp, read)

PUB SLEEP_G(reps) | tmp, read

    _test_row := 16
    repeat reps
        repeat tmp from 0 to 1
            imu.GyroSleep (tmp)
            read := imu.GyroSleep (-2)
            Message (string("SLEEP_G"), tmp, read)

PUB HR(reps) | tmp, read

    _test_row := 15
    repeat reps
        repeat tmp from 0 to 1
            imu.AccelHighRes (tmp)
            read := imu.AccelHighRes (-2)
            Message (string("HR"), tmp, read)

PUB FS_XL(reps) | tmp, read

    _test_row := 14
    repeat reps
        repeat tmp from 1 to 4
            imu.AccelScale (lookup(tmp: 2, 16, 4, 8))
            read := imu.AccelScale (-2)
            Message (string("FS_XL"), lookup(tmp: 2, 16, 4, 8), read)

PUB XLDA(reps) | read
' XXX No verification
    _test_row := 13
    repeat reps
        read := imu.AvailAccel
        Message (string("XLDA"), read, read)

PUB GDA(reps) | read
' XXX No verification
    _test_row := 12
    repeat reps
        read := imu.AvailGyro
        Message (string("GDA"), read, read)

PUB TDA(reps) | read
' XXX No verification
    _test_row := 11
    repeat reps
        read := imu.AvailTemp
        Message (string("TDA"), read, read)

PUB IG_INACT (reps) | read
' XXX No verification
    _test_row := 10
    repeat reps
        read := imu.IntInactivity
        Message (string("IG_INACT"), read, read)

PUB IG_G (reps) | read
' XXX No verification
    _test_row := 9
    repeat reps
        read := imu.IntGyro
        Message (string("IG_G"), read, read)

PUB IG_XL (reps) | read
' XXX No verification
    _test_row := 8
    repeat reps
        read := imu.IntAccel
        Message (string("IG_XL"), read, read)

PUB OUT_TEMP(reps) | read
' XXX No verification
    _test_row := 7
    repeat reps
        read := imu.Temperature
        Message (string("OUT_TEMP"), read, read)

PUB LP_MODE(reps) | tmp, read

    _test_row := 6
    repeat reps
        repeat tmp from 0 to 1
            imu.GyroLowPower (tmp)
            read := imu.GyroLowPower (-2)
            Message (string("LP_MODE"), tmp, read)

PUB FS(reps) | tmp, read

    _test_row := 5
    repeat reps
        repeat tmp from 1 to 4
            if tmp == 3
                next
            imu.GyroScale (lookup(tmp: 245, 500, 0, 2000))
            read := imu.GyroScale (-2)
            Message (string("FS"), lookup(tmp: 245, 500, 0, 2000), read)

PUB ODR(reps) | tmp, read

    _test_row := 4
    repeat reps
        repeat tmp from 1 to 7
            imu.AGDataRate (lookup(tmp: 0, 14{.9}, 59{.5}, 119, 238, 476, 952))
            read := imu.AGDataRate (-2)
            Message (string("ODR"), lookup(tmp: 0, 14{.9}, 59{.5}, 119, 238, 476, 952), read)

PUB H_LACTIVE(reps) | tmp, read

    _test_row := 3
    repeat reps
        repeat tmp from 0 to 1
            imu.IntLevel (tmp)
            read := imu.IntLevel (-2)
            Message (string("H_LACTIVE"), tmp, read)

PUB BDU(reps) | tmp, read

    _test_row := 2
    repeat reps
        repeat tmp from 0 to 1
            imu.BlockUpdate (tmp)
            read := imu.BlockUpdate (-2)
            Message (string("BDU"), tmp, read)

PUB BLE(reps) | tmp, read

    _test_row := 1
    repeat reps
        repeat tmp from 0 to 1
            imu.Endian (tmp)
            read := imu.Endian (-2)
            Message (string("BLE"), tmp, read)

PUB SW_RESET(reps) | tmp    'XXX

    _test_row := 0
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

    ser.PositionY (_test_row)
    ser.PositionX (COL_REG)
    ser.Str (field)

    ser.PositionX (COL_SET)
    ser.Str (string("SET: "))
    ser.Hex (arg1, 4)

    ser.PositionX (COL_READ)
    ser.Str (string("   READ: "))
    ser.Hex (arg2, 4)

    ser.PositionX (COL_PF)
    PassFail (arg1 == arg2)
'    ser.Chars (32, (COL_READ+9)-COL_SET)
'    ser.NewLine

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
