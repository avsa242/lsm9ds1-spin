{
    --------------------------------------------
    Filename: sensor.imu.9dof.lsm9ds1.spi.spin
    Author: Jesse Burt
    Description: Driver for the ST LSM9DS1 9DoF/3-axis IMU
    Copyright (c) 2021
    Started Aug 12, 2017
    Updated Jun 5, 2021
    See end of file for terms of use.
    --------------------------------------------
}

CON

' Indicate to user apps how many Degrees of Freedom each sub-sensor has
'   (also imply whether or not it has a particular sensor)
    ACCEL_DOF               = 3
    GYRO_DOF                = 3
    MAG_DOF                 = 3
    BARO_DOF                = 0
    DOF                     = ACCEL_DOF + GYRO_DOF + MAG_DOF + BARO_DOF

' Scales and data rates used during calibration/bias/offset process
    CAL_XL_SCL              = 2
    CAL_G_SCL               = 245
    CAL_M_SCL               = 4
    CAL_XL_DR               = 238
    CAL_G_DR                = 238
    CAL_M_DR                = 80

' Constants used in low-level SPI read/write
    READ                    = 1 << 7
    WRITE                   = 0
    MS                      = 1 << 6

' Bias adjustment (AccelBias(), GyroBias(), MagBias()) read or write
    R                       = 0
    W                       = 1

' Axis-specific constants
    X_AXIS                  = 0
    Y_AXIS                  = 1
    Z_AXIS                  = 2
    ALL_AXIS                = 3

' Temperature scale constants
    C                       = 0
    F                       = 1

' Endian constants
    LITTLE                  = 0
    BIG                     = 1

' Interrupt active states (applies to both XLG and Mag)
    ACTIVE_HIGH             = 0
    ACTIVE_LOW              = 1

' FIFO settings
    FIFO_OFF                = core#FIFO_OFF
    FIFO_THS                = core#FIFO_THS
    FIFO_CONT_TRIG          = core#FIFO_CONT_TRIG
    FIFO_OFF_TRIG           = core#FIFO_OFF_TRIG
    FIFO_CONT               = core#FIFO_CONT

' Sensor-specific constants
    XLG                     = 0
    MAG                     = 1
    BOTH                    = 2

' Magnetometer operation modes
    MAG_OPMODE_CONT         = %00
    MAG_OPMODE_SINGLE       = %01
    MAG_OPMODE_POWERDOWN    = %10

' Magnetometer performance setting
    MAG_PERF_LOW            = %00
    MAG_PERF_MED            = %01
    MAG_PERF_HIGH           = %10
    MAG_PERF_ULTRA          = %11

' Operating modes (dummy)
    STANDBY                 = 0
    MEASURE                 = 1

' Gyroscope operating modes (dummy)
    #0, POWERDOWN, SLP, NORMAL

' Accel & gyro interrupts
    XLG_INT                 = 1 << 6
    Z_HIGH                  = 1 << 5
    Z_LOW                   = 1 << 4
    Y_HIGH                  = 1 << 3
    Y_LOW                   = 1 << 2
    X_HIGH                  = 1 << 1
    X_LOW                   = 1

' INT1 pin interrupts
    INT1_IG_G               = 1 << 7
    INT1_IG_XL              = 1 << 6
    INT1_FSS5               = 1 << 5
    INT1_OVR                = 1 << 4
    INT1_FTH                = 1 << 3
    INT1_BOOT               = 1 << 2
    INT1_DRDY_G             = 1 << 1
    INT1_DRDY_XL            = 1

' INT2 pin interrupts
    INT2_IG_G               = 1 << 7
    INT2_FSS5               = 1 << 5
    INT2_OVR                = 1 << 4
    INT2_FTH                = 1 << 3
    INT2_DRDY_TEMP          = 1 << 2
    INT2_DRDY_G             = 1 << 1
    INT2_DRDY_XL            = 1

OBJ

    spi     : "com.spi.4w"
    core    : "core.con.lsm9ds1"
    time    : "time"

VAR

    long _gres, _gbiasraw[GYRO_DOF]
    long _ares, _abiasraw[ACCEL_DOF]
    long _mres
    long _CS_AG, _CS_M
    byte _temp_scale

PUB Null{}
' This is not a top-level object

PUB Startx(CS_AG_PIN, CS_M_PIN, SCL_PIN, SDIO_PIN): status
' Start using custom I/O pins
    if lookdown(SCL_PIN: 0..31) and lookdown(SDIO_PIN: 0..31) and {
}   lookdown(CS_AG_PIN: 0..31) and lookdown(CS_M_PIN: 0..31)
        if (status := spi.init(SCL_PIN, SDIO_PIN, SDIO_PIN, core#SPI_MODE))
            longmove(@_CS_AG, @CS_AG_PIN, 2)
            outa[_CS_AG] := 1                   ' make sure CS starts
            outa[_CS_M] := 1                    '   high
            dira[_CS_AG] := 1
            dira[_CS_M] := 1
            time.usleep(core#TPOR)              ' startup time

            xlgsoftreset{}                      ' reset/initialize to
            magsoftreset{}                      ' POR defaults

            if deviceid{} == core#WHOAMI_BOTH_RESP
                return status                   ' validate device
    ' if this point is reached, something above failed
    ' Double check I/O pin assignments, connections, power
    ' Lastly - make sure you have at least one free core/cog
    return FALSE

PUB Stop{}

    spi.deinit{}

PUB Defaults{}
' Factory default settings
    xlgsoftreset{}
    magsoftreset{}
    time.usleep(core#TPOR)

PUB Preset_XL_G_M_3WSPI{}
' Like Defaults(), but
'   * enables output data (XL/G: 59Hz, Mag: 40Hz)
'   * sets SPI mode to 3-wire
'   * disables magnetometer I2C interface
    xlgsoftreset{}
    magsoftreset{}
    time.usleep(core#TPOR)

' Set both the Accel/Gyro and Mag to 3-wire SPI mode
    setspi3wiremode{}
    addressautoinc(TRUE)
    magi2c(FALSE)                               ' disable mag I2C interface
    xlgdatablockupdate(TRUE)                    ' ensure MSB & LSB are from
    magblockupdate(TRUE)                        '   the same "frame" of data
    xlgdatarate(59)                             ' arbitrary
    magdatarate(40_000)                         '
    gyroscale(245)                              ' already the POR defaults,
    accelscale(2)                               ' but still need to call these
    magscale(4)                                 ' to set scale factor hub vars

PUB AccelAxisEnabled(mask): curr_mask
' Enable data output for Accelerometer - per axis
'   Valid values: FALSE (0) or TRUE (1 or -1), for each axis
'   Any other value polls the chip and returns the current setting
    readreg(XLG, core#CTRL_REG5_XL, 1, @curr_mask)
    case mask
        %000..%111:
            mask <<= core#XEN_XL
        other:
            return ((curr_mask >> core#EN_XL) & core#EN_XL_BITS)

    mask := ((curr_mask & core#EN_XL_MASK) | mask)
    writereg(XLG, core#CTRL_REG5_XL, 1, @mask)

PUB AccelBias(axbias, aybias, azbias, rw)
' Read or write/manually set accelerometer calibration offset values
'   Valid values:
'       rw:
'           R (0), W (1)
'       axbias, aybias, azbias:
'           -32768..32767
'   NOTE: When rw is set to READ, axbias, aybias and azbias must be addresses
'       of respective variables to hold the returned calibration offset values
    case rw
        R:
            long[axbias] := _abiasraw[X_AXIS]
            long[aybias] := _abiasraw[Y_AXIS]
            long[azbias] := _abiasraw[Z_AXIS]
        W:
            case axbias
                -32768..32767:
                    _abiasraw[X_AXIS] := axbias
                other:
            case aybias
                -32768..32767:
                    _abiasraw[Y_AXIS] := aybias
                other:
            case azbias
                -32768..32767:
                    _abiasraw[Z_AXIS] := azbias
                other:

PUB AccelData(ax, ay, az) | tmp[2]
' Reads the Accelerometer output registers
    readreg(XLG, core#OUT_X_L_XL, 6, @tmp)

    long[ax] := ~~tmp.word[X_AXIS] - _abiasraw[X_AXIS]
    long[ay] := ~~tmp.word[Y_AXIS] - _abiasraw[Y_AXIS]
    long[az] := ~~tmp.word[Z_AXIS] - _abiasraw[Z_AXIS]

PUB AccelDataOverrun{}: flag
' Dummy method

PUB AccelDataRate(rate): curr_rate
' Set accelerometer output data rate, in Hz
'   NOTE: This is locked with the gyroscope output data rate
'       (hardware limitation)
    return xlgdatarate(rate)

PUB AccelDataReady{} | tmp
' Accelerometer sensor new data available
'   Returns TRUE or FALSE
    readreg(XLG, core#STATUS_REG, 1, @tmp)
    result := (((tmp >> core#XLDA) & 1) == 1)

PUB AccelG(ax, ay, az) | tmpx, tmpy, tmpz
' Reads the Accelerometer output registers and scales the outputs to
'   micro-g's (1_000_000 = 1.000000 g = 9.8 m/s/s)
    acceldata(@tmpx, @tmpy, @tmpz)
    long[ax] := tmpx * _ares
    long[ay] := tmpy * _ares
    long[az] := tmpz * _ares

PUB AccelHighRes(state): curr_state
' Enable high resolution mode for accelerometer
'   Valid values: FALSE (0) or TRUE (1 or -1)
'   Any other value polls the chip and returns the current setting
    return booleanchoice(XLG, core#CTRL_REG7_XL, core#HR, core#HR, {
}   core#CTRL_REG7_XL_MASK, state, 1)

PUB AccelInt{}: int_src
' Read accelerometer interrupts
'   Bit 6..0
'       6: one or more interrupts have been generated
'       5: Z-axis high
'       4: Z-axis low
'       3: Y-axis high
'       2: Y-axis low
'       1: X-axis high
'       0: X-axis low
'   NOTE: Calling this method will clear the interrupts
    int_src := 0
    readreg(XLG, core#INT_GEN_SRC_XL, 1, @int_src)

PUB AccelIntDur(samples): curr_smp
' Set number of samples accelerometer data must be past threshold to be
'   considered an interrupt
'   Valid values: 0..127
'   Any other value polls the chip and returns the current setting
    curr_smp := 0
    readreg(XLG, core#INT_GEN_DUR_XL, 1, @curr_smp)
    case samples
        0..127:
        other:
            return (curr_smp & core#SAMPLES_BITS)

    samples := ((curr_smp & core#SAMPLES_MASK) | samples)
    writereg(XLG, core#INT_GEN_DUR_XL, 1, @samples)

PUB AccelIntHyst(state): curr_state
' Enable accelerometer interrupt hysteresis
'   Valid values: TRUE (-1 or 1), FALSE (0)
'   Any other value polls the chip and returns the current setting
'   NOTE: The hysteresis used is equivalent to/set by the interrupt
'       duration time AccelIntDur()
    curr_state := 0
    readreg(XLG, core#INT_GEN_DUR_XL, 1, @curr_state)
    case ||(state)
        0, 1:
            state := ||(state) << core#WAIT_XL
        other:
            return ((curr_state >> core#WAIT_XL) & 1) == 1

    state := ((curr_state & core#WAIT_XL_MASK) | state)
    writereg(XLG, core#INT_GEN_DUR_XL, 1, @state)

PUB AccelIntMask(mask): curr_mask
' Set accelerometer interrupt mask
'   Valid values:
'       bits 7..0
'       7: *OR (0)/AND (1) interrupt events
'       6: 6-direction detection
'       5: Z-axis high
'       4: Z-axis low
'       3: Y-axis high
'       2: Y-axis low
'       1: Z-axis high
'       0: Z-axis low
'   Any other value polls the chip and returns the current setting
    case mask
        %00000000..%11111111:
            writereg(XLG, core#INT_GEN_CFG_XL, 1, @mask)
        other:
            curr_mask := 0
            readreg(XLG, core#INT_GEN_CFG_XL, 1, @curr_mask)
            return

PUB AccelIntThresh(x, y, z, rw) | lsb, ascl, tmp
' Set accelerometer interrupt thresholds per axis, in micro-g's (unsigned)
'   Valid values: 0..(full-scale * 1_000_000)
'   Any other value will be clamped to min/max limits
'   NOTE: When rw == R (0), x, y, and z must be pointers to variables
'       to hold values read from chip
    ascl := accelscale(-2) * 1_000000
    lsb := ascl / 256                           ' calc LSB for the thresh reg

    case rw
        W:
            x := 0 #> x <# ascl                 ' clamp values to full-scale
            y := 0 #> y <# ascl
            z := 0 #> z <# ascl
            x /= lsb                            ' scale values down to reg's
            y /= lsb                            '   8-bit format
            z /= lsb
            writereg(XLG, core#INT_GEN_THS_X_XL, 1, @x)
            writereg(XLG, core#INT_GEN_THS_Y_XL, 1, @y)
            writereg(XLG, core#INT_GEN_THS_Z_XL, 1, @z)
        R:
            tmp := 0
            readreg(XLG, core#INT_GEN_THS_X_XL, 3, @tmp)
            long[x] := tmp.byte[X_AXIS] * lsb   ' scale values up to output
            long[y] := tmp.byte[Y_AXIS] * lsb   '   data scale (micro-g's)
            long[z] := tmp.byte[Z_AXIS] * lsb

PUB AccelScale(scale): curr_scl
' Sets the full-scale range of the Accelerometer, in g's
'   Valid values: 2, 4, 8, 16
'   Any other value polls the chip and returns the current setting
    readreg(XLG, core#CTRL_REG6_XL, 1, @curr_scl)
    case scale
        2, 4, 8, 16:
            scale := lookdownz(scale: 2, 16, 4, 8)
            _ares := lookupz(scale: 0_000061, 0_000732, 0_000122, 0_000244)
            scale <<= core#FS_XL
        other:
            curr_scl := ((curr_scl >> core#FS_XL) & core#FS_XL_BITS) + 1
            return lookup(curr_scl: 2, 16, 4, 8)

    scale := ((curr_scl & core#FS_XL_MASK) | scale)
    writereg(XLG, core#CTRL_REG6_XL, 1, @scale)

PUB CalibrateAccel{} | axis, orig_res, orig_scl, orig_dr, tmp[ACCEL_DOF], tmpx, tmpy, tmpz, samples
' Calibrate the accelerometer
'   NOTE: The accelerometer must be oriented with the package top facing up
'       for this method to be successful
    longfill(@axis, 0, 11)                      ' initialize vars to 0
    orig_scl := accelscale(-2)                  ' save user's current settings
    orig_dr := acceldatarate(-2)
    accelbias(0, 0, 0, W)                       ' clear existing bias

    ' set sensor to CAL_XL_SCL range, CAL_XL_DR Hz data rate
    accelscale(CAL_XL_SCL)
    acceldatarate(CAL_XL_DR)
    samples := CAL_XL_DR                        ' samples = DR, for 1 sec time

    ' accumulate and average approx. 1sec worth of samples
    repeat samples
        repeat until acceldataready{}
        acceldata(@tmpx, @tmpy, @tmpz)
        tmp[X_AXIS] += tmpx
        tmp[Y_AXIS] += tmpy
        tmp[Z_AXIS] += (tmpz-(1_000_000 / _ares))' cancel out 1g on Z-axis

    repeat axis from X_AXIS to Z_AXIS           ' calc avg
        tmp[axis] /= samples

    ' update offsets
    accelbias(tmp[X_AXIS], tmp[Y_AXIS], tmp[Z_AXIS], W)

    accelscale(orig_scl)                        ' restore user's settings
    acceldatarate(orig_dr)

PUB CalibrateGyro{} | axis, orig_scl, orig_dr, tmpx, tmpy, tmpz, tmp[GYRO_DOF], samples
' Calibrate the gyroscope
    longfill(@axis, 0, 10)                      ' initialize vars to 0
    orig_scl := gyroscale(-2)                   ' save user's current settings
    orig_dr := gyrodatarate(-2)
    gyrobias(0, 0, 0, W)                        ' clear existing bias

    ' set sensor to CAL_G_SCL range, CAL_G_DR Hz data rate
    gyroscale(CAL_G_SCL)
    gyrodatarate(CAL_G_DR)
    samples := CAL_G_DR                         ' samples = DR, for 1 sec time

    ' accumulate and average approx. 1sec worth of samples
    repeat samples
        repeat until gyrodataready{}
        gyrodata(@tmpx, @tmpy, @tmpz)
        tmp[X_AXIS] += tmpx
        tmp[Y_AXIS] += tmpy
        tmp[Z_AXIS] += tmpz

    repeat axis from X_AXIS to Z_AXIS           ' calc avg
        tmp[axis] /= samples

    ' update offsets
    gyrobias(tmp[X_AXIS], tmp[Y_AXIS], tmp[Z_AXIS], W)

    gyroscale(orig_scl)                         ' restore user's settings
    gyrodatarate(orig_dr)

PUB CalibrateMag{} | axis, orig_scl, orig_dr, tmpx, tmpy, tmpz, tmp[MAG_DOF], samples
' Calibrate the magnetometer
    longfill(@axis, 0, 11)                      ' initialize vars to 0
    orig_scl := magscale(-2)                    ' save user's current settings
    orig_dr := magdatarate(-2)
    magbias(0, 0, 0, W)                         ' clear existing bias

    ' set sensor to CAL_M_SCL range, CAL_M_DR Hz data rate
    magscale(CAL_M_SCL)
    magdatarate(CAL_M_DR)
    samples := CAL_M_DR                         ' samples = DR, for 1 sec time

    ' accumulate and average approx. 1sec worth of samples
    repeat samples
        repeat until magdataready{}
        magdata(@tmpx, @tmpy, @tmpz)
        tmp[X_AXIS] += tmpx
        tmp[Y_AXIS] += tmpy
        tmp[Z_AXIS] += tmpz

    repeat axis from X_AXIS to Z_AXIS           ' calc avg
        tmp[axis] /= samples

    ' update offsets
    magbias(tmp[X_AXIS], tmp[Y_AXIS], tmp[Z_AXIS], W)

    magscale(orig_scl)                          ' restore user's settings
    magdatarate(orig_dr)

PUB DeviceID{}: id
' Read device identification
'   Returns: $683D
    id := 0
    readreg(XLG, core#WHO_AM_I_XG, 1, @id.byte[1])
    readreg(MAG, core#WHO_AM_I_M, 1, @id.byte[0])

PUB Endian(order): curr_order  'XXX rename to DataByteOrder()
' Choose byte order of acclerometer/gyroscope data
'   Valid values: LITTLE (0) or BIG (1)
'   Any other value polls the chip and returns the current setting
    readreg(XLG, core#CTRL_REG8, 1, @curr_order)
    case order
        LITTLE, BIG:
            order := order << core#BLE
        other:
            return ((curr_order >> core#BLE) & 1)

    order := ((curr_order & core#BLE_MASK) | order)
    writereg(XLG, core#CTRL_REG8, 1, @order)

PUB FIFOEnabled(state): curr_state
' Enable FIFO memory
'   Valid values: FALSE (0), TRUE(1 or -1)
'   Any other value polls the chip and returns the current setting
    return booleanchoice(XLG, core#CTRL_REG9, core#FIFO_EN, core#FIFO_EN,{
}   core#CTRL_REG9_MASK, state, 1)

PUB FIFOFull{}: flag
' FIFO Threshold status
'   Returns: FALSE (0): lower than threshold level, TRUE(-1): at or higher than threshold level
    readreg(XLG, core#FIFO_SRC, 1, @flag)
    return (((flag >> core#FTH_STAT) & 1) == 1)

PUB FIFOMode(mode): curr_mode
' Set FIFO behavior
'   Valid values:
'       FIFO_OFF        (%000) - Bypass mode - FIFO off
'       FIFO_THS        (%001) - Stop collecting data when FIFO full
'       FIFO_CONT_TRIG  (%011) - Continuous mode until trigger is deasserted,
'           then FIFO mode
'       FIFO_OFF_TRIG   (%100) - FIFO off until trigger is deasserted,
'           then continuous mode
'       FIFO_CONT       (%110) - Continuous mode. If FIFO full, new sample
'           overwrites older sample
'   Any other value polls the chip and returns the current setting
    readreg(XLG, core#FIFO_CTRL, 1, @curr_mode)
    case mode
        FIFO_OFF, FIFO_THS, FIFO_CONT_TRIG, FIFO_OFF_TRIG, FIFO_CONT:
            mode <<= core#FMODE
        other:
            return (curr_mode >> core#FMODE) & core#FMODE_BITS

    mode := ((curr_mode & core#FMODE_MASK) | mode)
    writereg(XLG, core#FIFO_CTRL, 1, @mode)

PUB FIFODataOverrun{}: flag
' Flag indicating FIFO has overrun
'   Returns:
'       TRUE (-1) if at least one sample has been overwritten
'       FALSE (0) otherwise
    flag := 0
    readreg(XLG, core#FIFO_SRC, 1, @flag)
    return ((flag >> core#OVRN) & 1) == 1

PUB FIFOThreshold(level): curr_lvl
' Set FIFO threshold level
'   Valid values: 0..31
'   Any other value polls the chip and returns the current setting
    readreg(XLG, core#FIFO_CTRL, 1, @curr_lvl)
    case level
        0..31:
        other:
            return curr_lvl & core#FTH_BITS

    level := ((curr_lvl & core#FTH_MASK) | level)
    writereg(XLG, core#FIFO_CTRL, 1, @level)

PUB FIFOUnreadSamples{}: nr_samples
' Number of unread samples stored in FIFO
'   Returns: 0 (empty) .. 32
    readreg(XLG, core#FIFO_SRC, 1, @nr_samples)
    return nr_samples & core#FSS_BITS

PUB GyroAxisEnabled(mask): curr_mask
' Enable Gyroscope data output, per axis mask
'   Valid values: 0 or 1, for each axis
'   Any other value polls the chip and returns the current setting
    readreg(XLG, core#CTRL_REG4, 1, @curr_mask)
    case mask
        %000..%111:
            mask <<= core#XEN_G
        other:
            return (curr_mask >> core#XEN_G) & core#EN_G_BITS

    mask := ((curr_mask & core#EN_G_MASK) | mask)
    writereg(XLG, core#CTRL_REG4, 1, @mask)

PUB GyroBias(gxbias, gybias, gzbias, rw)
' Read or write/manually set Gyroscope calibration offset values
'   Valid values:
'       rw:
'           R (0), W (1)
'       gxbias, gybias, gzbias:
'           -32768..32767
'   NOTE: When rw is set to READ, gxbias, gybias and gzbias must be addresses
'       of respective variables to hold the returned calibration offset values.
    case rw
        R:
            long[gxbias] := _gbiasraw[X_AXIS]
            long[gybias] := _gbiasraw[Y_AXIS]
            long[gzbias] := _gbiasraw[Z_AXIS]
        W:
            case gxbias
                -32768..32767:
                    _gbiasraw[X_AXIS] := gxbias
                other:
            case gybias
                -32768..32767:
                    _gbiasraw[Y_AXIS] := gybias
                other:
            case gzbias
                -32768..32767:
                    _gbiasraw[Z_AXIS] := gzbias
                other:

PUB GyroData(ptr_x, ptr_y, ptr_z) | tmp[2]
' Read gyroscope data
    readreg(XLG, core#OUT_X_G_L, 6, @tmp)
    long[ptr_x] := ~~tmp.word[X_AXIS] - _gbiasraw[X_AXIS]
    long[ptr_y] := ~~tmp.word[Y_AXIS] - _gbiasraw[Y_AXIS]
    long[ptr_z] := ~~tmp.word[Z_AXIS] - _gbiasraw[Z_AXIS]

PUB GyroDataRate(rate): curr_rate
' Set Gyroscope Output Data Rate, in Hz
'   Valid values: 0, 15, 60, 119, 238, 476, 952
'   Any other value polls the chip and returns the current setting
'   NOTE: 0 powers down the Gyroscope
'   NOTE: 15 and 60 are rounded up from the datasheet specifications of 14.9
'       and 59.5, respectively
    readreg(XLG, core#CTRL_REG1_G, 1, @curr_rate)
    case rate
        0, 15, 60, 119, 238, 476, 952:
            rate := lookdownz(rate: 0, 15, 60, 119, 238, 476, 952) << core#ODR
        other:
            curr_rate := ((curr_rate >> core#ODR) & core#ODR_BITS)
            return lookupz(curr_rate: 0, 15, 60, 119, 238, 476, 952)

    rate := ((curr_rate & core#ODR_MASK) | rate)
    writereg(XLG, core#CTRL_REG1_G, 1, @rate)

PUB GyroDataReady{}: flag
' Flag indicating new gyroscope data available
'   Returns TRUE or FALSE
    readreg(XLG, core#STATUS_REG, 1, @flag)
    return (((flag >> core#GDA) & 1) == 1)

PUB GyroDPS(gx, gy, gz) | tmp[3]
' Read the Gyroscope output registers and scale the outputs to micro-degrees
'   of rotation per second (1_000_000 = 1.000000 deg/sec)
    gyrodata(@tmp[X_AXIS], @tmp[Y_AXIS], @tmp[Z_AXIS])
    long[gx] := tmp[X_AXIS] * _gres
    long[gy] := tmp[Y_AXIS] * _gres
    long[gz] := tmp[Z_AXIS] * _gres

PUB GyroHighPass(freq): curr_freq
' Set Gyroscope high-pass filter cutoff frequency
'   Valid values: dependent on GyroDataRate(), see table below
'   Any other value polls the chip and returns the current setting
    curr_freq := 0
    readreg(XLG, core#CTRL_REG3_G, 1, @curr_freq)
    case gyrodatarate(-2)
        15:
            case freq
                0_001, 0_002, 0_005, 0_010, 0_020, 0_050, 0_100, 0_200, 0_500, 1_000:
                    freq := lookdownz(freq: 1_000, 0_500, 0_200, 0_100, 0_050, 0_020, 0_010, 0_005, 0_002, 0_001) << core#HPCF_G
                other:
                    curr_freq := (curr_freq >> core#HPCF_G) & core#HPCF_G_BITS
                    return lookupz(curr_freq: 1_000, 0_500, 0_200, 0_100, 0_050, 0_020, 0_010, 0_005, 0_002, 0_001)
        60:
            case freq
                0_005, 0_010, 0_020, 0_050, 0_100, 0_200, 0_500, 1_000, 2_000, 4_000:
                    freq := lookdownz(freq: 4_000, 2_000, 1_000, 0_500, 0_200, 0_100, 0_050, 0_020, 0_010, 0_005) << core#HPCF_G
                other:
                    curr_freq := (curr_freq >> core#HPCF_G) & core#HPCF_G_BITS
                    return lookupz(curr_freq: 1_000, 0_500, 0_200, 0_100, 0_050, 0_020, 0_010, 0_005, 0_002, 0_001)
        119:
            case freq
                0_010, 0_020, 0_050, 0_100, 0_200, 0_500, 1_000, 2_000, 4_000, 8_000:
                    freq := lookdownz(freq: 8_000, 4_000, 2_000, 1_000, 0_500, 0_200, 0_100, 0_050, 0_020, 0_010) << core#HPCF_G
                other:
                    curr_freq := (curr_freq >> core#HPCF_G) & core#HPCF_G_BITS
                    return lookupz(curr_freq: 1_000, 0_500, 0_200, 0_100, 0_050, 0_020, 0_010, 0_005, 0_002, 0_001)
        238:
            case freq
                0_020, 0_050, 0_100, 0_200, 0_500, 1_000, 2_000, 4_000, 8_000, 15_000:
                    freq := lookdownz(freq: 15_000, 8_000, 4_000, 2_000, 1_000, 0_500, 0_200, 0_100, 0_050, 0_020) << core#HPCF_G
                other:
                    curr_freq := (curr_freq >> core#HPCF_G) & core#HPCF_G_BITS
                    return lookupz(curr_freq: 1_000, 0_500, 0_200, 0_100, 0_050, 0_020, 0_010, 0_005, 0_002, 0_001)
        476:
            case freq
                0_050, 0_100, 0_200, 0_500, 1_000, 2_000, 4_000, 8_000, 15_000, 30_000:
                    freq := lookdownz(freq: 30_000, 15_000, 8_000, 4_000, 2_000, 1_000, 0_500, 0_200, 0_100, 0_050) << core#HPCF_G
                other:
                    curr_freq := (curr_freq >> core#HPCF_G) & core#HPCF_G_BITS
                    return lookupz(curr_freq: 1_000, 0_500, 0_200, 0_100, 0_050, 0_020, 0_010, 0_005, 0_002, 0_001)
        952:
            case freq
                0_100, 0_200, 0_500, 1_000, 2_000, 4_000, 8_000, 15_000, 30_000, 57_000:
                    freq := lookdownz(freq: 57_000, 30_000, 15_000, 8_000, 4_000, 2_000, 1_000, 0_500, 0_200, 0_100) << core#HPCF_G
                other:
                    curr_freq := (curr_freq >> core#HPCF_G) & core#HPCF_G_BITS
                    return lookupz(curr_freq: 1_000, 0_500, 0_200, 0_100, 0_050, 0_020, 0_010, 0_005, 0_002, 0_001)

    freq := ((curr_freq & core#HPCF_G_MASK) | freq)
    writereg(XLG, core#CTRL_REG3_G, 1, @freq)

PUB GyroInactiveDur(duration): curr_dur
' Set gyroscope inactivity timer (use GyroInactiveSleep() to define behavior on
'   inactivity)
'   Valid values: 0..255 (0 effectively disables the feature)
'   Any other value polls the chip and returns the current setting
    curr_dur := 0
    readreg(XLG, core#ACT_DUR, 1, @curr_dur)
    case duration
        0..255:
        other:
            return curr_dur

    writereg(XLG, core#ACT_DUR, 1, @duration)

PUB GyroInactiveThr(thresh): curr_thr
' Set gyroscope inactivity threshold (use GyroInactiveSleep() to define
'   behavior on inactivity)
'   Valid values: 0..127 (0 effectively disables the feature)
'   Any other value polls the chip and returns the current setting
    curr_thr := 0
    readreg(XLG, core#ACT_THS, 1, @curr_thr)
    case thresh
        0..127:
        other:
            return curr_thr & core#ACT_THR_BITS

    thresh := ((curr_thr & core#ACT_THR_MASK) | thresh)
    writereg(XLG, core#ACT_THS, 1, @thresh)

PUB GyroInactiveSleep(state): curr_state
' Enable gyroscope sleep mode when inactive (see GyroActivityThr())
'   Valid values:
'       FALSE (0): Gyroscope powers down
'       TRUE (1 or -1) Gyroscope enters sleep mode
'   Any other value polls the chip and returns the current setting
    return booleanChoice(XLG, core#ACT_THS, core#SLP_ON_INACT, {
}   core#SLP_ON_INACT_MASK, core#ACT_THS_MASK, state, 1)

PUB GyroInt{}: int_src
' Read gyroscope interrupts
'   Bit 6..0
'       6: one or more interrupts have been generated
'       5: Yaw/Z-axis high
'       4: Yaw/Z-axis low
'       3: Roll/Y-axis high
'       2: Roll/Y-axis low
'       1: Pitch/X-axis high
'       0: Pitch/X-axis low
'   NOTE: Calling this method will clear the interrupts
    int_src := 0
    readreg(XLG, core#INT_GEN_SRC_G, 1, @int_src)

PUB GyroIntDataPath(mode): curr_mode
' Set gyroscope interrupt filter path
'   Valid values:
'       %00: interrupt generator uses data fed by LPF (low-pass filtered) only
'       %01: interrupt generator uses data fed by LPF->HPF (high-pass filtered)
'           (effective only if high-pass filter is enabled)
'       %10, %11: interrupt generator uses data fed by LPF->HPF->2nd LPF
'   Any other value polls the chip and returns the current setting
    curr_mode := 0
    readreg(XLG, core#CTRL_REG2_G, 1, @curr_mode)
    case mode
        %00..%11:
            mode := mode << core#INT_SEL
        other:
            return (curr_mode >> core#INT_SEL) & core#INT_SEL_BITS

    mode := ((curr_mode & core#INT_SEL_MASK) | mode)
    writereg(XLG, core#CTRL_REG2_G, 1, @mode)

PUB GyroIntDur(samples): curr_smp
' Set number of samples gyroscope data must be past threshold to be
'   considered an interrupt
'   Valid values: 0..127
'   Any other value polls the chip and returns the current setting
    curr_smp := 0
    readreg(XLG, core#INT_GEN_DUR_G, 1, @curr_smp)
    case samples
        0..127:
        other:
            return curr_smp & core#SAMPLES_BITS

    samples := ((curr_smp & core#SAMPLES_MASK) | samples)
    writereg(XLG, core#INT_GEN_DUR_G, 1, @samples)

PUB GyroIntHyst(state): curr_state
' Enable gyroscope interrupt hysteresis
'   Valid values: TRUE (-1 or 1), FALSE (0)
'   Any other value polls the chip and returns the current setting
'   NOTE: The hysteresis used is equivalent to/set by the interrupt
'       duration time GyroIntDur()
    curr_state := 0
    readreg(XLG, core#INT_GEN_DUR_G, 1, @curr_state)
    case ||(state)
        0, 1:
            state := ||(state) << core#WAIT_G
        other:
            return ((curr_state >> core#WAIT_G) & 1) == 1

    state := ((curr_state & core#WAIT_G_MASK) | state)
    writereg(XLG, core#INT_GEN_DUR_G, 1, @state)

PUB GyroIntThresh(x, y, z, rw) | gscl, lsb, tmp[2], axis
' Set gyroscope interrupt thresholds per axis, in micro-dps (signed)
'   Valid values: +/- full-scale * 1_000_000
'   Any other value will be clamped to min/max limits
'   NOTE: When rw == R (0), x, y, and z must be pointers to variables
'       to hold values read from chip
    gscl := gyroscale(-2) * 1_000000
    lsb := gscl / 16384                         ' calc LSB for the thresh reg
    case rw
        W:
            x := 0 #> x <# gscl                 ' clamp values to full-scale
            y := 0 #> y <# gscl
            z := 0 #> z <# gscl
            x /= lsb                            ' scale values down to reg's
            y /= lsb                            '   15-bit signed format
            z /= lsb
            writereg(XLG, core#INT_GEN_THS_XH_G, 2, @x)
            writereg(XLG, core#INT_GEN_THS_YH_G, 2, @y)
            writereg(XLG, core#INT_GEN_THS_ZH_G, 2, @z)
        R:
            tmp := 0
            readreg(XLG, core#INT_GEN_THS_XH_G, 6, @tmp)
            ' scale values up to output
            '   data scale (micro-dps)
            repeat axis from X_AXIS to Z_AXIS
                tmp.word[axis] := ((tmp.word[axis] & core#INT_G_BITS) << 1) ~> 1
            long[x] := ~~tmp.word[X_AXIS] * lsb
            long[y] := ~~tmp.word[Y_AXIS] * lsb
            long[z] := ~~tmp.word[Z_AXIS] * lsb

PUB GyroLowPower(state): curr_state
' Enable low-power mode
'   Valid values: FALSE (0), TRUE (1 or -1)
'   Any other value polls the chip and returns the current setting
    return booleanChoice(XLG, core#CTRL_REG3_G, core#LP_MODE, {
}   core#LP_MODE_MASK, core#CTRL_REG3_G_MASK, state, 1)

PUB GyroScale(scale): curr_scale
' Set full scale of gyroscope output, in degrees per second (dps)
'   Valid values: 245, 500, 2000
'   Any other value polls the chip and returns the current setting
    curr_scale := 0
    readreg(XLG, core#CTRL_REG1_G, 1, @curr_scale)
    case scale
        245, 500, 2000:
            scale := lookdownz(scale: 245, 500, 0, 2000)
            _gres := lookupz(scale: 0_008750, 0_017500, 0, 0_070000)
            scale <<= core#FS
        other:
            curr_scale := ((curr_scale >> core#FS) & core#FS_BITS) + 1
            return lookup(curr_scale: 245, 500, 0, 2000)

    scale := ((curr_scale & core#FS_MASK) | scale)
    writereg(XLG, core#CTRL_REG1_G, 1, @scale)

PUB GyroSleep(state): curr_state
' Enable gyroscope sleep mode
'   Valid values: FALSE (0), TRUE (1 or -1)
'   Any other value polls the chip and returns the current setting
'   NOTE: If state, the gyro output will contain the last measured values
    return booleanChoice(XLG, core#CTRL_REG9, core#SLP_G, core#SLP_G_MASK,{
}   core#CTRL_REG9_MASK, state, 1)

PUB Int1Mask(mask): curr_mask
' Set interrupt enable mask on INT1 pin
'   Bits: 7..0 (1=enable interrupt, 0=disable interrupt)
'       7: gyroscope interrupt
'       6: accelerometer interrupt
'       5: FSS5 interrupt
'       4: data overrun interrupt
'       3: FIFO threshold interrupt
'       2: boot status interrupt
'       1: gyroscope data ready interrupt
'       0: accelerometer data ready interrupt
    case mask
        %00000000..%11111111:
            writereg(XLG, core#INT1_CTRL, 1, @mask)
        other:
            readreg(XLG, core#INT1_CTRL, 1, @curr_mask)
            return

PUB Int2Mask(mask): curr_mask
' Set interrupt enable mask on INT2 pin
'   Bits: 7..0 (1=enable interrupt, 0=disable interrupt)
'       7: gyroscope interrupt
'       6: - N/A -
'       5: FSS5 interrupt
'       4: data overrun interrupt
'       3: FIFO threshold interrupt
'       2: boot status interrupt
'       1: gyroscope data ready interrupt
'       0: accelerometer data ready interrupt
    case mask
        %00000000..%11111111:
            mask &= core#INT2_CTRL_MASK         ' mask off bit 6 (unused)
            writereg(XLG, core#INT2_CTRL, 1, @mask)
        other:
            readreg(XLG, core#INT2_CTRL, 1, @curr_mask)
            return

PUB Interrupt{}: flag
' Flag indicating one or more interrupts asserted
'   Returns TRUE if one or more interrupts asserted, FALSE if not
    readreg(XLG, core#INT_GEN_SRC_XL, 1, @flag)
    return (((flag >> core#IA_XL) & 1) == 1)

PUB IntInactivity{}: flag
' Flag indicating inactivity interrupt asserted
'   Returns TRUE if interrupt asserted, FALSE if not
    readreg(XLG, core#STATUS_REG, 1, @flag)
    return (((flag >> core#INACT) & 1) == 1)

PUB MagBlockUpdate(state): curr_state
' Enable block update for magnetometer data
'   Valid values:
'       TRUE(-1 or 1): Output registers not updated until MSB and LSB have been
'           read
'       FALSE(0): Continuous update
'   Any other value polls the chip and returns the current setting
    return booleanChoice (MAG, core#CTRL_REG5_M, core#BDU_M, core#BDU_M_MASK,{
}   core#CTRL_REG5_M_MASK, state, 1)

PUB MagBias(mxbias, mybias, mzbias, rw) | tmp[2]
' Read or write/manually set Magnetometer calibration offset values
'   Valid values:
'       rw:
'           R (0), W (1)
'       mxbias, mybias, mzbias:
'           -32768..32767
'   NOTE: When rw is set to READ, mxbias, mybias and mzbias must be addresses
'       of respective variables to hold the returned calibration offset values
    case rw
        R:
            readreg(MAG, core#OFFSET_X_REG_L_M, 6, @tmp)
            long[mxbias] := ~~tmp.word[X_AXIS]
            long[mybias] := ~~tmp.word[Y_AXIS]
            long[mzbias] := ~~tmp.word[Z_AXIS]
        W:
            case mxbias
                -32768..32767:
                other:
                    return
            case mybias
                -32768..32767:
                other:
                    return
            case mzbias
                -32768..32767:
                other:
                    return
            writereg(MAG, core#OFFSET_X_REG_L_M, 2, @mxbias)
            writereg(MAG, core#OFFSET_Y_REG_L_M, 2, @mybias)
            writereg(MAG, core#OFFSET_Z_REG_L_M, 2, @mzbias)

PUB MagData(ptr_x, ptr_y, ptr_z) | tmp[2]
' Read the Magnetometer output registers
    readreg(MAG, core#OUT_X_L_M, 6, @tmp)
    long[ptr_x] := ~~tmp.word[X_AXIS]           ' no offset correction
    long[ptr_y] := ~~tmp.word[Y_AXIS]           ' because the mag has
    long[ptr_z] := ~~tmp.word[Z_AXIS]           ' offset registers built-in

PUB MagDataOverrun{}: status
' Magnetometer data overrun
'   Returns: Overrun flag as bitfield
'       MSB   LSB
'       |     |
'       3 2 1 0
'       3: All axes data overrun
'       2: Z-axis data overrun
'       1: Y-axis data overrun
'       0: X-axis data overrun
'   Example:
'       %1111: Indicates data has overrun on all axes
'       %0010: Indicates Y-axis data has overrun
'   NOTE: Overrun flag indicates new data for axis has overwritten the previous data.
    readreg(MAG, core#STATUS_REG_M, 1, @status)
    return ((status >> core#OVERRN) & core#OVERRN_BITS)

PUB MagDataRate(rate): curr_rate
' Set Magnetometer Output Data Rate, in Hz
'   Valid values: 0 (0.625Hz), 1 (1.250), 2 (2.5), 5, *10, 20, 40, 80
'   Any other value polls the chip and returns the current setting
    curr_rate := 0
    readreg(MAG, core#CTRL_REG1_M, 1, @curr_rate)
    case rate
        0, 1, 2, 5, 10, 20, 40, 80:
            rate := lookdownz(rate: 0, 1, 2, 5, 10, 20, 40, 80) << core#DO
        other:
            curr_rate := ((curr_rate >> core#DO) & core#DO_BITS)
            return lookupz(curr_rate: 0, 1, 2, 5, 10, 20, 40, 80)

    rate := ((curr_rate & core#DO_MASK) | rate)
    writereg(MAG, core#CTRL_REG1_M, 1, @rate)

PUB MagDataReady{}: flag
' Flag indicating new magnetometer data ready
'   Returns: TRUE (-1) if data available, FALSE (0) otherwise
    readreg(MAG, core#STATUS_REG_M, 1, @flag)
    return (((flag >> core#ZYXDA) & 1) == 1)

PUB MagEndian(order): curr_order
' Choose byte order of magnetometer data
'   Valid values: LITTLE (0) or BIG (1)
'   Any other value polls the chip and returns the current setting
    curr_order := 0
    readreg(MAG, core#CTRL_REG4_M, 1, @curr_order)
    case order
        LITTLE, BIG:
            order := order << core#BLE_M
        other:
            return ((curr_order >> core#BLE_M) & 1)

    order := ((curr_order & core#BLE_M_MASK) | order)
    writereg(MAG, core#CTRL_REG4_M, 1, @order)

PUB MagFastRead(state): curr_state
' Enable reading of only the MSB of data to increase reading efficiency, at
'   the cost of precision and accuracy
'   Valid values: TRUE(-1 or 1), FALSE(0)
'   Any other value polls the chip and returns the current setting
    return booleanChoice (MAG, core#CTRL_REG5_M, core#FAST_READ, {
}   core#FAST_READ_MASK, core#CTRL_REG5_M_MASK, state, 1)

PUB MagGauss(ptr_x, ptr_y, ptr_z) | tmp[3]
' Read the Magnetometer output registers and scale the outputs to micro-Gauss
'   (1_000_000 = 1.000000 Gs)
    magdata(@tmp[X_AXIS], @tmp[Y_AXIS], @tmp[Z_AXIS])
    long[ptr_x] := tmp[X_AXIS] * _mres
    long[ptr_y] := tmp[Y_AXIS] * _mres
    long[ptr_z] := tmp[Z_AXIS] * _mres

PUB MagInt{}: intsrc
' Magnetometer interrupt source(s)
'   Returns: Interrupts that are currently asserted, as a bitmask
'   MSB    LSB
'   |      |
'   76543210
'   7: X-axis exceeds threshold, positive side
'   6: Y-axis exceeds threshold, positive side
'   5: Z-axis exceeds threshold, positive side
'   4: X-axis exceeds threshold, negative side
'   3: Y-axis exceeds threshold, negative side
'   2: Z-axis exceeds threshold, negative side
'   1: A measurement exceeded the magnetometer's measurement range (overflow)
'   0: Interrupt asserted
    readreg(MAG, core#INT_SRC_M, 1, @intsrc)

PUB MagIntActiveState(state): curr_state
' Set active state of INT_MAG pin when magnetometer interrupt asserted
'   Valid values: ACTIVE_LOW (0), ACTIVE_HIGH (1)
'   Any other value polls the chip and returns the current setting
    curr_state := 0
    readreg(MAG, core#INT_CFG_M, 1, @curr_state)
    case state
        ACTIVE_LOW, ACTIVE_HIGH:
            state ^= 1                   ' This bit's polarity is
            state <<= core#IEA           ' opposite that of the XLG
        other:
            return (curr_state >> core#IEA) & 1

    state := ((curr_state & core#IEA_MASK) | state)
    writereg(MAG, core#INT_CFG_M, 1, @state)

PUB MagIntClear{}
' Clear magnetometer interrupts
'   NOTE: This is only required if MagIntsLatched() is set to TRUE
    magint{}

PUB MagIntMask(mask): curr_mask
' Enable magnetometer threshold interrupts, as a bitmask
'   Valid values: %000..%111
'     MSB   LSB
'       |   |
'       2 1 0
'       2: X-axis
'       1: Y-axis
'       0: Z-axis
'   Example:
'       %111: Enable interrupts for all three axes
'       %010: Enable interrupts for Y axis only

'   Any other value polls the chip and returns the current setting
    curr_mask := 0
    readreg(MAG, core#INT_CFG_M, 1, @curr_mask)
    case mask
        %000..%111:
            mask <<= core#XYZIEN
        other:
            return (curr_mask >> core#XYZIEN) & core#XYZIEN_BITS

    mask := ((curr_mask & core#XYZIEN_MASK) | mask)
    writereg(MAG, core#INT_CFG_M, 1, @mask)

PUB MagIntsLatched(state): curr_state
' Latch interrupts asserted by the magnetometer
'   Valid values: TRUE (-1 or 1) or FALSE
'   Any other value polls the chip and returns the current setting
'   NOTE: If enabled, interrupts must be explicitly cleared using MagClearInt()
    return booleanchoice(MAG, core#INT_CFG_M, core#IEL, core#IEL_MASK,{
}   core#INT_CFG_M, state, -1)

PUB MagIntThresh(thresh): curr_thr 'XXX rewrite to take gauss as a param
' Set magnetometer interrupt threshold
'   Valid values: 0..32767
'   Any other value polls the chip and returns the current setting
'   NOTE: The set thresh is an absolute value and is compared to positive and
'       negative measurements alike
    case thresh
        0..32767:
            writereg(MAG, core#INT_THS_L_M, 2, @thresh)
        other:
            curr_thr := 0
            readreg(MAG, core#INT_THS_L_M, 2, @curr_thr)
            return

PUB MagLowPower(state): curr_state
' Enable magnetometer low-power mode
'   Valid values: TRUE (-1 or 1) or FALSE
'   Any other value polls the chip and returns the current setting
    return booleanchoice(MAG, core#CTRL_REG3_M, core#LP, core#LP_MASK, {
}   core#CTRL_REG3_M_MASK, state, 1)

PUB MagOpMode(mode): curr_mode
' Set magnetometer operating mode
'   Valid values:
'       MAG_OPMODE_CONT (0): Continuous conversion
'       MAG_OPMODE_SINGLE (1): Single conversion
'       MAG_OPMODE_POWERDOWN (2): Power down
    curr_mode := 0
    readreg(MAG, core#CTRL_REG3_M, 1, @curr_mode)
    case mode
        MAG_OPMODE_CONT, MAG_OPMODE_SINGLE, MAG_OPMODE_POWERDOWN:
        other:
            return (curr_mode & core#MD_BITS)

    mode := ((curr_mode & core#MD_MASK) | mode)
    writereg(MAG, core#CTRL_REG3_M, 1, @mode)

PUB MagOverflow{}: flag
' Flag indicating magnetometer measurement has overflowed
'   Returns:
'       TRUE (-1) if measurement overflows sensor's internal range
'       FALSE (0) otherwise
    return (((magint{} >> core#MROI) & 1) == 1)

PUB MagPerf(mode): curr_mode
' Set magnetometer performance mode
'   Valid values:
'       MAG_PERF_LOW (0)
'       MAG_PERF_MED (1)
'       MAG_PERF_HIGH (2)
'       MAG_PERF_ULTRA (3)
'   Any other value polls the chip and returns the current setting
    readreg(MAG, core#CTRL_REG1_M, 1, @curr_mode.byte[0])
    readreg(MAG, core#CTRL_REG4_M, 1, @curr_mode.byte[1])

    case mode
        MAG_PERF_LOW, MAG_PERF_MED, MAG_PERF_HIGH, MAG_PERF_ULTRA:
        other:
            return (curr_mode.byte[0] >> core#OM) & core#OM_BITS

    curr_mode.byte[0] &= core#OM_MASK
    curr_mode.byte[0] := (curr_mode.byte[0] | (mode << core#OM))
    curr_mode.byte[1] &= core#OMZ_MASK
    curr_mode.byte[1] := (curr_mode.byte[1] | (mode << core#OMZ))

    writereg(MAG, core#CTRL_REG1_M, 1, @curr_mode.byte[0])
    writereg(MAG, core#CTRL_REG4_M, 1, @curr_mode.byte[1])

PUB MagScale(scale): curr_scl
' Set full scale of Magnetometer, in Gauss
'   Valid values: 4, 8, 12, 16
'   Any other value polls the chip and returns the current setting
    case(scale)
        4, 8, 12, 16:
            scale := lookdownz(scale: 4, 8, 12, 16)
            _mres := lookupz(scale: 0_000140, 0_000290, 0_000430, 0_000580)
            scale <<= core#FS_M
            writereg(MAG, core#CTRL_REG2_M, 1, @scale)
        other:
            curr_scl := 0
            readreg(MAG, core#CTRL_REG2_M, 1, @curr_scl)
            curr_scl := (curr_scl >> core#FS_M) & core#FS_M_BITS
            return lookupz(curr_scl: 4, 8, 12, 16)

PUB MagSelfTest(state): curr_state
' Enable on-chip magnetometer self-test
'   Valid values: TRUE (-1 or 1) or FALSE
'   Any other value polls the chip and returns the current setting
    return booleanchoice(MAG, core#CTRL_REG1_M, core#ST, core#ST_MASK, {
}   core#CTRL_REG1_M_MASK, state, 1)

PUB MagSoftreset{} | tmp
' Perform soft-test of magnetometer
    tmp := (1 << core#RE_BOOT) | (1 << core#SOFT_RST)
    tmp &= core#CTRL_REG2_M_MASK
    writereg(MAG, core#CTRL_REG2_M, 1, @tmp)
    time.msleep(10)

    tmp := 0                                    ' clear reset bit manually
    writereg(MAG, core#CTRL_REG2_M, 1, @tmp)    ' to come out of reset
    setspi3wiremode{}

PUB Temperature{}: temp
' Get temperature from chip
'   Returns: Temperature in hundredths of a degree in chosen scale
    return adc2temp(tempdata{})

PUB TempCompensation(enable): curr_setting
' Enable on-chip temperature compensation for magnetometer readings
'   Valid values: TRUE (-1 or 1) or FALSE
'   Any other value polls the chip and returns the current setting
    return booleanchoice(MAG, core#CTRL_REG1_M, core#TEMP_COMP, {
}   core#TEMP_COMP_MASK, core#CTRL_REG1_M, enable, 1)

PUB TempData{}: temp_adc
' Temperature ADC data
    temp_adc := 0
    readreg(XLG, core#OUT_TEMP_L, 2, @temp_adc)
    return ~~temp_adc

PUB TempDataReady{}: flag
' Temperature sensor new data available
'   Returns TRUE or FALSE
    readreg(XLG, core#STATUS_REG, 1, @flag)
    return (((flag >> core#TDA) & 1) == 1)

PUB TempScale(scale): curr_scl
' Set temperature scale used by Temperature method
'   Valid values:
'      *C (0): Celsius
'       F (1): Fahrenheit
'   Any other value returns the current setting
    case scale
        C, F:
            _temp_scale := scale
        other:
            return _temp_scale

PRI XLGDataBlockUpdate(state): curr_state
' Wait until both MSB and LSB of output registers are read before updating
'   Valid values:
'       FALSE (0): Continuous update
'       TRUE (1 or -1): Do not update until both MSB and LSB are read
'   Any other value polls the chip and returns the current setting
    return booleanchoice(XLG, core#CTRL_REG8, core#BDU, core#BDU_MASK,{
}   core#CTRL_REG8_MASK, state, 1)

PUB XLGDataRate(rate): curr_rate
' Set output data rate of accelerometer and gyroscope, in Hz
'   Valid values: 0 (power down), 14, 59, 119, 238, 476, 952
'   Any other value polls the chip and returns the current setting
    curr_rate := 0
    readreg(XLG, core#CTRL_REG1_G, 1, @curr_rate)
    case rate := lookdown(rate: 0, 14{.9}, 59{.5}, 119, 238, 476, 952)
        1..7:
            rate := (rate - 1) << core#ODR
        other:
            curr_rate := ((curr_rate >> core#ODR) & core#ODR_BITS) + 1
            return lookup(curr_rate: 0, 14{.9}, 59{.5}, 119, 238, 476, 952)

    rate := ((curr_rate & core#ODR_MASK) | rate)
    writereg(XLG, core#CTRL_REG1_G, 1, @rate)

PUB XLGIntActiveState(state): curr_state
' Set active state for interrupts from Accelerometer and Gyroscope
'   Valid values: ACTIVE_HIGH (0) - active high, ACTIVE_LOW (1) - active low
'   Any other value polls the chip and returns the current setting
    curr_state := 0
    readreg(XLG, core#CTRL_REG8, 1, @curr_state)
    case state
        ACTIVE_HIGH, ACTIVE_LOW:
            state := state << core#H_LACTIVE
        other:
            return ((curr_state >> core#H_LACTIVE) & 1)

    state := ((curr_state & core#H_LACTIVE_MASK) | state)
    writereg(XLG, core#CTRL_REG8, 1, @state)

PUB XLGSoftreset{} | tmp
' Perform soft-reset of accelerometer/gyroscope
    tmp := core#XLG_SW_RESET
    writereg(XLG, core#CTRL_REG8, 1, @tmp)
    time.msleep(10)

PRI adc2temp(temp_word): temp_cal
' Calculate temperature, using temperature word
'   Returns: temperature, in hundredths of a degree, in chosen scale
    temp_cal := ((temp_word * 10) / 16) + 2500
    case _temp_scale
        C:
            return
        F:
            return ((temp_cal * 90) / 50) + 32_00
        other:
            return FALSE

PRI addressAutoInc(state): curr_state
' Enable automatic address increment, for multibyte transfers (SPI and I2C)
'   Valid values: TRUE (-1 or 1), FALSE (0)
'   Any other value polls the chip and returns the current setting
    curr_state := 0
    readreg(XLG, core#CTRL_REG8, 1, @curr_state)
    case ||(state)
        0, 1:
            state := (||(state)) << core#IF_ADD_INC
        other:
            return (((curr_state >> core#IF_ADD_INC) & 1) == 1)

    state := ((curr_state & core#IF_ADD_INC) | state)
    writereg(XLG, core#CTRL_REG8, 1, @state)

PRI MagI2C(state): curr_state
' Enable Magnetometer I2C interface
'   Valid values: *TRUE (-1 or 1), FALSE (0)
'   Any other value polls the chip and returns the current setting
    return booleanchoice(MAG, core#CTRL_REG3_M, core#M_I2C_DIS, {
}   core#M_I2C_DIS_MASK, core#CTRL_REG3_M_MASK, state, -1)

PRI setSPI3WireMode{} | tmp
' Set SPI interface to 3-wire mode
    tmp := core#XLG_3WSPI
    writereg(XLG, core#CTRL_REG8, 1, @tmp)
    tmp := core#M_3WSPI
    writereg(MAG, core#CTRL_REG3_M, 1, @tmp)

PRI booleanChoice(device, reg_nr, field, fieldmask, regmask, choice, invertchoice): bool
' Reusable method for writing a field that is of a boolean or on-off type
'   device: AG or MAG
'   reg: register
'   field: field within register to modify
'   fieldmask: bitmask that clears the bits in the field being modified
'   regmask: bitmask to ensure only valid bits within the register can be modified
'   choice: the choice (TRUE/FALSE, 1/0)
'   invertchoice: whether to invert the boolean logic (1 for normal, -1 for inverted)
    bool := 0
    readreg(device, reg_nr, 1, @bool)
    case ||(choice)
        0, 1:
            choice := ||(choice * invertchoice) << field
        other:
            return ((((bool >> field) & 1) == 1) * invertchoice)

    bool &= fieldmask
    bool := (bool | choice) & regmask
    choice := ((bool & fieldmask) | choice) & regmask
    writereg(device, reg_nr, 1, @choice)

PRI readReg(device, reg_nr, nr_bytes, ptr_buff) | tmp
' Read from device
' Validate register - allow only registers that are
'   not 'reserved' (ST states reading should only be performed on registers listed in
'   their datasheet to guarantee proper behavior of the device)
    case device
        XLG:
            case reg_nr
                $04..$0D, $0F..$24, $26..$37:
                    outa[_CS_AG] := 0
                    spi.wr_byte(reg_nr | READ)
                    spi.rdblock_lsbf(ptr_buff, nr_bytes)
                    outa[_CS_AG] := 1
                other:
                    return
        MAG:
            case reg_nr
                $05..$0A, $0F, $20..$24, $27..$2D, $30..$33:
                    reg_nr |= READ
                    reg_nr |= MS
                    outa[_CS_M] := 0
                    spi.wr_byte(reg_nr)
                    spi.rdblock_lsbf(ptr_buff, nr_bytes)
                    outa[_CS_M] := 1
                other:
                    return

        other:
            return

PRI writeReg(device, reg_nr, nr_bytes, ptr_buff) | tmp
' Write byte to device
'   Validate register - allow only registers that are
'       writeable, and not 'reserved' (ST claims writing to these can
'       permanently damage the device)
    case device
        XLG:
            case reg_nr
                $04..$0D, $10..$13, $1E..$21, $23, $24, $2E, $30..$37:
                    outa[_CS_AG] := 0
                    spi.wr_byte(reg_nr)
                    spi.wrblock_lsbf(ptr_buff, nr_bytes)
                    outa[_CS_AG] := 1
                core#CTRL_REG8:
                    outa[_CS_AG] := 0
                    spi.wr_byte(reg_nr)
                    ' enforce 3-wire SPI mode
                    byte[ptr_buff][0] := byte[ptr_buff][0] | (1 << core#SIM)
                    spi.wrblock_lsbf(ptr_buff, nr_bytes)
                    outa[_CS_AG] := 1
                other:
                    return
        MAG:
            case reg_nr
                $05..$0A, $0F, $20, $21, $23, $24, $27..$2D, $30..$33:
                    reg_nr |= WRITE
                    reg_nr |= MS
                    outa[_CS_M] := 0
                    spi.wr_byte(reg_nr)
                    spi.wrblock_lsbf(ptr_buff, nr_bytes)
                    outa[_CS_M] := 1
                core#CTRL_REG3_M:
                    reg_nr |= WRITE
                    outa[_CS_M] := 0
                    spi.wr_byte(reg_nr)
                    ' enforce 3-wire SPI mode
                    byte[ptr_buff][0] := byte[ptr_buff][0] | (1 << core#M_SIM)
                    spi.wrblock_lsbf(ptr_buff, nr_bytes)
                    outa[_CS_M] := 1
                other:
                    return
        other:
            return
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
