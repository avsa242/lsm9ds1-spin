{
    --------------------------------------------
    Filename: sensor.imu.tri.lsm9ds1.spin
    Author: Jesse Burt
    Description: Driver for the ST LSM9DS1 9DoF/3-axis IMU
    Copyright (c) 2019
    Started Aug 12, 2017
    Updated Feb 18, 2019
    See end of file for terms of use.
    --------------------------------------------
}

CON

    X_AXIS          = 0
    Y_AXIS          = 1
    Z_AXIS          = 2
    ALL_AXIS        = 3

    CELSIUS         = 0
    FAHRENHEIT      = 1
    KELVIN          = 2

    LITTLE          = 0
    BIG             = 1

    HIGH            = 0
    LOW             = 1

    FIFO_OFF        = core#FIFO_OFF
    FIFO_THS        = core#FIFO_THS
    FIFO_CONT_TRIG  = core#FIFO_CONT_TRIG
    FIFO_OFF_TRIG   = core#FIFO_OFF_TRIG
    FIFO_CONT       = core#FIFO_CONT

    AG              = 0
    MAG             = 1
    BOTH            = 2

    FP_SCALE        = 1000

OBJ

    spi     : "SPI_Asm"
    core    : "core.con.lsm9ds1"
    io      : "io"

VAR

    long _autoCalc
    long _gRes, _gBias[3], _gBiasRaw[3]
    long _aRes, _aBias[3], _aBiasRaw[3]
    long _mRes, _mBias[3], _mBiasRaw[3]
    long _scl_pin, _sdio_pin, _cs_ag_pin, _cs_m_pin, _int_ag_pin, _int_m_pin

PUB Null
'This is not a top-level object  

PUB Start(SCL_PIN, SDIO_PIN, CS_AG_PIN, CS_M_PIN, INT_AG_PIN, INT_M_PIN): okay

    if lookup(SCL_PIN: 0..31) and lookup(SDIO_PIN: 0..31) and lookup(CS_AG_PIN: 0..31) and lookup(CS_M_PIN: 0..31) and lookup(INT_AG_PIN: 0..31) and lookup(INT_M_PIN: 0..31)
        okay := spi.start (core#CLK_DELAY, core#CPOL)

        _scl_pin := SCL_PIN
        _sdio_pin := SDIO_PIN
        _cs_ag_pin := CS_AG_PIN
        _cs_m_pin := CS_M_PIN
        _int_ag_pin := INT_AG_PIN
        _int_m_pin := INT_M_PIN

        io.Input (_scl_pin)
        io.Input (_sdio_pin)
        io.Output (_cs_ag_pin)
        io.Output (_cs_m_pin)
        io.Input (_int_ag_pin)
        io.Input (_int_m_pin)

' Initialize the IMU
        io.High (_cs_ag_pin)
        io.High (_cs_m_pin)
        io.Low (_scl_pin)
        waitcnt(cnt + clkfreq / 1000)
' Set both the Accel/Gyro and Mag to 3-wire SPI mode
        WriteAGReg8 (core#CTRL_REG8, %0000_1100)
        WriteMReg8 (core#CTRL_REG3_M, %1000_0100)

' Once everything is initialized, check the WHO_AM_I registers
        if ID(BOTH) == core#WHOAMI_BOTH_RESP
            Defaults
            return okay
        Stop
        return FALSE

PUB Stop

    spi.stop

PUB Defaults

'Init Gyro
    WriteAGReg8 (core#CTRL_REG1_G, $C0)
    WriteAGReg8 (core#CTRL_REG2_G, $00)
    WriteAGReg8 (core#CTRL_REG3_G, $00)
    WriteAGReg8 (core#CTRL_REG4, $38)
    WriteAGReg8 (core#ORIENT_CFG_G, $00)

'Init Accel
    WriteAGReg8 (core#CTRL_REG5_XL, $38)
    WriteAGReg8 (core#CTRL_REG6_XL, $C0)
    WriteAGReg8 (core#CTRL_REG7_XL, $00)

'Init Mag
    WriteMReg8 (core#CTRL_REG2_M, $00)
    WriteMReg8 (core#CTRL_REG4_M, $0C)
    WriteMReg8 (core#CTRL_REG5_M, $00)

'Set Scales
    GyroScale(245)
    AccelScale(2)
    MagScale(4)

PUB AccelAvail | tmp
' Accelerometer sensor new data available
'   Returns TRUE or FALSE
    ReadAGReg (core#STATUS_REG, @tmp, 1)
    result := ((tmp >> core#FLD_XLDA) & %1) * TRUE

PUB AccelClearInt | temp, reg
' Clears out any interrupts set up on the Accelerometer and
' resets all Accelerometer interrupt registers to their default values.
    repeat reg from core#INT_GEN_CFG_XL to core#INT_GEN_DUR_XL
        WriteAGReg8 (reg, $00)
'    WriteAGReg8 (core#INT_GEN_THS_X_XL, $00)
'    WriteAGReg8 (core#INT_GEN_THS_Y_XL, $00)
'    WriteAGReg8 (core#INT_GEN_THS_Z_XL, $00)
'    WriteAGReg8 (core#INT_GEN_CFG_XL, $00)
'    WriteAGReg8 (core#INT_GEN_DUR_XL, $00)
    ReadAGReg (core#INT1_CTRL, @temp, 1)
    temp &= core#MASK_INT1_IG_XL
    WriteAGReg8 (core#INT1_CTRL, temp)

PUB AccelHighRes(enabled) | tmp
' Enable high resolution mode for accelerometer
'   Valid values: FALSE (0) or TRUE (1 or -1)
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#CTRL_REG7_XL, @tmp, 1)
    case ||enabled
        0, 1:
            enabled := enabled << core#FLD_HR
        OTHER:
            tmp := (tmp >> core#FLD_HR) & core#BITS_HR
            return tmp

    tmp &= core#MASK_HR
    tmp := (tmp | enabled) & core#CTRL_REG7_XL_MASK
    WriteAGReg8 (core#CTRL_REG7_XL, tmp)

PUB AccelOutEnable(x, y, z) | tmp, bits
' Enable data output for Accelerometer - per axis
'   Valid values: FALSE (0) or TRUE (1 or -1), for each axis
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#CTRL_REG5_XL, @tmp, 1)
    case bits := (||z << 2) | (||y << 1) | ||x
        %000..%111:
            bits <<= core#FLD_XEN_XL
        OTHER:
            tmp := (tmp >> core#FLD_XEN_XL) & core#BITS_EN_XL
            return tmp

    tmp &= core#MASK_EN_XL
    tmp := (tmp | bits) & core#CTRL_REG5_XL_MASK
    WriteAGReg8 (core#CTRL_REG5_XL, tmp)

PUB AccelScale(scale) | tmp
' Sets the full-scale range of the Accelerometer, in g's
'   Valid values: 2, 4, 8, 16
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#CTRL_REG6_XL, @tmp, 1)
    case scale
        2, 4, 8, 16:
            _aRes := 32768/scale
            scale := (lookdown(scale: 2, 16, 4, 8) - 1) << core#FLD_FS_XL
        OTHER:
            tmp := ((tmp >> core#FLD_FS_XL) & core#BITS_FS_XL) + 1
            return lookup(tmp: 2, 16, 4, 8)

    tmp &= core#MASK_FS_XL
    tmp := (tmp | scale) & core#CTRL_REG6_XL_MASK
    WriteAGReg8 (core#CTRL_REG6_XL, tmp)

PUB AccelSetCal(axBias, ayBias, azBias)
' Manually set accelerometer calibration offset values
    _aBiasRaw[X_AXIS] := axBias
    _aBiasRaw[Y_AXIS] := ayBias
    _aBiasRaw[Z_AXIS] := azBias

PUB AGDataRate(Hz) | tmp
' Set output data rate, in Hz, of accelerometer and gyroscope
'   Valid values: 0 (power down), 14, 59, 119, 238, 476, 952
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#CTRL_REG1_G, @tmp, 1)
    case Hz := lookdown(Hz: 0, 14{.9}, 59{.5}, 119, 238, 476, 952)
        1..7:
            Hz := (Hz - 1) << core#FLD_ODR
        OTHER:
            tmp := ((tmp >> core#FLD_ODR) & core#BITS_ODR) +1
            return lookup(tmp: 0, 14{.9}, 59{.5}, 119, 238, 476, 952)

    tmp &= core#MASK_ODR
    tmp := (tmp | Hz) & core#CTRL_REG1_G_MASK
    WriteAGReg8 (core#CTRL_REG1_G, tmp)

PUB BlockUpdate(enabled) | tmp 'XXX Make PRI? Doesn't seem like user-facing functionality
' Wait until both MSB and LSB of output registers are read before updating
'   Valid values: FALSE (0): Continuous update, TRUE (1 or -1): Do not update until both MSB and LSB are read
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#CTRL_REG8, @tmp, 1)
    case ||enabled
        0, 1:
            enabled := ||enabled << core#FLD_BDU
        OTHER:
            tmp := (tmp >> core#FLD_BDU) & %1
            return tmp

    tmp &= core#MASK_BDU
    tmp := (tmp | enabled) & core#CTRL_REG8_MASK
    WriteAGReg8 (core#CTRL_REG8, tmp)

PUB CalibrateAG | aBiasRawTemp[3], gBiasRawTemp[3], axis, ax, ay, az, gx, gy, gz, samples
' Calibrates the Accelerometer and Gyroscope
' Turn on FIFO and set threshold to 32 samples
    FIFO(TRUE)
    FIFOMode(FIFO_THS)
    FIFOThreshold (31)
    samples := FIFOThreshold (-2)
    repeat until FIFOFull
    _autoCalc := FALSE
    repeat axis from 0 to 2
        gBiasRawTemp[axis] := 0
        aBiasRawTemp[axis] := 0

    repeat samples
' Read the gyro and accel data stored in the FIFO
        ReadGyro(@gx, @gy, @gz)
        gBiasRawTemp[X_AXIS] += gx
        gBiasRawTemp[Y_AXIS] += gy
        gBiasRawTemp[Z_AXIS] += gz

        ReadAccel(@ax, @ay, @az)
        aBiasRawTemp[X_AXIS] += ax
        aBiasRawTemp[Y_AXIS] += ay
        aBiasRawTemp[Z_AXIS] += az - _aRes ' Assumes sensor facing up!

    repeat axis from 0 to 2
        _gBiasRaw[axis] := gBiasRawTemp[axis] / samples
        _gBias[axis] := (_gBiasRaw[axis]) / _gRes
        _aBiasRaw[axis] := aBiasRawTemp[axis] / samples
        _aBias[axis] := _aBiasRaw[axis] / _aRes

    _autoCalc := TRUE
    FIFO(FALSE)
    FIFOMode (FIFO_OFF)

PUB CalibrateMag(samples) | magMin[3], magMax[3], magTemp[3], axis, mx, my, mz, msb, lsb
' Calibrates the Magnetometer on the LSM9DS1 IMU module
    repeat samples
        repeat until MagAvail
        readMag(@mx, @my, @mz)
        magTemp[X_AXIS] := mx
        magTemp[Y_AXIS] := my
        magTemp[Z_AXIS] := mz
        repeat axis from X_AXIS to Z_AXIS
            if (magTemp[axis] > magMax[axis])
                magMax[axis] := magTemp[axis]
            if (magTemp[axis] < magMin[axis])
                magMin[axis] := magTemp[axis]

    repeat axis from X_AXIS to Z_AXIS
        _mBiasRaw[axis] := (magMax[axis] + magMin[axis]) / 2
        msb := (_mBiasRaw[axis] & $FF00) >> 8
        lsb := _mBiasRaw[axis] & $00FF
        WriteMReg8 (core#OFFSET_X_REG_L_M + (2 * axis), lsb)
        WriteMReg8 (core#OFFSET_X_REG_H_M + (2 * axis), msb)

PUB Endian(endianness) | tmp
' Choose byte order of data
'   Valid values: LITTLE (0) or BIG (1)
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#CTRL_REG8, @tmp, 1)
    case endianness
        LITTLE, BIG:
            endianness := endianness << core#FLD_BLE
        OTHER:
            tmp := (tmp >> core#FLD_BLE) & %1
            return tmp

    tmp &= core#MASK_BLE
    tmp := (tmp | endianness) & core#CTRL_REG8_MASK
    WriteAGReg8 (core#CTRL_REG8, tmp)

PUB FIFO(enabled) | tmp
' Enable FIFO memory
'   Valid values: FALSE (0), TRUE(1 or -1)
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#CTRL_REG9, @tmp, 1)
    case ||enabled
        0, 1:
            enabled := ||enabled << core#FLD_FIFO_EN
        OTHER:
            tmp := (tmp >> core#FLD_FIFO_EN) & core#BITS_FIFO_EN
            return tmp

    tmp &= core#MASK_FIFO_EN
    tmp := (tmp | enabled) & core#CTRL_REG9_MASK
    WriteAGReg8 (core#CTRL_REG9, tmp)

PUB FIFOFull | tmp
' FIFO Threshold status
'   Returns: FALSE (0): lower than threshold level, TRUE(-1): at or higher than threshold level
    ReadAGReg (core#FIFO_SRC, @result, 1)
    result := ((result >> core#FLD_FTH_STAT) & %1) * TRUE

PUB FIFOMode(mode) | tmp
' Set FIFO behavior
'   Valid values:
'       FIFO_OFF        (%000) - Bypass mode - FIFO off
'       FIFO_THS        (%001) - Stop collecting data when FIFO full
'       FIFO_CONT_TRIG  (%011) - Continuous mode until trigger is deasserted, then FIFO mode
'       FIFO_OFF_TRIG   (%100) - FIFO off until trigger is deasserted, then continuous mode
'       FIFO_CONT       (%110) - Continuous mode. If FIFO full, new sample overwrites older sample
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#FIFO_CTRL, @tmp, 1)
    case mode
        FIFO_OFF, FIFO_THS, FIFO_CONT_TRIG, FIFO_OFF_TRIG, FIFO_CONT:
            mode <<= core#FLD_FMODE
        OTHER:
            return (tmp >> core#FLD_FMODE) & core#BITS_FMODE
    tmp &= core#MASK_FMODE
    tmp := (tmp | mode) & core#FIFO_CTRL_MASK
    WriteAGReg8 (core#FIFO_CTRL, tmp)

PUB FIFOThreshold(level) | tmp
' Set FIFO threshold level
'   Valid values: 0..31
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#FIFO_CTRL, @tmp, 1)
    case level
        0..31:
        OTHER:
            return tmp & core#BITS_FTH

    tmp &= core#MASK_FTH
    tmp := (tmp | level) & core#FIFO_CTRL_MASK
    WriteAGReg8 (core#FIFO_CTRL, tmp)

PUB FIFOUnread
' Number of unread samples stored in FIFO
'   Returns: 0 (empty) .. 32
    ReadAGReg (core#FIFO_SRC, @result, 1)
    result &= core#BITS_FSS

PUB GyroActivityDur(duration) | tmp
' Set gyroscope inactivity timer (use GyroInactiveSleep to define behavior on inactivity)
'   Valid values: 0..255 (0 effectively disables the feature)
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#ACT_DUR, @tmp, 1)
    case duration
        0..255:
        OTHER:
            return tmp

    WriteAGReg8 (core#ACT_DUR, duration)

PUB GyroActivityThr(threshold) | tmp
' Set gyroscope inactivity threshold (use GyroInactiveSleep to define behavior on inactivity)
'   Valid values: 0..127 (0 effectively disables the feature)
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#ACT_THS, @tmp, 1)
    case threshold
        0..127:
        OTHER:
            return tmp & core#BITS_ACT_THS

    tmp &= core#MASK_ACT_THS
    tmp := (tmp | threshold) & core#ACT_THS_MASK
    WriteAGReg8 (core#ACT_THS, tmp)

PUB GyroAvail | tmp
' Gyroscope sensor new data available
'   Returns TRUE or FALSE
    ReadAGReg (core#STATUS_REG, @tmp, 1)
    result := ((tmp >> core#FLD_GDA) & %1) * TRUE

PUB GyroClearInt | temp, reg
' Clears out any interrupts set up on the Gyroscope and resets all Gyroscope interrupt registers to their default values.
    repeat reg from core#INT_GEN_CFG_G to core#INT_GEN_DUR_G
        WriteAGReg8 (reg, $00)
'    WriteAGReg8 (core#INT_GEN_THS_XH_G, $00)'XXX See if these are contiguous and if they are,
'    WriteAGReg8 (core#INT_GEN_THS_XL_G, $00)' iterate through them instead
'    WriteAGReg8 (core#INT_GEN_THS_YH_G, $00)
'    WriteAGReg8 (core#INT_GEN_THS_YL_G, $00)
'    WriteAGReg8 (core#INT_GEN_THS_ZH_G, $00)
'    WriteAGReg8 (core#INT_GEN_THS_ZL_G, $00)
'    WriteAGReg8 (core#INT_GEN_CFG_G, $00)
'    WriteAGReg8 (core#INT_GEN_DUR_G, $00)
    ReadAGReg (core#INT1_CTRL, @temp, 1)
    temp &= core#MASK_INT1_IG_G
    WriteAGReg8 (core#INT1_CTRL, temp)

PUB GyroInactiveSleep(enabled) | tmp
' Enable gyroscope sleep mode when inactive (see GyroActivityThr)
'   Valid values: FALSE (0): Gyroscope powers down, TRUE (1 or -1) Gyroscope enters sleep mode
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#ACT_THS, @tmp, 1)
    case ||enabled
        0, 1:
            enabled := ||enabled << core#FLD_SLEEP_ON_INACT
        OTHER:
            tmp := (tmp >> core#FLD_SLEEP_ON_INACT) & 1
            return tmp

    tmp &= core#MASK_SLEEP_ON_INACT
    tmp := (tmp | enabled) & core#ACT_THS_MASK
    WriteAGReg8 (core#ACT_THS, tmp)

PUB GyroOutEnable(x, y, z) | tmp, bits
' Enable data output for Gyroscope - per axis
'   Valid values: FALSE (0) or TRUE (1 or -1), for each axis
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#CTRL_REG4, @tmp, 1)
    case bits := (||z << 2) | (||y << 1) | ||x
        %000..%111:
            bits <<= core#FLD_XEN_G
        OTHER:
            tmp := (tmp >> core#FLD_XEN_G) & core#BITS_EN_G
            return tmp

    tmp &= core#MASK_EN_G
    tmp := (tmp | bits) & core#CTRL_REG4_MASK
    WriteAGReg8 (core#CTRL_REG4, tmp)

PUB GyroLowPower(enabled) | tmp
' Enable low-power mode
'   Valid values: FALSE (0), TRUE (1 or -1)
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#CTRL_REG3_G, @tmp, 1)
    case ||enabled
        0, 1:
            enabled := ||enabled << core#FLD_LP_MODE
        OTHER:
            tmp := (tmp >> core#FLD_LP_MODE) & %1
            return tmp

    tmp &= core#MASK_LP_MODE
    tmp := (tmp | enabled) & core#CTRL_REG3_G_MASK
    WriteAGReg8 (core#CTRL_REG3_G, tmp)

PUB GyroSetCal(gxBias, gyBias, gzBias)
' Manually set gyroscope calibration offset values
    _gBiasRaw[X_AXIS] := gxBias
    _gBiasRaw[Y_AXIS] := gyBias
    _gBiasRaw[Z_AXIS] := gzBias

PUB GyroSleep(enabled) | tmp
' Enable gyroscope sleep mode (last measured values frozen)
'   Valid values: FALSE (0), TRUE (1 or -1)
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#CTRL_REG9, @tmp, 1)
    case ||enabled
        0, 1:
            enabled := ||enabled << core#FLD_SLEEP_G
        OTHER:
            tmp := (tmp >> core#FLD_SLEEP_G) & core#BITS_SLEEP_G
            return tmp

    tmp &= core#MASK_SLEEP_G
    tmp := (tmp | enabled) & core#CTRL_REG9_MASK
    WriteAGReg8 (core#CTRL_REG9, tmp)

PUB GyroScale(scale) | tmp
' Set full scale of gyroscope output, in degrees per second (dps)
'   Valid values: 245, 500, 2000
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#CTRL_REG1_G, @tmp, 1)
    case scale
        245, 500, 2000:
            _gRes := 32768/scale
            scale := (lookdown(scale: 245, 500, 0, 2000) - 1) << core#FLD_FS
        OTHER:
            tmp := ((tmp >> core#FLD_FS) & core#BITS_FS) + 1
            return lookup(tmp: 245, 500, 0, 2000)

    tmp &= core#MASK_FS
    tmp := (tmp | scale) & core#CTRL_REG1_G_MASK
    WriteAGReg8 (core#CTRL_REG1_G, tmp)

PUB ID(sensor) | tmp
' Poll sensor for WHO_AM_I ID
'   Valid values: AG (0) or MAG (1)
'   Any other value returns both values OR'd together as a word (MSB=AG ID, LSB=M ID)
'   Returns
'       $68 if AG requested and response is valid
'       $3D if Mag requested and response is valid
'       $683D if both sensors requested and responses are valid
    case sensor
        AG:
            ReadAGReg (core#WHO_AM_I_XG, @result, 1)
        MAG:
            ReadMReg (core#WHO_AM_I_M, @result, 1)
        OTHER:
            ReadAGReg (core#WHO_AM_I_XG, @result, 1)
            ReadMReg (core#WHO_AM_I_M, @tmp, 1)
            result <<= 8
            result |= tmp

PUB Interrupt | tmp
' Interrupt active
'   Returns TRUE if one or more interrupts asserted, FALSE if not
    ReadAGReg (core#INT_GEN_SRC_XL, @tmp, 1)
    result := ((tmp >> core#FLD_IA_XL) & %1) * TRUE

PUB IntAccel | tmp
' Accelerometer interrupt output signal
'   Returns TRUE if interrupt asserted, FALSE if not
    ReadAGReg (core#STATUS_REG, @tmp, 1)
    result := ((tmp >> core#FLD_IG_XL) & %1) * TRUE

PUB IntGyro | tmp
' Accelerometer interrupt output signal
'   Returns TRUE if interrupt asserted, FALSE if not
    ReadAGReg (core#STATUS_REG, @tmp, 1)
    result := ((tmp >> core#FLD_IG_G) & %1) * TRUE

PUB IntInactivity | tmp
' Accelerometer interrupt output signal
'   Returns TRUE if interrupt asserted, FALSE if not
    ReadAGReg (core#STATUS_REG, @tmp, 1)
    result := ((tmp >> core#FLD_INACT) & %1) * TRUE

PUB IntLevel(active_state) | tmp
' Set active state for interrupts
'   Valid values: HIGH (0) - active high, LOW (1) - active low
'   Any other value polls the chip and returns the current setting
    ReadAGReg (core#CTRL_REG8, @tmp, 1)
    case active_state
        HIGH, LOW:
            active_state := active_state << core#FLD_H_LACTIVE
        OTHER:
            tmp := (tmp >> core#FLD_H_LACTIVE) & %1
            return tmp

    tmp &= core#MASK_H_LACTIVE
    tmp := (tmp | active_state) & core#CTRL_REG8_MASK
    WriteAGReg8 (core#CTRL_REG8, tmp)

PUB MagAvail
' Polls the Magnetometer status register to check if new data is available.
'   Returns TRUE if data available, FALSE if not
    ReadMReg (core#STATUS_REG_M, @result, 1)
    if result & core#BITS_DA
        result := TRUE
    else
        result := FALSE

PUB MagScale(scale) | tmp
' Set full scale of Magnetometer, in Gauss
'   Valid values: 4, 8, 12, 16
'   Any other value polls the chip and returns the current setting
    ReadMReg (core#CTRL_REG2_M, @tmp, 1)
    case(scale)
        4, 8, 12, 16:
            _mRes := lookup(scale/4: 6896{.55}, 3448{.28}, 2298{.85}, 1724{.14})
            scale := lookdownz(scale: 4, 8, 12, 16) << core#FLD_FS_M
        OTHER:
            return (tmp >> core#FLD_FS_M) & core#BITS_FS_M

    tmp &= core#MASK_FS_M
    tmp := (tmp | scale) & core#CTRL_REG2_M_MASK
    WriteMReg8(core#CTRL_REG2_M, tmp)

PUB MagSetCal(mxBias, myBias, mzBias) | axis, msb, lsb
' Manually set magnetometer calibration offset values
    _mBiasRaw[X_AXIS] := mxBias
    _mBiasRaw[Y_AXIS] := myBias
    _mBiasRaw[Z_AXIS] := mzBias

    repeat axis from X_AXIS to Z_AXIS
        msb := (_mBiasRaw[axis] & $FF00) >> 8
        lsb := _mBiasRaw[axis] & $00FF

        WriteMReg8(core#OFFSET_X_REG_L_M + (2 * axis), lsb)
        WriteMReg8(core#OFFSET_X_REG_H_M + (2 * axis), msb)

PUB ReadAccel(ax, ay, az) | temp[2]
'Reads the Accelerometer output registers
' We'll read six bytes from the accelerometer into temp
    ReadAGReg (core#OUT_X_L_XL, @temp, 6)

    long[ax] := ~~temp.word[0]
    long[ay] := ~~temp.word[1]
    long[az] := ~~temp.word[2]

    if (_autoCalc)
        long[ax] -= _aBiasRaw[X_AXIS]
        long[ay] -= _aBiasRaw[Y_AXIS]
        long[az] -= _aBiasRaw[Z_AXIS]

PUB ReadAccelCalculated(ax, ay, az) | tempX, tempY, tempZ
' Reads the Accelerometer output registers and scales the outputs to milli-g's (1 g = 9.8 m/s/s)
    readAccel(@tempX, @tempY, @tempZ)
    long[ax] := (tempX * FP_SCALE) / (_aRes)
    long[ay] := (tempY * FP_SCALE) / (_aRes)
    long[az] := (tempZ * FP_SCALE) / (_aRes)

PUB ReadGyro(gx, gy, gz) | temp[2]
' Reads the Gyroscope output registers.
' We'll read six bytes from the gyro into temp
    ReadAGReg (core#OUT_X_G_L, @temp, 6)

    long[gx] := ~~temp.word[0]
    long[gy] := ~~temp.word[1]
    long[gz] := ~~temp.word[2]

    if (_autoCalc)
        long[gx] -= _gBiasRaw[X_AXIS]
        long[gy] -= _gBiasRaw[Y_AXIS]
        long[gz] -= _gBiasRaw[Z_AXIS]

PUB ReadGyroCalculated(gx, gy, gz) | tempX, tempY, tempZ
' Reads the Gyroscope output registers and scales the outputs to milli-degrees of rotation per second (DPS).
    readGyro(@tempX, @tempY, @tempZ)
    long[gx] := (tempX * FP_SCALE) / _gRes
    long[gy] := (tempY * FP_SCALE) / _gRes
    long[gz] := (tempZ * FP_SCALE) / _gRes

PUB ReadMag(mx, my, mz) | temp[2]
' Reads the Magnetometer output registers.
' We'll read six bytes from the mag into temp
    ReadMReg (core#OUT_X_L_M, @temp, 6)

    long[mx] := ~~temp.word[0]
    long[my] := ~~temp.word[1]
    long[mz] := ~~temp.word[2]

PUB ReadMagCalculated(mx, my, mz) | tempX, tempY, tempZ
' Reads the Magnetometer output registers and scales the outputs to milli-Gauss.
    readMag(@tempX, @tempY, @tempZ)
    long[mx] := (tempX * FP_SCALE) / _mRes
    long[my] := (tempY * FP_SCALE) / _mRes
    long[mz] := (tempZ * FP_SCALE) / _mRes

PUB SWReset | tmp'XXX

    ReadAGReg (core#CTRL_REG8, @tmp, 1)
    tmp &= core#MASK_SW_RESET
    tmp := (tmp | %1) & core#CTRL_REG8_MASK
    WriteAGReg8 (core#CTRL_REG8, tmp)
    return tmp

PUB Temperature
' Get temperature from chip
'   Result is two's-complement
    ReadAGReg (core#OUT_TEMP_L, @result, 2)
    result &= $FFFF
    ~~result

PUB TempAvail | tmp
' Temperature sensor new data available
'   Returns TRUE or FALSE
    ReadAGReg (core#STATUS_REG, @tmp, 1)
    result := ((tmp >> core#FLD_TDA) & %1) * TRUE

'--- OLD CODE BELOW ---

PUB clearMagInterrupt | tempRegValue 'UNTESTED
' Clears out any interrupts set up on the Magnetometer and
' resets all Magnetometer interrupt registers to their default values
    WriteMReg8 (core#INT_THS_L_M, $00)
    WriteMReg8 (core#INT_THS_H_M, $00)
    WriteMReg8 (core#INT_SRC_M, $00)
    WriteMReg8 (core#INT_CFG_M, $00)

PUB getAccelCalibration(axBias, ayBias, azBias) 'UNTESTED

    long[axBias] := _aBiasRaw[X_AXIS]
    long[ayBias] := _aBiasRaw[Y_AXIS]
    long[azBias] := _aBiasRaw[Z_AXIS]

PUB getGyroCalibration(gxBias, gyBias, gzBias) 'UNTESTED

    long[gxBias] := _gBiasRaw[X_AXIS]
    long[gyBias] := _gBiasRaw[Y_AXIS]
    long[gzBias] := _gBiasRaw[Z_AXIS]

PUB getMagCalibration(mxBias, myBias, mzBias) 'UNTESTED

    long[mxBias] := _mBiasRaw[X_AXIS]
    long[myBias] := _mBiasRaw[Y_AXIS]
    long[mzBias] := _mBiasRaw[Z_AXIS]


{PUB readTemp(temperature) | temp[1], tempT
' We'll read two bytes from the temperature sensor into temp
' Read 2 bytes, beginning at OUT_TEMP_L
    ReadAGReg (core#OUT_TEMP_L, @temp, 2)
    tempT := (temp.byte[1] << 8) | temp.byte[0]
    long[temperature] := ~~tempT}

{PUB readTempCalculated(temperature, tempUnit) | tempTemp 'TODO: REVIEW (remove CELSIUS case - OTHER covers it)

    readTemp(@tempTemp)
    case tempUnit
        FAHRENHEIT:
            long[temperature] := ((tempTemp / 16) + 25) * 1800 + 32000
        CELSIUS:
            long[temperature] := ((tempTemp / 16) + 25) * 1000 '(tempTemp / 16.0) + 25.0
        KELVIN:
            long[temperature] := (((tempTemp/ 16) + 25) * 1000) + 273150'16) + 25) + 273.15
        OTHER:
            long[temperature] := (tempTemp / 16) + 25 * 1000'(tempTemp / 16.0) + 25.0
}

PUB setAccelInterrupt(axis, threshold, duration, overUnder, andOr) | tempRegValue, accelThs, accelThsH, tempThs
'Configures the Accelerometer interrupt output to the INT_A/G pin.
    overUnder &= $01
    andOr &= $01
    tempRegValue := 0
    ReadAGReg (core#CTRL_REG4, @tempRegValue, 1)
    tempRegValue &= $FD
    WriteAGReg8 (core#CTRL_REG4, tempRegValue)
    ReadAGReg (core#INT_GEN_CFG_XL, @tempRegValue, 1)
    if andOr
        tempRegValue |= $80
    else
        tempRegValue &= $7F
    if (threshold < 0)
        threshold := -1 * threshold
    accelThs := 0
    tempThs := 0
    tempThs := (_aRes * threshold) >> 7
    accelThs := tempThs & $FF

    case(axis)
        X_AXIS:
            tempRegValue |= (1 <<(0 + overUnder))
            WriteAGReg8 (core#INT_GEN_THS_X_XL, accelThs)
        Y_AXIS:
            tempRegValue |= (1 <<(2 + overUnder))
            WriteAGReg8 (core#INT_GEN_THS_Y_XL, accelThs)
        Z_AXIS:
            tempRegValue |= (1 <<(4 + overUnder))
            WriteAGReg8 (core#INT_GEN_THS_Z_XL, accelThs)
        OTHER:
            WriteAGReg8 (core#INT_GEN_THS_X_XL, accelThs)
            WriteAGReg8 (core#INT_GEN_THS_Y_XL, accelThs)
            WriteAGReg8 (core#INT_GEN_THS_Z_XL, accelThs)
            tempRegValue |= (%00010101 << overUnder)
    WriteAGReg8 (core#INT_GEN_CFG_XL, tempRegValue)
    if (duration > 0)
        duration := $80 | (duration & $7F)
    else
        duration := $00
    WriteAGReg8 (core#INT_GEN_DUR_XL, duration)
    ReadAGReg (core#INT1_CTRL, @tempRegValue, 1)
    tempRegValue |= $40
    WriteAGReg8 (core#INT1_CTRL, tempRegValue)

PUB setGyroInterrupt(axis, threshold, duration, overUnder, andOr) | tempRegValue, gyroThs, gyroThsH, gyroThsL
' Configures the Gyroscope interrupt output to the INT_A/G pin.
    overUnder &= $01
    tempRegValue := 0
    ReadAGReg (core#CTRL_REG4, @tempRegValue, 1)
    tempRegValue &= $FD
    WriteAGReg8 (core#CTRL_REG4, tempRegValue)
    WriteAGReg8 (core#CTRL_REG4, tempRegValue)
    ReadAGReg (core#INT_GEN_CFG_G, @tempRegValue, 1)
    if andOr
        tempRegValue |= $80
    else
        tempRegValue &= $7F
    gyroThs := 0
    gyroThsH := 0
    gyroThsL := 0
    gyroThs := _gRes * threshold 'TODO: REVIEW (use limit min/max operators and eliminate conditionals below?)

    if gyroThs > 16383
        gyroThs := 16383
    if gyroThs < -16384
        gyroThs := -16384
    gyroThsL := (gyroThs & $FF)
    gyroThsH := (gyroThs >> 8) & $7F

    case(axis)
        X_AXIS :
            tempRegValue |= (1 <<(0 + overUnder))
            WriteAGReg8 (core#INT_GEN_THS_XH_G, gyroThsH)
            WriteAGReg8 (core#INT_GEN_THS_XL_G, gyroThsL)
        Y_AXIS :
            tempRegValue |= (1 <<(2 + overUnder))
            WriteAGReg8 (core#INT_GEN_THS_YH_G, gyroThsH)
            WriteAGReg8 (core#INT_GEN_THS_YL_G, gyroThsL)
        Z_AXIS :
            tempRegValue |= (1 <<(4 + overUnder))
            WriteAGReg8 (core#INT_GEN_THS_ZH_G, gyroThsH)
            WriteAGReg8 (core#INT_GEN_THS_ZL_G, gyroThsL)
        OTHER :
            WriteAGReg8 (core#INT_GEN_THS_XH_G, gyroThsH)
            WriteAGReg8 (core#INT_GEN_THS_XL_G, gyroThsL)
            WriteAGReg8 (core#INT_GEN_THS_YH_G, gyroThsH)
            WriteAGReg8 (core#INT_GEN_THS_YL_G, gyroThsL)
            WriteAGReg8 (core#INT_GEN_THS_ZH_G, gyroThsH)
            WriteAGReg8 (core#INT_GEN_THS_ZL_G, gyroThsL)
            tempRegValue |= (%00010101 << overUnder)
    WriteAGReg8 (core#INT_GEN_CFG_G, tempRegValue)
    if (duration > 0)
        duration := $80 | (duration & $7F)
    else
        duration := $00
    WriteAGReg8 (core#INT_GEN_DUR_G, duration)
    ReadAGReg (core#INT1_CTRL, @tempRegValue, 1)
    tempRegValue |= $80
    WriteAGReg8 (core#INT1_CTRL, tempRegValue)

PUB setMagInterrupt(axis, threshold, lowHigh) | tempCfgValue, tempSrcValue, magThs, magThsL, magThsH 'PARTIAL

    lowHigh &= $01
    tempCfgValue := $00
    tempCfgValue |= (lowHigh << 2)
    tempCfgValue |= $03
    tempSrcValue := $00
    magThs := 0
    magThsL := 0
    magThsH := 0
    magThs := _mRes * threshold

    if (magThs < 0)
        magThs := -1 * magThs
    if (magThs > 32767)
        magThs := 32767
    magThsL := magThs & $FF
    magThsH := (magThs >> 8) & $7F
    WriteMReg8(core#INT_THS_L_M, magThsL)
    WriteMReg8(core#INT_THS_H_M, magThsH)
    case axis
        X_AXIS :
            tempCfgValue |= ((1 << 7) | 2)
        Y_AXIS :
            tempCfgValue |= ((1 << 6) | 2)
        Z_AXIS :
            tempCfgValue |= ((1 << 5) | 2)
        OTHER :
            tempCfgValue |= (%11100010)
    WriteMReg8(core#INT_CFG_M, tempCfgValue)

PUB ReadAGReg(reg, ptr, count) | i
'Validate register and read word from Accel/Gyro device
'Allow only registers that are
'Not 'reserved' (ST states reading should only be performed on registers listed in
' their datasheet to guarantee proper behavior of the device)
    if count > 0
        case reg
            $04..$0D, $0F..$24, $26..$37:
                reg |= $80
                io.Low(_cs_ag_pin)
                spi.shiftout(_sdio_pin, _scl_pin, spi#MSBFIRST, 8, reg)
                repeat i from 0 to count-1
                    byte[ptr][i] := spi.shiftin(_sdio_pin, _scl_pin, spi#MSBPRE, 8)
                io.High(_cs_ag_pin)
                return ptr
            OTHER:
                return 0
    else
        return 0

PUB ReadMReg(reg, ptr, count) | i
'Validate register and read word from Magnetometer device
'Allow only registers that are
'Not 'reserved' (ST states reading should only be performed on registers listed in
' their datasheet to guarantee proper behavior of the device)
    if count > 0
        case reg
            $05..$0A, $0F, $20..$24, $27..$2D, $30..$33:
                reg |= $80
                reg |= $40
                io.Low(_cs_m_pin)
                spi.shiftout(_sdio_pin, _scl_pin, spi#MSBFIRST, 8, reg)
                repeat i from 0 to count-1
                    byte[ptr][i] := spi.shiftin(_sdio_pin, _scl_pin, spi#MSBPRE, 8)
                io.High(_cs_m_pin)
                return ptr
            OTHER:
                return 0
    else
        return 0

PUB WriteAGReg8(reg, writebyte)
'Validate register and write byte to Accel/Gyro device
'Allow only registers that are
'1. Writeable
'2. Not 'reserved' (ST claims writing to these can
' permanently damage the device)
    writebyte &= $FF
    case reg
        $04..$0D, $10..$13, $1E..$24, $2E, $30..$37:
            SPIwriteBytes(_cs_ag_pin, reg, writebyte, 1)
        OTHER:
            return 0

PUB WriteMReg8(reg, writebyte)
'Validate register and write byte to Magnetometer device
'Allow only registers that are
'1. Writeable
'2. Not 'reserved' (ST claims writing to these can
' permanently damage the device)
    writebyte &= $FF
    case reg
        $05..$0A, $0F, $20..$24, $27..$2D, $30..$33:
            SPIwriteBytes(_cs_m_pin, reg, writebyte, 1)
        OTHER:
            return 0

PRI SPIwriteBytes(csPin, subAddress, data, count) | bytecnt
' SPI: Write byte _data_ to SPI device at _subAddress_ on Propeller I/O pin _csPin_
    io.Low (csPin)
    spi.shiftout(_sdio_pin, _scl_pin, core#MOSI_BITORDER, 8, subAddress & $3F)
    repeat bytecnt from 0 to count-1
        spi.shiftout(_sdio_pin, _scl_pin, core#MOSI_BITORDER, 8, data.byte[bytecnt])
    io.High (csPin)

PRI SPIreadBytes(csPin, subAddress, dest, count) | rAddress, i
' SPI: Read _count_ bytes from SPI device at _subAddress_ on Propeller I/O pin _csPin_ into pointer _dest_
  ' To indicate a read, set bit 0 (msb) of first byte to 1, rAddress
    rAddress := $80 | (subAddress & $3F)
' Mag SPI port is different. If we're reading multiple bytes,
' set bit 1 to 1. The remaining six bytes are the address to be read
    if (csPin == _cs_m_pin) and count > 1
        rAddress |= $40
    io.Low (csPin)
    spi.shiftout(_sdio_pin, _scl_pin, core#MOSI_BITORDER, 8, rAddress)
    repeat i from 0 to count-1
        byte[dest][i] := spi.shiftin(_sdio_pin, _scl_pin, core#MISO_BITORDER, 8)
    io.High(csPin)

{*
 * TERMS OF USE: MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 }

