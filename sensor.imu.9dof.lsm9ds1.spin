{
----------------------------------------------------------------------------------------------------
    Filename:       sensor.imu.9dof.lsm9ds1.spin
    Description:    Driver for the ST LSM9DS1 9DoF/3-axis IMU
    Author:         Jesse Burt
    Started:        Aug 12, 2017
    Updated:        Jan 21, 2024
    Copyright (c) 2024 - See end of file for terms of use.
---------------------------------------------------------------------------------------------------
}
#include "sensor.accel.common.spinh"
#include "sensor.gyroscope.common.spinh"
#include "sensor.magnetometer.common.spinh"
#include "sensor.temp.common.spinh"

CON

    { default I/O configuration - these can be overridden by the parent object }
    { /// I2C /// }
    SCL                     = 28
    SDA                     = 29
    I2C_FREQ                = 100_000
    I2C_ADDR                = 0

    { /// SPI /// }
    CS_AG                   = 0
    CS_M                    = 1
    SCK                     = 2
    MOSI                    = 3
    MISO                    = 4
    SPI_FREQ                = 1_000_000


    DEF_SCL                 = 28
    DEF_SDA                 = 29
    DEF_HZ                  = 100_000
    I2C_MAX_FREQ            = core#I2C_MAX_FREQ

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

' Axis-specific constants
    X_AXIS                  = 0
    Y_AXIS                  = 1
    Z_AXIS                  = 2
    ALL_AXIS                = 3

' Temperature scale constants
    C                       = 0
    F                       = 1

' Output data byte order
    LSBF                    = 0
    MSBF                    = 1

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

{ SPI? }
#ifdef LSM9DS1_SPI
{ decide: Bytecode SPI engine, or PASM? Default is PASM if BC isn't specified }
#ifdef LSM9DS1_SPI_BC
    spi : "com.spi.25khz.nocog"                       ' BC SPI engine
#else
    spi : "com.spi.1mhz"                          ' PASM SPI engine
#endif
#else
{ no, not SPI - default to I2C }
#define LSM9DS1_I2C
{ decide: Bytecode I2C engine, or PASM? Default is PASM if BC isn't specified }
#ifdef LSM9DS1_I2C_BC
    i2c : "com.i2c.nocog"                       ' BC I2C engine
#else
    i2c : "com.i2c"                             ' PASM I2C engine
#endif

#endif
    core    : "core.con.lsm9ds1"
    time    : "time"

VAR

    long _CS_AG, _CS_M
    byte _spi_wire_md
    byte _addr_bits

PUB null{}
' This is not a top-level object

#ifdef LSM9DS1_I2C
PUB start(): status
' Start the driver using default settings
    return startx(SCL, SDA, I2C_FREQ, I2C_ADDR)


PUB startx(SCL_PIN, SDA_PIN, I2C_HZ, ADDR_BITS): status
' Start using custom I/O pins
    if lookdown(SCL_PIN: 0..31) and lookdown(SDA_PIN: 0..31) and {
}   I2C_HZ =< core#I2C_MAX_FREQ
        if (status := i2c.init(SCL_PIN, SDA_PIN, I2C_HZ))
            time.usleep(core#TPOR)              ' startup time
            _addr_bits := (ADDR_BITS << 1)
            if (dev_id{} == core#WHOAMI_BOTH_RESP)
                xlg_soft_reset{}                  ' reset/initialize to
                mag_soft_reset{}                  ' POR defaults
                return status                   ' validate device
    ' if this point is reached, something above failed
    ' Double check I/O pin assignments, connections, power
    ' Lastly - make sure you have at least one free core/cog
    return FALSE

#elseifdef LSM9DS1_SPI

PUB start(): status
' Start the driver using default settings
    return startx(CS_AG, CS_M, SCK, MOSI, MISO)


PUB startx(CS_AG_PIN, CS_M_PIN, SCL_PIN, SDA_PIN, SDO_PIN): status
' Start using custom I/O pins
    if lookdown(SCL_PIN: 0..31) and lookdown(SDA_PIN: 0..31) and {
}   lookdown(CS_AG_PIN: 0..31) and lookdown(CS_M_PIN: 0..31) and {
}   lookdown(SDO_PIN: 0..31)
        if (status := spi.init(SCL_PIN, SDA_PIN, SDO_PIN, core#SPI_MODE))
            longmove(@_CS_AG, @CS_AG_PIN, 2)
            outa[_CS_AG] := 1                   ' make sure CS starts
            outa[_CS_M] := 1                    '   high
            dira[_CS_AG] := 1
            dira[_CS_M] := 1
            time.usleep(core#TPOR)              ' startup time

            { if SDA_PIN and SDO_PIN are the same, }
            { assume 3-wire SPI mode is wanted }
            if (SDA_PIN == SDO_PIN)
                spi_mode(3)
            else
                spi_mode(4)

            xlg_soft_reset{}                      ' reset/initialize to
            mag_soft_reset{}                      ' POR defaults

            if (dev_id{} == core#WHOAMI_BOTH_RESP)
                xlg_soft_reset{}                  ' reset/initialize to
                mag_soft_reset{}                  ' POR defaults
                return status                   ' validate device
    ' if this point is reached, something above failed
    ' Double check I/O pin assignments, connections, power
    ' Lastly - make sure you have at least one free core/cog
    return FALSE
#endif

PUB stop{}
' Stop the driver
#ifdef LSM9DS1_I2C
    i2c.deinit{}
#elseifdef LSM9DS1_SPI
    spi.deinit{}
#endif

PUB defaults{}
' Factory default settings
    xlg_soft_reset{}
    mag_soft_reset{}
    time.usleep(core#TPOR)

PUB preset_active{}
' Like defaults(), but
'   * enables output data (XL/G: 59Hz, Mag: 40Hz)
    xlg_soft_reset{}
    mag_soft_reset{}
    time.usleep(core#TPOR)

    addr_auto_inc_ena(TRUE)
#ifdef LSM9DS1_I2C
    mag_i2c_ena(TRUE)                           ' enable mag I2C interface
#elseifdef LSM9DS1_SPI
    mag_i2c_ena(FALSE)                          ' disable mag I2C interface
#endif
    blk_updt_ena{}
    xlg_data_rate(59)                           ' arbitrary
    mag_data_rate(40)                           '
    gyro_scale(245)                             ' already the POR defaults,
    accel_scale(2)                              ' but still need to call these
    mag_scale(4)                                ' to set scale factor hub vars
    mag_opmode(MAG_OPMODE_CONT)

PUB accel_axis_ena(mask): curr_mask
' Enable data output for accelerometer - per axis
'   Valid values: FALSE (0) or TRUE (1 or -1), for each axis
'   Any other value polls the chip and returns the current setting
    curr_mask := 0
    readreg(XLG, core#CTRL_REG5_XL, 1, @curr_mask)
    case mask
        %000..%111:
            mask <<= core#XEN_XL
        other:
            return ((curr_mask >> core#EN_XL) & core#EN_XL_BITS)

    mask := ((curr_mask & core#EN_XL_MASK) | mask)
    writereg(XLG, core#CTRL_REG5_XL, 1, @mask)

PUB accel_bias(x, y, z)
' Read accelerometer calibration offset values
'   x, y, z: pointers to copy offsets to
    long[x] := _abias[X_AXIS]
    long[y] := _abias[Y_AXIS]
    long[z] := _abias[Z_AXIS]

PUB accel_set_bias(x, y, z)
' Write accelerometer calibration offset values
'   Valid values:
'       -32768..32767 (clamped to range)
    _abias[X_AXIS] := -32768 #> x <# 32767
    _abias[Y_AXIS] := -32768 #> y <# 32767
    _abias[Z_AXIS] := -32768 #> z <# 32767

PUB accel_data(ax, ay, az) | tmp[2]
' Reads the accelerometer output registers
    readreg(XLG, core#OUT_X_L_XL, 6, @tmp)

    { accel ADC words are 15-bit signed - extend sign and }
    {   cancel out bias }
    long[ax] := ~~tmp.word[X_AXIS] - _abias[X_AXIS]
    long[ay] := ~~tmp.word[Y_AXIS] - _abias[Y_AXIS]
    long[az] := ~~tmp.word[Z_AXIS] - _abias[Z_AXIS]

PUB accel_data_byte_order(order): curr_order
' Byte order of accelerometer output data
'   Valid values: LSBF (0) or MSBF (1)
'   Any other value polls the chip and returns the current setting
'   NOTE: This setting also affects gyroscope output data
'       (hardware limitation)
    curr_order := 0
    readreg(XLG, core#CTRL_REG8, 1, @curr_order)
    case order
        LSBF, MSBF:
            order := order << core#BLE
        other:
            return ((curr_order >> core#BLE) & 1)

    order := ((curr_order & core#BLE_MASK) | order)
    writereg(XLG, core#CTRL_REG8, 1, @order)

PUB accel_data_rate(rate): curr_rate
' Set accelerometer output data rate, in Hz
'   NOTE: This is locked with the gyroscope output data rate
'       (hardware limitation)
    return xlg_data_rate(rate)

PUB accel_data_rdy{}: f | tmp
' Flag indicating new accelerometer data available
'   Returns TRUE or FALSE
    tmp := 0
    readreg(XLG, core#STATUS_REG, 1, @tmp)
    return (((tmp >> core#XLDA) & 1) == 1)

PUB accel_high_res_ena(state): curr_state
' Enable high resolution mode for accelerometer
'   Valid values: FALSE (0) or TRUE (1 or -1)
'   Any other value polls the chip and returns the current setting
    return bool_choice(XLG, core#CTRL_REG7_XL, core#HR, core#HR, {
}   core#CTRL_REG7_XL_MASK, state, 1)

PUB accel_int{}: int_src
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

PUB accel_int_duration(samples): curr_smp
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

PUB accel_int_hyst(state): curr_state
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

PUB accel_int_mask{}: mask
' Get accelerometer interrupt mask
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
    mask := 0
    readreg(XLG, core#INT_GEN_CFG_XL, 1, @mask)

PUB accel_int_set_mask(mask)
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
    mask &= %1111_1111
    writereg(XLG, core#INT_GEN_CFG_XL, 1, @mask)

PUB accel_int_thresh_x{}: thresh | ascl, lsb
' Get accelerometer interrupt threshold, X-axis
'   Returns: micro-g's
    ascl := (accel_scale(-2) * 1_000000)
    lsb := (ascl / 256)

    tmp := 0
    readreg(XLG, core#INT_GEN_THS_X_XL, 1, @tmp)
    return (tmp * lsb)                          ' scale to micro-g's

PUB accel_int_thresh_y{}: thresh | ascl, lsb
' Get accelerometer interrupt threshold, Y-axis
'   Returns: micro-g's
    ascl := (accel_scale(-2) * 1_000000)
    lsb := (ascl / 256)

    tmp := 0
    readreg(XLG, core#INT_GEN_THS_Y_XL, 1, @tmp)
    return (tmp * lsb)                          ' scale to micro-g's

PUB accel_int_thresh_z{}: thresh | ascl, lsb
' Get accelerometer interrupt threshold, Z-axis
'   Returns: micro-g's
    ascl := (accel_scale(-2) * 1_000000)
    lsb := (ascl / 256)

    tmp := 0
    readreg(XLG, core#INT_GEN_THS_Z_XL, 1, @tmp)
    return (tmp * lsb)                          ' scale to micro-g's

PUB accel_int_set_thresh_x(thresh) | ascl, lsb
' Set accelerometer interrupt thresholds per axis, in micro-g's (unsigned)
'   Valid values: 0..(full-scale * 1_000_000)
    ascl := (accel_scale(-2) * 1_000000)
    lsb := (ascl / 256)
    thresh := (0 #> thresh <# ascl) / lsb
    writereg(XLG, core#INT_GEN_THS_X_XL, 1, @thresh)

PUB accel_int_set_thresh_y(thresh) | ascl, lsb
' Set accelerometer interrupt thresholds per axis, in micro-g's (unsigned)
'   Valid values: 0..(full-scale * 1_000_000)
    ascl := (accel_scale(-2) * 1_000000)
    lsb := (ascl / 256)
    thresh := (0 #> thresh <# ascl) / lsb
    writereg(XLG, core#INT_GEN_THS_Y_XL, 1, @thresh)

PUB accel_int_set_thresh_z(thresh) | ascl, lsb
' Set accelerometer interrupt thresholds per axis, in micro-g's (unsigned)
'   Valid values: 0..(full-scale * 1_000_000)
    ascl := (accel_scale(-2) * 1_000000)
    lsb := (ascl / 256)
    thresh := (0 #> thresh <# ascl) / lsb
    writereg(XLG, core#INT_GEN_THS_Z_XL, 1, @thresh)

PUB accel_scale(scale): curr_scl
' Sets the full-scale range of the accelerometer, in g's
'   Valid values: 2, 4, 8, 16
'   Any other value polls the chip and returns the current setting
    curr_scl := 0
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

PUB dev_id{}: id | tmp[2]
' Read device identification
'   Returns: $683D
    id := 0
    readreg(XLG, core#WHO_AM_I_XG, 1, @tmp[1])
    readreg(MAG, core#WHO_AM_I_M, 1, @tmp[0])
    id.byte[1] := tmp[1]
    id.byte[0] := tmp[0]

PUB fifo_ena(state): curr_state
' Enable FIFO memory
'   Valid values: FALSE (0), TRUE(1 or -1)
'   Any other value polls the chip and returns the current setting
    return bool_choice(XLG, core#CTRL_REG9, core#FIFO_EN, core#FIFO_EN,{
}   core#CTRL_REG9_MASK, state, 1)

PUB fifo_full{}: flag
' FIFO Threshold status
'   Returns: FALSE (0): lower than threshold level, TRUE(-1): at or higher than threshold level
    flag := 0
    readreg(XLG, core#FIFO_SRC, 1, @flag)
    return (((flag >> core#FTH_STAT) & 1) == 1)

PUB fifo_mode(mode): curr_mode
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
    curr_mode := 0
    readreg(XLG, core#FIFO_CTRL, 1, @curr_mode)
    case mode
        FIFO_OFF, FIFO_THS, FIFO_CONT_TRIG, FIFO_OFF_TRIG, FIFO_CONT:
            mode <<= core#FMODE
        other:
            return (curr_mode >> core#FMODE) & core#FMODE_BITS

    mode := ((curr_mode & core#FMODE_MASK) | mode)
    writereg(XLG, core#FIFO_CTRL, 1, @mode)

PUB fifo_data_overrun{}: flag
' Flag indicating FIFO has overrun
'   Returns:
'       TRUE (-1) if at least one sample has been overwritten
'       FALSE (0) otherwise
    flag := 0
    readreg(XLG, core#FIFO_SRC, 1, @flag)
    return ((flag >> core#OVRN) & 1) == 1

PUB fifo_thresh(level): curr_lvl
' Set FIFO threshold level
'   Valid values: 0..31
'   Any other value polls the chip and returns the current setting
    curr_lvl := 0
    readreg(XLG, core#FIFO_CTRL, 1, @curr_lvl)
    case level
        0..31:
        other:
            return curr_lvl & core#FTH_BITS

    level := ((curr_lvl & core#FTH_MASK) | level)
    writereg(XLG, core#FIFO_CTRL, 1, @level)

PUB fifo_nr_unread{}: nr_samples
' Number of unread samples stored in FIFO
'   Returns: 0 (empty) .. 32
    nr_samples := 0
    readreg(XLG, core#FIFO_SRC, 1, @nr_samples)
    return nr_samples & core#FSS_BITS

PUB gyro_axis_ena(mask): curr_mask
' Enable gyroscope data output, per axis mask
'   Valid values: 0 or 1, for each axis
'   Any other value polls the chip and returns the current setting
    curr_mask := 0
    readreg(XLG, core#CTRL_REG4, 1, @curr_mask)
    case mask
        %000..%111:
            mask <<= core#XEN_G
        other:
            return (curr_mask >> core#XEN_G) & core#EN_G_BITS

    mask := ((curr_mask & core#EN_G_MASK) | mask)
    writereg(XLG, core#CTRL_REG4, 1, @mask)

PUB gyro_bias(x, y, z)
' Read gyroscope calibration offset values
'   x, y, z: pointers to copy offsets to
    long[x] := _gbias[X_AXIS]
    long[y] := _gbias[Y_AXIS]
    long[z] := _gbias[Z_AXIS]

PUB gyro_set_bias(x, y, z)
' Write gyroscope calibration offset values
'   Valid values:
'       -32768..32767
    _gbias[X_AXIS] := -32768 #> x <# 32767
    _gbias[Y_AXIS] := -32768 #> y <# 32767
    _gbias[Z_AXIS] := -32768 #> z <# 32767

PUB gyro_data(ptr_x, ptr_y, ptr_z) | tmp[2]
' Read gyroscope data
    longfill(@tmp, 0, 2)
    readreg(XLG, core#OUT_X_G_L, 6, @tmp)
    long[ptr_x] := ~~tmp.word[X_AXIS] - _gbias[X_AXIS]
    long[ptr_y] := ~~tmp.word[Y_AXIS] - _gbias[Y_AXIS]
    long[ptr_z] := ~~tmp.word[Z_AXIS] - _gbias[Z_AXIS]

PUB gyro_data_byte_order(order): curr_order
' Byte order of gyroscope output data
'   Valid values: LSBF (0) or MSBF (1)
'   Any other value polls the chip and returns the current setting
'   NOTE: This setting also affects accelerometer output data
'       (hardware limitation)
    return accel_data_byte_order(order)

PUB gyro_data_rate(rate): curr_rate
' Set gyroscope output data rate, in Hz
'   Valid values: 0, 15, 60, 119, 238, 476, 952
'   Any other value polls the chip and returns the current setting
'   NOTE: 0 powers down the gyroscope
'   NOTE: 15 and 60 are rounded up from the datasheet specifications of 14.9
'       and 59.5, respectively
    curr_rate := 0
    readreg(XLG, core#CTRL_REG1_G, 1, @curr_rate)
    case rate
        0, 15, 60, 119, 238, 476, 952:
            rate := lookdownz(rate: 0, 15, 60, 119, 238, 476, 952) << core#ODR
        other:
            curr_rate := ((curr_rate >> core#ODR) & core#ODR_BITS)
            return lookupz(curr_rate: 0, 15, 60, 119, 238, 476, 952)

    rate := ((curr_rate & core#ODR_MASK) | rate)
    writereg(XLG, core#CTRL_REG1_G, 1, @rate)

PUB gyro_data_rdy{}: flag
' Flag indicating new gyroscope data available
'   Returns TRUE or FALSE
    flag := 0
    readreg(XLG, core#STATUS_REG, 1, @flag)
    return (((flag >> core#GDA) & 1) == 1)

PUB gyro_hpf_freq(freq): curr_freq
' Set gyroscope high-pass filter cutoff frequency, in milli-Hz
'   Valid values: dependent on GyroDataRate(), see table below
'   Any other value polls the chip and returns the current setting
    curr_freq := 0
    readreg(XLG, core#CTRL_REG3_G, 1, @curr_freq)
    case gyro_data_rate(-2)
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

PUB gyro_inact_dur{}: dur
' Get gyroscope inactivity timer
    dur := 0
    readreg(XLG, core#ACT_DUR, 1, @dur)

PUB gyro_inact_set_dur(dur)
' Set gyroscope inactivity timer (use gyro_inact_sleep_ena() to define behavior on
'   inactivity)
'   Valid values: 0..255 (0 effectively disables the feature)
    writereg(XLG, core#ACT_DUR, 1, @dur)

PUB gyro_inact_thresh(thresh): curr_thr
' Set gyroscope inactivity threshold (use gyro_inact_sleep_ena() to define
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

PUB gyro_inact_sleep_ena(state): curr_state
' Enable gyroscope sleep mode when inactive (see gyro_inact_thresh())
'   Valid values:
'       FALSE (0): gyroscope powers down
'       TRUE (1 or -1) gyroscope enters sleep mode
'   Any other value polls the chip and returns the current setting
    return bool_choice(XLG, core#ACT_THS, core#SLP_ON_INACT, {
}   core#SLP_ON_INACT_MASK, core#ACT_THS_MASK, state, 1)

PUB gyro_int{}: int_src
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

PUB gyro_int_data_filt(mode): curr_mode
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

PUB gyro_int_duration(samples): curr_smp
' Set number of samples gyroscope data must be past threshold to be considered an interrupt
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

PUB gyro_int_hyst_ena(state): curr_state
' Enable gyroscope interrupt hysteresis
'   Valid values: TRUE (-1 or 1), FALSE (0)
'   Any other value polls the chip and returns the current setting
'   NOTE: The hysteresis used is equivalent to/set by the interrupt
'       duration time gyro_int_duration()
    curr_state := 0
    readreg(XLG, core#INT_GEN_DUR_G, 1, @curr_state)
    case ||(state)
        0, 1:
            state := ||(state) << core#WAIT_G
        other:
            return ((curr_state >> core#WAIT_G) & 1) == 1

    state := ((curr_state & core#WAIT_G_MASK) | state)
    writereg(XLG, core#INT_GEN_DUR_G, 1, @state)

PUB gyro_int_thresh_x{}: thresh | gscl, lsb
' Get gyroscope interrupt threshold, X-axis
'   Returns: micro-g's
    gscl := (gyro_scale(-2) * 1_000000)
    lsb := (gscl / 16384)

    thresh := 0
    readreg(XLG, core#INT_GEN_THS_XH_G, 2, @thresh)
    thresh := (((thresh & core#INT_G_BITS) << 1) ~> 1)
    return (~~thresh * lsb)

PUB gyro_int_thresh_y{}: thresh | gscl, lsb
' Get gyroscope interrupt threshold, X-axis
'   Returns: micro-g's
    gscl := (gyro_scale(-2) * 1_000000)
    lsb := (gscl / 16384)

    thresh := 0
    readreg(XLG, core#INT_GEN_THS_YH_G, 2, @thresh)
    thresh := (((thresh & core#INT_G_BITS) << 1) ~> 1)
    return (~~thresh * lsb)

PUB gyro_int_thresh_z{}: thresh | gscl, lsb
' Get gyroscope interrupt threshold, X-axis
'   Returns: micro-g's
    gscl := (gyro_scale(-2) * 1_000000)
    lsb := (gscl / 16384)

    thresh := 0
    readreg(XLG, core#INT_GEN_THS_ZH_G, 2, @thresh)
    thresh := (((thresh & core#INT_G_BITS) << 1) ~> 1)
    return (~~thresh * lsb)

PUB gyro_int_set_thresh_x(thresh) | gscl, lsb
' Set gyroscope interrupt thresholds per axis, in micro-g's (unsigned)
'   Valid values: 0..(full-scale * 1_000_000)
    gscl := (gyro_scale(-2) * 1_000000)
    lsb := (gscl / 16384)
    thresh := (0 #> thresh <# gscl) / lsb
    writereg(XLG, core#INT_GEN_THS_XH_G, 2, @thresh)

PUB gyro_int_set_thresh_y(thresh) | gscl, lsb
' Set gyroscope interrupt thresholds per axis, in micro-g's (unsigned)
'   Valid values: 0..(full-scale * 1_000_000)
    gscl := (gyro_scale(-2) * 1_000000)
    lsb := (gscl / 16384)
    thresh := (0 #> thresh <# gscl) / lsb
    writereg(XLG, core#INT_GEN_THS_YH_G, 2, @thresh)

PUB gyro_int_set_thresh_z(thresh) | gscl, lsb
' Set gyroscope interrupt thresholds per axis, in micro-g's (unsigned)
'   Valid values: 0..(full-scale * 1_000_000)
    gscl := (gyro_scale(-2) * 1_000000)
    lsb := (gscl / 16384)
    thresh := (0 #> thresh <# gscl) / lsb
    writereg(XLG, core#INT_GEN_THS_ZH_G, 2, @thresh)

PUB gyro_low_pwr_ena(state): curr_state
' Enable low-power mode
'   Valid values: FALSE (0), TRUE (1 or -1)
'   Any other value polls the chip and returns the current setting
    return bool_choice(XLG, core#CTRL_REG3_G, core#LP_MODE, {
}   core#LP_MODE_MASK, core#CTRL_REG3_G_MASK, state, 1)

PUB gyro_scale(scale): curr_scale
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

PUB gyro_sleep(state): curr_state
' Enable gyroscope sleep mode
'   Valid values: FALSE (0), TRUE (1 or -1)
'   Any other value polls the chip and returns the current setting
'   NOTE: If state, the gyro output will contain the last measured values
    return bool_choice(XLG, core#CTRL_REG9, core#SLP_G, core#SLP_G_MASK,{
}   core#CTRL_REG9_MASK, state, 1)

PUB int1_mask{}: mask
' Get interrupt enable mask on INT1 pin
'   Bits: 7..0 (1 = enabled, 0 = disabled)
'       7: gyroscope interrupt
'       6: accelerometer interrupt
'       5: FSS5 interrupt
'       4: data overrun interrupt
'       3: FIFO threshold interrupt
'       2: boot status interrupt
'       1: gyroscope data ready interrupt
'       0: accelerometer data ready interrupt
    mask := 0
    readreg(XLG, core#INT1_CTRL, 1, @mask)

PUB int1_set_mask(mask)
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
    mask &= %1111_1111
    writereg(XLG, core#INT1_CTRL, 1, @mask)

PUB int2_mask{}: mask
' Get interrupt enable mask on INT2 pin
'   Bits: 7..0 (1 = enabled, 0 = disabled)
'       7: gyroscope interrupt
'       6: - N/A -
'       5: FSS5 interrupt
'       4: data overrun interrupt
'       3: FIFO threshold interrupt
'       2: boot status interrupt
'       1: gyroscope data ready interrupt
'       0: accelerometer data ready interrupt
    mask := 0
    readreg(XLG, core#INT2_CTRL, 1, @mask)

PUB int2_set_mask(mask)
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
    mask &= core#INT2_CTRL_MASK         ' mask off bit 6 (unused)
    writereg(XLG, core#INT2_CTRL, 1, @mask)

PUB interrupt{}: flag
' Flag indicating one or more interrupts asserted
'   Returns TRUE if one or more interrupts asserted, FALSE if not
    flag := 0
    readreg(XLG, core#INT_GEN_SRC_XL, 1, @flag)
    return (((flag >> core#IA_XL) & 1) == 1)

PUB int_inactivity{}: flag
' Flag indicating inactivity interrupt asserted
'   Returns TRUE if interrupt asserted, FALSE if not
    flag := 0
    readreg(XLG, core#STATUS_REG, 1, @flag)
    return (((flag >> core#INACT) & 1) == 1)

PUB mag_bias(x, y, z) | tmp[2]
' Read magnetometer calibration offset values
'   x, y, z: pointers to copy offsets to
    longfill(@tmp, 0, 2)
    readreg(MAG, core#OFFSET_X_REG_L_M, 6, @tmp)
    long[x] := ~~tmp.word[X_AXIS]
    long[y] := ~~tmp.word[Y_AXIS]
    long[z] := ~~tmp.word[Z_AXIS]

PUB mag_set_bias(x, y, z)
' Write magnetometer calibration offset values
'   Valid values:
'       -32768..32767 (clamped to range)
    x := -32768 #> x <# 32767
    y := -32768 #> y <# 32767
    z := -32768 #> z <# 32767

    writereg(MAG, core#OFFSET_X_REG_L_M, 2, @x)
    writereg(MAG, core#OFFSET_Y_REG_L_M, 2, @y)
    writereg(MAG, core#OFFSET_Z_REG_L_M, 2, @z)

PUB mag_data(ptr_x, ptr_y, ptr_z) | tmp[2]
' Read the magnetometer output registers
    longfill(@tmp, 0, 2)
    readreg(MAG, core#OUT_X_L_M, 6, @tmp)
    long[ptr_x] := ~~tmp.word[X_AXIS]           ' no offset correction
    long[ptr_y] := ~~tmp.word[Y_AXIS]           ' because the mag has
    long[ptr_z] := ~~tmp.word[Z_AXIS]           ' offset registers built-in

PUB mag_data_order(order): curr_order
' Byte order of magnetometer output data
'   Valid values: LSBF (0) or MSBF (1)
'   Any other value polls the chip and returns the current setting
    curr_order := 0
    readreg(MAG, core#CTRL_REG4_M, 1, @curr_order)
    case order
        LSBF, MSBF:
            order := order << core#BLE_M
        other:
            return ((curr_order >> core#BLE_M) & 1)

    order := ((curr_order & core#BLE_M_MASK) | order)
    writereg(MAG, core#CTRL_REG4_M, 1, @order)

PUB mag_data_overrun{}: status
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
    status := 0
    readreg(MAG, core#STATUS_REG_M, 1, @status)
    return ((status >> core#OVERRN) & core#OVERRN_BITS)

PUB mag_data_rate(rate): curr_rate
' Set magnetometer output data rate, in Hz
'   Valid values: 0 (0.625), 1 (1.250), 2 (2.5), 5, *10, 20, 40, 80
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

PUB mag_data_rdy{}: flag
' Flag indicating new magnetometer data ready
'   Returns: TRUE (-1) if data available, FALSE (0) otherwise
    flag := 0
    readreg(MAG, core#STATUS_REG_M, 1, @flag)
    return (((flag >> core#ZYXDA) & 1) == 1)

PUB mag_fast_read_ena(state): curr_state
' Enable reading of only the MSB of data to increase reading efficiency, at
'   the cost of precision and accuracy
'   Valid values: TRUE(-1 or 1), FALSE(0)
'   Any other value polls the chip and returns the current setting
    return bool_choice (MAG, core#CTRL_REG5_M, core#FAST_READ, {
}   core#FAST_READ_MASK, core#CTRL_REG5_M_MASK, state, 1)

PUB mag_int{}: intsrc
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
    intsrc := 0
    readreg(MAG, core#INT_SRC_M, 1, @intsrc)

PUB mag_int_polarity(state): curr_state
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

PUB mag_int_clear{}
' Clear magnetometer interrupts
'   NOTE: This is only required if MagIntsLatched() is set to TRUE
    mag_int{}

PUB mag_int_mask(mask): curr_mask
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

PUB mag_int_latch_ena(state): curr_state
' Latch interrupts asserted by the magnetometer
'   Valid values: TRUE (-1 or 1) or FALSE
'   Any other value polls the chip and returns the current setting
'   NOTE: If enabled, interrupts must be explicitly cleared using MagClearInt()
    return bool_choice(MAG, core#INT_CFG_M, core#IEL, core#IEL_MASK,{
}   core#INT_CFG_M, state, -1)

PUB mag_int_thresh{}: thresh
' Get magnetometer interrupt threshold
    thresh := 0
    readreg(MAG, core#INT_THS_L_M, 2, @thresh)

PUB mag_int_set_thresh(thresh)
' Set magnetometer interrupt threshold
'   Valid values: 0..32767 (clamped to range)
'   Any other value polls the chip and returns the current setting
'   NOTE: The set thresh is an absolute value and is compared to positive and
'       negative measurements alike
    thresh := 0 #> thresh <# 32767
    writereg(MAG, core#INT_THS_L_M, 2, @thresh)

PUB mag_low_pwr_ena(state): curr_state
' Enable magnetometer low-power mode
'   Valid values: TRUE (-1 or 1) or FALSE
'   Any other value polls the chip and returns the current setting
    return bool_choice(MAG, core#CTRL_REG3_M, core#LP, core#LP_MASK, {
}   core#CTRL_REG3_M_MASK, state, 1)

PUB mag_opmode(mode): curr_mode
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

PUB mag_overflow{}: flag
' Flag indicating magnetometer measurement has overflowed
'   Returns:
'       TRUE (-1) if measurement overflows sensor's internal range
'       FALSE (0) otherwise
    return (((mag_int{} >> core#MROI) & 1) == 1)

PUB mag_perf(mode): curr_mode
' Set magnetometer performance mode
'   Valid values:
'       MAG_PERF_LOW (0)
'       MAG_PERF_MED (1)
'       MAG_PERF_HIGH (2)
'       MAG_PERF_ULTRA (3)
'   Any other value polls the chip and returns the current setting
    curr_mode := 0
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

PUB mag_scale(scale): curr_scl
' Set full scale of magnetometer, in Gauss
'   Valid values: 4, 8, 12, 16
'   Any other value polls the chip and returns the current setting
    case(scale)
        4, 8, 12, 16:
            scale := lookdownz(scale: 4, 8, 12, 16)
            longfill(@_mres, lookupz(scale: 0_000140, 0_000290, 0_000430, 0_000580), MAG_DOF)
            scale <<= core#FS_M
            writereg(MAG, core#CTRL_REG2_M, 1, @scale)
        other:
            curr_scl := 0
            readreg(MAG, core#CTRL_REG2_M, 1, @curr_scl)
            curr_scl := (curr_scl >> core#FS_M) & core#FS_M_BITS
            return lookupz(curr_scl: 4, 8, 12, 16)

PUB mag_self_test(state): curr_state
' Enable on-chip magnetometer self-test
'   Valid values: TRUE (-1 or 1) or FALSE
'   Any other value polls the chip and returns the current setting
    return bool_choice(MAG, core#CTRL_REG1_M, core#ST, core#ST_MASK, {
}   core#CTRL_REG1_M_MASK, state, 1)

PUB mag_soft_reset{} | tmp
' Perform soft-test of magnetometer
    tmp := (1 << core#RE_BOOT) | (1 << core#SOFT_RST)
    tmp &= core#CTRL_REG2_M_MASK
    writereg(MAG, core#CTRL_REG2_M, 1, @tmp)
    time.msleep(10)

    tmp := 0                                    ' clear reset bit manually
    writereg(MAG, core#CTRL_REG2_M, 1, @tmp)    ' to come out of reset
    spi_mode(_spi_wire_md)

PUB temp_comp_ena(enable): curr_setting
' Enable on-chip temperature compensation for magnetometer readings
'   Valid values: TRUE (-1 or 1) or FALSE
'   Any other value polls the chip and returns the current setting
    return bool_choice(MAG, core#CTRL_REG1_M, core#TEMP_COMP, {
}   core#TEMP_COMP_MASK, core#CTRL_REG1_M, enable, 1)

PUB temp_data{}: temp_adc
' Temperature ADC data
    temp_adc := 0
    readreg(XLG, core#OUT_TEMP_L, 2, @temp_adc)
    return ~~temp_adc

PUB temp_data_rdy{}: flag
' Temperature sensor new data available
'   Returns TRUE or FALSE
    flag := 0
    readreg(XLG, core#STATUS_REG, 1, @flag)
    return (((flag >> core#TDA) & 1) == 1)

PUB temp_word2deg(adc_word): temp
' Convert temperature ADC word to degrees in chosen scale
    temp := ((adc_word * 10) / 16) + 2500
    case _temp_scale
        C:
            return
        F:
            return ((temp * 90) / 50) + 32_00
        other:
            return FALSE

PUB xlg_data_rate(rate): curr_rate
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

PUB xlg_int_polarity(state): curr_state
' Set active state for interrupts from accelerometer and gyroscope
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

PUB xlg_soft_reset{} | tmp
' Perform soft-reset of accelerometer/gyroscope
    tmp := core#XLG_SW_RESET | (1 << core#BOOT)
    writereg(XLG, core#CTRL_REG8, 1, @tmp)
    time.msleep(10)

PRI addr_auto_inc_ena(state): curr_state
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

    state := ((curr_state & core#IF_ADD_INC_MASK) | state)
    writereg(XLG, core#CTRL_REG8, 1, @state)

PRI blk_updt_ena{} | tmp
' Wait to update the output data registers until both the MSB and LSB
'   are updated internally
    tmp := 0
    readreg(XLG, core#CTRL_REG8, 1, @tmp)
    tmp |= (1 << core#BDU)
    writereg(XLG, core#CTRL_REG8, 1, @tmp)

    tmp := 0
    readreg(MAG, core#CTRL_REG5_M, 1, @tmp)
    tmp |= (1 << core#BDU_M)
    writereg(MAG, core#CTRL_REG5_M, 1, @tmp)

PRI mag_i2c_ena(state): curr_state
' Enable magnetometer I2C interface
'   Valid values: *TRUE (-1 or 1), FALSE (0)
'   Any other value polls the chip and returns the current setting
    curr_state := 0
    readreg(MAG, core#CTRL_REG3_M, 1, @curr_state)
    case ||(state)
        0, 1:
            ' setting the M_I2C_DIS bit _disables_ I2C, so invert
            '   the value passed to this method
            state := (||(state) ^ 1) << core#M_I2C_DIS
        other:
            return ((((curr_state >> core#M_I2C_DIS) & 1) ^ 1) == 1)

    state := ((curr_state & core#M_I2C_DIS_MASK) | state)
    writereg(MAG, core#CTRL_REG3_M, 1, @state)

PRI spi_mode(mode)
' Set SPI interface mode
    case mode
        3:
            _spi_wire_md := 3
            mode := core#XLG_3WSPI              ' set the SIM bit
            writereg(XLG, core#CTRL_REG8, 1, @mode)
            mode := core#M_3WSPI
            writereg(MAG, core#CTRL_REG3_M, 1, @mode)
        4:
            _spi_wire_md := 4
            mode := 0                           ' clear all bits
            writereg(XLG, core#CTRL_REG8, 1, @mode)
            mode := 0
            writereg(MAG, core#CTRL_REG3_M, 1, @mode)
        other:
            return

PRI bool_choice(device, reg_nr, field, fieldmask, regmask, choice, invertchoice): bool
' Reusable method for writing a field that is of a boolean or on-off type
'   device: XLG or MAG
'   reg_nr: register
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

PRI readreg(device, reg_nr, nr_bytes, ptr_buff) | cmd_pkt
' Read from device
' Validate register - allow only registers that are
'   not 'reserved' (ST states reading should only be performed on registers listed in
'   their datasheet to guarantee proper behavior of the device)
    case device
        XLG:
            case reg_nr
                $15, $16, $17, $18, $1A, $1C, $28, $2A, $2C:
                $04..$0D, $0F..$14, $1E..$24, $26, $27, $2E..$37:
                other:
                    return
#ifdef LSM9DS1_I2C
            cmd_pkt.byte[0] := (core#SLAVE_ADDR_XLG | _addr_bits)
            cmd_pkt.byte[1] := reg_nr
            i2c.start{}
            i2c.wrblock_lsbf(@cmd_pkt, 2)
            i2c.start{}
            i2c.write(core#SLAVE_ADDR_XLG | _addr_bits | 1)
            i2c.rdblock_lsbf(ptr_buff, nr_bytes, i2c#NAK)
            i2c.stop{}
#elseifdef LSM9DS1_SPI
            outa[_CS_AG] := 0
            spi.wr_byte(reg_nr | READ)
            spi.rdblock_lsbf(ptr_buff, nr_bytes)
            outa[_CS_AG] := 1
#endif
        MAG:
            case reg_nr
                $05, $07, $09, $28, $2A, $2C:   ' multi-byte regs
#ifdef LSM9DS1_I2C
                    reg_nr |= core#MB_I2C       ' multi-byte trans. enabled
#elseifdef LSM9DS1_SPI
                    reg_nr |= core#MS_SPI       ' multi-byte trans. enabled
#endif
                $0F, $20..$24, $27, $30..$33:
                other:
                    return
#ifdef LSM9DS1_I2C
            cmd_pkt.byte[0] := (core#SLAVE_ADDR_MAG | (_addr_bits << 1))
            cmd_pkt.byte[1] := reg_nr
            i2c.start{}
            i2c.wrblock_lsbf(@cmd_pkt, 2)
            i2c.start{}
            i2c.write(core#SLAVE_ADDR_MAG | (_addr_bits << 1) | 1)
            i2c.rdblock_lsbf(ptr_buff, nr_bytes, i2c#NAK)
            i2c.stop{}
#elseifdef LSM9DS1_SPI
            outa[_CS_M] := 0
            spi.wr_byte(reg_nr | READ)
            spi.rdblock_lsbf(ptr_buff, nr_bytes)
            outa[_CS_M] := 1
#endif
        other:
            return

PRI writereg(device, reg_nr, nr_bytes, ptr_buff) | cmd_pkt
' Write byte to device
'   Validate register - allow only registers that are
'       writeable, and not 'reserved' (ST claims writing to these can
'       permanently damage the device)
    case device
        XLG:
            case reg_nr
#ifdef LSM9DS1_I2C
                $04..$0D, $10..$13, $1E..$24, $2E, $30..$37:
                    cmd_pkt.byte[0] := (core#SLAVE_ADDR_XLG | _addr_bits)
                    cmd_pkt.byte[1] := reg_nr
                    i2c.start{}
                    i2c.wrblock_lsbf(@cmd_pkt, 2)
                    i2c.wrblock_lsbf(ptr_buff, nr_bytes)
                    i2c.stop{}
                    return
#elseifdef LSM9DS1_SPI
                $04..$0D, $10..$13, $1E..$21, $23, $24, $2E, $30..$37:
                    outa[_CS_AG] := 0
                    spi.wr_byte(reg_nr)
                    spi.wrblock_lsbf(ptr_buff, nr_bytes)
                    outa[_CS_AG] := 1
                core#CTRL_REG8:
                    outa[_CS_AG] := 0
                    spi.wr_byte(reg_nr)

                    { enforce 3-wire SPI mode }
                    if (_spi_wire_md == 3)
                        byte[ptr_buff][0] := byte[ptr_buff][0] | (1 << core#SIM)
                    spi.wrblock_lsbf(ptr_buff, nr_bytes)
                    outa[_CS_AG] := 1
#endif
                other:
                    return
        MAG:
            case reg_nr
#ifdef LSM9DS1_I2C
                $05..$0A, $0F, $20, $21..$24, $27..$2D, $30..$33:
                    cmd_pkt.byte[0] := (core#SLAVE_ADDR_MAG | (_addr_bits << 1))
                    cmd_pkt.byte[1] := reg_nr | core#MB_I2C
                    i2c.start{}
                    i2c.wrblock_lsbf(@cmd_pkt, 2)
                    i2c.wrblock_lsbf(ptr_buff, nr_bytes)
                    i2c.stop
#elseifdef LSM9DS1_SPI
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

                    { enforce 3-wire SPI mode }
                    if (_spi_wire_md == 3)
                        byte[ptr_buff][0] := byte[ptr_buff][0] | (1 << core#M_SIM)
                    spi.wrblock_lsbf(ptr_buff, nr_bytes)
                    outa[_CS_M] := 1
#endif
                other:
                    return
        other:
            return


DAT
{
Copyright 2024 Jesse Burt

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

