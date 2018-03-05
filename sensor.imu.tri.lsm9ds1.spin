{
    --------------------------------------------
    Filename: sensor.imu.tri.lsm9ds1.spin
    Description: Driver for the ST LSM9DS1 3-axis IMU
      (ported from Parallax's Simple Library in C)
    Author: Jesse Burt
    Copyright (c) 2018
    See end of file for terms of use.
    --------------------------------------------
}
CON
' LSM9DS1 Register mappings to their symbolic names
  ACT_THS           = $04
  ACT_DUR           = $05
  INT_GEN_CFG_XL    = $06
  INT_GEN_THS_X_XL  = $07
  INT_GEN_THS_Y_XL  = $08
  INT_GEN_THS_Z_XL  = $09
  INT_GEN_DUR_XL    = $0A
  REFERENCE_G       = $0B
  INT1_CTRL         = $0C
  INT2_CTRL         = $0D
  WHO_AM_I_XG       = $0F
  CTRL_REG1_G       = $10
  CTRL_REG2_G       = $11
  CTRL_REG3_G       = $12
  ORIENT_CFG_G      = $13
  INT_GEN_SRC_G     = $14
  OUT_TEMP_L        = $15
  OUT_TEMP_H        = $16
  STATUS_REG_0      = $17
  OUT_X_L_G         = $18
  OUT_X_H_G         = $19
  OUT_Y_L_G         = $1A
  OUT_Y_H_G         = $1B
  OUT_Z_L_G         = $1C
  OUT_Z_H_G         = $1D
  CTRL_REG4         = $1E
  CTRL_REG5_XL      = $1F
  CTRL_REG6_XL      = $20
  CTRL_REG7_XL      = $21
  CTRL_REG8         = $22
  CTRL_REG9         = $23
  CTRL_REG10        = $24
  INT_GEN_SRC_XL    = $26
  STATUS_REG_1      = $27
  OUT_X_L_XL        = $28
  OUT_X_H_XL        = $29
  OUT_Y_L_XL        = $2A
  OUT_Y_H_XL        = $2B
  OUT_Z_L_XL        = $2C
  OUT_Z_H_XL        = $2D
  FIFO_CTRL         = $2E
  FIFO_SRC          = $2F
  INT_GEN_CFG_G     = $30
  INT_GEN_THS_XH_G  = $31
  INT_GEN_THS_XL_G  = $32
  INT_GEN_THS_YH_G  = $33
  INT_GEN_THS_YL_G  = $34
  INT_GEN_THS_ZH_G  = $35
  INT_GEN_THS_ZL_G  = $36
  INT_GEN_DUR_G     = $37
  OFFSET_X_REG_L_M  = $05
  OFFSET_X_REG_H_M  = $06
  OFFSET_Y_REG_L_M  = $07
  OFFSET_Y_REG_H_M  = $08
  OFFSET_Z_REG_L_M  = $09
  OFFSET_Z_REG_H_M  = $0A
  WHO_AM_I_M        = $0F
  CTRL_REG1_M       = $20
  CTRL_REG2_M       = $21
  CTRL_REG3_M       = $22
  CTRL_REG4_M       = $23
  CTRL_REG5_M       = $24
  STATUS_REG_M      = $27
  OUT_X_L_M         = $28
  OUT_X_H_M         = $29
  OUT_Y_L_M         = $2A
  OUT_Y_H_M         = $2B
  OUT_Z_L_M         = $2C
  OUT_Z_H_M         = $2D
  INT_CFG_M         = $30
  INT_SRC_M         = $30
  INT_THS_L_M       = $32
  INT_THS_H_M       = $33
  WHO_AM_I_AG_RSP   = $68
  WHO_AM_I_M_RSP    = $3D

  FIFO_OFF          = 0
  FIFO_THS          = 1
  FIFO_CONT_TRIGGER = 3
  FIFO_OFF_TRIGGER  = 4
  FIFO_CONT         = 5
  X_AXIS            = 0
  Y_AXIS            = 1
  Z_AXIS            = 2
  ALL_AXIS          = 3
  CELSIUS           = 0
  FAHRENHEIT        = 1
  KELVIN            = 2

OBJ

  spi:    "SPI_Asm"

VAR

  long __autoCalc

  long __settings_gyro_scale, __gRes, __gBias[3], __gBiasRaw[3]
  long __gx, __gy, __gz ' x, y, and z axis readings of the gyroscope
  long _gyro_pre

  long __settings_accel_scale, __aRes, __aBias[3], __aBiasRaw[3]
  long __ax, __ay, __az ' x, y, and z axis readings of the accelerometer
  long _accel_pre
  
  long __settings_mag_scale, __mRes, __mBias[3], __mBiasRaw[3]
  long __mx, __my, __mz ' x, y, and z axis readings of the magnetometer
  long _mag_pre

  long _scl_pin, _sdio_pin, _cs_ag_pin, _cs_m_pin, _int_ag_pin, _int_m_pin

PUB Null
'This is not a top-level object  

PUB Start(SCL_PIN, SDIO_PIN, CS_AG_PIN, CS_M_PIN, INT_AG_PIN, INT_M_PIN): okay

  ifnot okay := spi.start (10, 0)
    spi.stop
    abort FALSE
  _scl_pin := SCL_PIN
  _sdio_pin := SDIO_PIN
  _cs_ag_pin := CS_AG_PIN
  _cs_m_pin := CS_M_PIN
  _int_ag_pin := INT_AG_PIN
  _int_m_pin := INT_M_PIN

  dira[_scl_pin] := 0
  dira[_sdio_pin] := 0
  dira[_cs_ag_pin] := 0
  dira[_cs_m_pin] := 0
  dira[_int_ag_pin] := 0
  dira[_int_m_pin] := 0

  ' Initialize the IMU
  high(_cs_ag_pin)
  high(_cs_m_pin)
  low(_scl_pin)
  waitcnt(cnt + clkfreq / 1000)
  ' Set both the Accel/Gyro and Mag to 3-wire SPI mode
  WriteAGReg8 (CTRL_REG8, %0000_1100)
  WriteMReg8 (CTRL_REG3_M, %1000_0100)

  'Init Gyro
  WriteAGReg8 (CTRL_REG1_G, $C0)
  WriteAGReg8 (CTRL_REG2_G, $00)
  WriteAGReg8 (CTRL_REG3_G, $00)
  WriteAGReg8 (CTRL_REG4, $38)
  WriteAGReg8 (ORIENT_CFG_G, $00)

  'Init Accel
  WriteAGReg8 (CTRL_REG5_XL, $38)
  WriteAGReg8 (CTRL_REG6_XL, $C0)
  WriteAGReg8 (CTRL_REG7_XL, $00)

  'Init Mag
  WriteMReg8 (CTRL_REG2_M, $00)
  WriteMReg8 (CTRL_REG4_M, $0C)
  WriteMReg8 (CTRL_REG5_M, $00)

  'Set Scales

  setGyroScale(245)
  setAccelScale(2)
  setMagScale(8)

  SetPrecision (3)
  ' Once everything is initialized, check the WHO_AM_I registers
  ifnot whoAmI
    abort FALSE

PUB Stop

  spi.stop

PUB accelAvailable: status
'Polls the Accelerometer status register to check if new data is available.
  ReadAGReg (STATUS_REG_1, @status, 1)
  return (status & (1 << 0))

PUB calibrateAG | data[2], samples, ii, ax, ay, az, gx, gy, gz, aBiasRawTemp[3], gBiasRawTemp[3], tempF, tempS
' Calibrates the Accelerometer and Gyroscope on the LSM9DS1 IMU module.
  samples := 0
  ' Turn on FIFO and set threshold to 32 samples
  ReadAGReg (CTRL_REG9, @tempF, 1)
  tempF |= (1 << 1)
  WriteAGReg8 (CTRL_REG9, tempF)
  WriteAGReg8 (FIFO_CTRL, (((FIFO_THS & $7) << 5) | $1F))
  repeat while samples < $1F
    ReadAGReg (FIFO_SRC, @tempS, 1)
    samples := tempS.byte[0] & $3F
  repeat ii from 0 to samples-1 'while (ii < byte[@samples])
    ' Read the gyro data stored in the FIFO
    readGyro(@gx, @gy, @gz)
    gBiasRawTemp[0] += gx
    gBiasRawTemp[1] += gy
    gBiasRawTemp[2] += gz
    readAccel(@ax, @ay, @az)
    aBiasRawTemp[0] += ax
    aBiasRawTemp[1] += ay
    aBiasRawTemp[2] += az - __aRes ' Assumes sensor facing up!
  repeat ii from 0 to 2'while (ii < 3)
    __gBiasRaw[ii] := gBiasRawTemp[ii] / samples
    __gBias[ii] := (__gBiasRaw[ii]) / __gRes
    __aBiasRaw[ii] := aBiasRawTemp[ii] / samples
    __aBias[ii] := __aBiasRaw[ii] / __aRes
  __autoCalc := 1
  'Disable FIFO
  ReadAGReg (CTRL_REG9, @tempF, 1)
  tempF &= !(1 << 1)
  WriteAGReg8 (CTRL_REG9, tempF)
  WriteAGReg8 (FIFO_CTRL, ((FIFO_OFF & $7) << 5))

PUB calibrateMag | i, j, k, mx, my, mz, magMin[3], magMax[3], magTemp[3], msb, lsb
' Calibrates the Magnetometer on the LSM9DS1 IMU module.
  repeat i from 0 to 128
    repeat while not magAvailable 'Wait until new data available
    readMag(@mx, @my, @mz)
    magTemp[0] := mx
    magTemp[1] := my
    magTemp[2] := mz
    repeat j from 0 to 2
      if (magTemp[j] > magMax[j])
        magMax[j] := magTemp[j]
      if (magTemp[j] < magMin[j])
        magMin[j] := magTemp[j]
  repeat j from 0 to 2
    __mBiasRaw[j] := (magMax[j] + magMin[j])/2
  repeat k from 0 to 2
    msb := (__mBiasRaw[k] & $FF00) >> 8
    lsb := __mBiasRaw[k] & $00FF
    WriteMReg8 (OFFSET_X_REG_L_M + (2 * k), lsb)
    WriteMReg8 (OFFSET_X_REG_H_M + (2 * k), msb)

PUB clearAccelInterrupt | tempRegValue
' Clears out any interrupts set up on the Accelerometer and
' resets all Accelerometer interrupt registers to their default values.
  WriteAGReg8 (INT_GEN_THS_X_XL, $00)
  WriteAGReg8 (INT_GEN_THS_Y_XL, $00)
  WriteAGReg8 (INT_GEN_THS_Z_XL, $00)
  WriteAGReg8 (INT_GEN_CFG_XL, $00)
  WriteAGReg8 (INT_GEN_DUR_XL, $00)
  ReadAGReg (INT1_CTRL, @tempRegValue, 1)
  tempRegValue &= $BF
  WriteAGReg8 (INT1_CTRL, tempRegValue)


PUB clearGyroInterrupt | tempRegValue
' Clears out any interrupts set up on the Gyroscope and resets all Gyroscope interrupt registers to their default values.
  WriteAGReg8 (INT_GEN_THS_XH_G, $00)
  WriteAGReg8 (INT_GEN_THS_XL_G, $00)
  WriteAGReg8 (INT_GEN_THS_YH_G, $00)
  WriteAGReg8 (INT_GEN_THS_YL_G, $00)
  WriteAGReg8 (INT_GEN_THS_ZH_G, $00)
  WriteAGReg8 (INT_GEN_THS_ZL_G, $00)
  WriteAGReg8 (INT_GEN_CFG_G, $00)
  WriteAGReg8 (INT_GEN_DUR_G, $00)
  ReadAGReg (INT1_CTRL, @tempRegValue, 1)
  tempRegValue &= $7F
  WriteAGReg8 (INT1_CTRL, tempRegValue)

PUB clearMagInterrupt | tempRegValue 'UNTESTED
' Clears out any interrupts set up on the Magnetometer and
' resets all Magnetometer interrupt registers to their default values
  WriteMReg8 (INT_THS_L_M, $00)
  WriteMReg8 (INT_THS_H_M, $00)
  WriteMReg8 (INT_SRC_M, $00)
  WriteMReg8 (INT_CFG_M, $00)

PUB getAccelCalibration(axBias, ayBias, azBias) 'UNTESTED

  long[axBias] := __aBiasRaw[X_AXIS]
  long[ayBias] := __aBiasRaw[Y_AXIS]
  long[azBias] := __aBiasRaw[Z_AXIS]

PUB getAccelScale

  return __settings_accel_scale

PUB getGyroCalibration(gxBias, gyBias, gzBias) 'UNTESTED

  long[gxBias] := __gBiasRaw[X_AXIS]
  long[gyBias] := __gBiasRaw[Y_AXIS]
  long[gzBias] := __gBiasRaw[Z_AXIS]

PUB getGyroScale

  return __settings_gyro_scale

PUB getMagCalibration(mxBias, myBias, mzBias) 'UNTESTED

  long[mxBias] := __mBiasRaw[X_AXIS]
  long[myBias] := __mBiasRaw[Y_AXIS]
  long[mzBias] := __mBiasRaw[Z_AXIS]

PUB getMagScale

  return __settings_mag_scale

PUB gyroAvailable | status
' Polls the Gyroscope status register to check if new data is available
  ReadAGReg (STATUS_REG_1, @status, 1)
  return ((status & (1 << 1)) >> 1)

PUB whoAmI: whoAmICombined | mTest, xgTest

  ReadMReg (WHO_AM_I_M, @mTest, 1)
  ReadAGReg (WHO_AM_I_XG, @xgTest, 1)
  mTest &= $FF
  xgTest &= $FF
  whoAmICombined := ((xgTest << 8) | mTest) & $FFFF

PUB whoAmI_M: resp_m

  ReadMReg (WHO_AM_I_M, @resp_m, 1)
  resp_m &= $FF

PUB whoAmI_AG: resp_ag

  ReadAGReg (WHO_AM_I_XG, @resp_ag, 1)
  resp_ag &= $FF

PUB magAvailable | status
' Polls the Magnetometer status register to check if new data is available.
  ReadMReg (STATUS_REG_M, @status, 1)
  return ((status & (1 << 3)) >> 3)

PUB readAccel(ax, ay, az) | temp[2], tempX, tempY, tempZ
'Reads the Accelerometer output registers

' We'll read six bytes from the accelerometer into temp  , tempX, tempY, tempZ
  ReadAGReg (OUT_X_L_XL, @temp, 6)'reg, ptr, count)
  tempX := (temp.byte[1] << 8) | temp.byte[0] ' Store x-axis values into ax
  tempY := (temp.byte[3] << 8) | temp.byte[2] ' Store y-axis values into ay
  tempZ := (temp.byte[5] << 8) | temp.byte[4] ' Store z-axis values into az
  long[ax] := ~~tempX
  long[ay] := ~~tempY
  long[az] := ~~tempZ
  if (__autoCalc)
    long[ax] -= __aBiasRaw[X_AXIS]
    long[ay] -= __aBiasRaw[Y_AXIS]
    long[az] -= __aBiasRaw[Z_AXIS]

PUB readAccelCalculated(ax, ay, az) | tempX, tempY, tempZ, scale
' Reads the Accelerometer output registers and scales the outputs to milli-g's (1 g = 9.8 m/s/s)
  readAccel(@tempX, @tempY, @tempZ)
  long[ax] := (tempX*_accel_pre)/(__aRes)
  long[ay] := (tempY*_accel_pre)/(__aRes)
  long[az] := (tempZ*_accel_pre)/(__aRes)

PUB readGyro(gx, gy, gz) | temp[2], tempX, tempY, tempZ
' Reads the Gyroscope output registers.
' We'll read six bytes from the gyro into temp
  ReadAGReg (OUT_X_L_G, @temp, 6)
  tempX := (temp.byte[1] << 8) | temp.byte[0] ' Store x-axis values into gx
  tempY := (temp.byte[3] << 8) | temp.byte[2] ' Store y-axis values into gy
  tempZ := (temp.byte[5] << 8) | temp.byte[4] ' Store z-axis values into gz
  long[gx] := ~~tempX
  long[gy] := ~~tempY
  long[gz] := ~~tempZ
  if (__autoCalc)
    long[gx] -= __gBiasRaw[X_AXIS]
    long[gy] -= __gBiasRaw[Y_AXIS]
    long[gz] -= __gBiasRaw[Z_AXIS]

PUB readGyroCalculated(gx, gy, gz) | tempX, tempY, tempZ
' Reads the Gyroscope output registers and scales the outputs to degrees of rotation per second (DPS).
' Return the gyro raw reading times our pre-calculated DPS / (ADC tick):, tempX, tempY, tempZ
  readGyro(@tempX, @tempY, @tempZ)
  long[gx] := (tempX*_gyro_pre)/__gRes
  long[gy] := (tempY*_gyro_pre)/__gRes
  long[gz] := (tempZ*_gyro_pre)/__gRes

PUB readMag(mx, my, mz) | temp[2], tempX, tempY, tempZ
' Reads the Magnetometer output registers.
' We'll read six bytes from the mag into temp
  ReadMReg (OUT_X_L_M, @temp, 6)
  tempX := (temp.byte[1] << 8) | temp.byte[0] ' Store x-axis values into mx
  tempY := (temp.byte[3] << 8) | temp.byte[2] ' Store y-axis values into my
  tempZ := (temp.byte[5] << 8) | temp.byte[4] ' Store z-axis values into mz

  long[mx] := ~~tempX
  long[my] := ~~tempY
  long[mz] := ~~tempZ

PUB readMagCalculated(mx, my, mz) | tempX, tempY, tempZ
' Reads the Magnetometer output registers and scales the outputs to Gauss.
  readMag(@tempX, @tempY, @tempZ)
  long[mx] := (tempX*_mag_pre)/__mRes
  long[my] := (tempY*_mag_pre)/__mRes
  long[mz] := (tempZ*_mag_pre)/__mRes
  if (__autoCalc)
    long[mx] -= __mBiasRaw[X_AXIS]
    long[my] -= __mBiasRaw[Y_AXIS]
    long[mz] -= __mBiasRaw[Z_AXIS]

PUB readTemp(temperature) | temp[1], tempT
' We'll read two bytes from the temperature sensor into temp
' Read 2 bytes, beginning at OUT_TEMP_L
  ReadAGReg (OUT_TEMP_L, @temp, 2)
  tempT := (temp.byte[1] << 8) | temp.byte[0]
  long[temperature] := ~~tempT

PUB readTempCalculated(temperature, tempUnit) | tempTemp 'PARTIAL

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

PUB setAccelCalibration(axBias, ayBias, azBias)
' Manually set accelerometer calibration offset values
  __aBiasRaw[X_AXIS] := axBias
  __aBiasRaw[Y_AXIS] := ayBias
  __aBiasRaw[Z_AXIS] := azBias

PUB setAccelInterrupt(axis, threshold, duration, overUnder, andOr) | tempRegValue, accelThs, accelThsH, tempThs
'Configures the Accelerometer interrupt output to the INT_A/G pin.
  overUnder &= $01
  andOr &= $01
  tempRegValue := 0
  ReadAGReg (CTRL_REG4, @tempRegValue, 1)
  tempRegValue &= $FD
  WriteAGReg8 (CTRL_REG4, tempRegValue)
  ReadAGReg (INT_GEN_CFG_XL, @tempRegValue, 1)
  if andOr
    tempRegValue |= $80
  else
    tempRegValue &= $7F
  if (threshold < 0)
    threshold := -1 * threshold
  accelThs := 0
  tempThs := 0
  tempThs := (__aRes * threshold) >> 7
  accelThs := tempThs & $FF

  case(axis)
    X_AXIS:
      tempRegValue |= (1 <<(0 + overUnder))
      WriteAGReg8 (INT_GEN_THS_X_XL, accelThs)
    Y_AXIS:
      tempRegValue |= (1 <<(2 + overUnder))
      WriteAGReg8 (INT_GEN_THS_Y_XL, accelThs)
    Z_AXIS:
      tempRegValue |= (1 <<(4 + overUnder))
      WriteAGReg8 (INT_GEN_THS_Z_XL, accelThs)
    OTHER:
      WriteAGReg8 (INT_GEN_THS_X_XL, accelThs)
      WriteAGReg8 (INT_GEN_THS_Y_XL, accelThs)
      WriteAGReg8 (INT_GEN_THS_Z_XL, accelThs)
      tempRegValue |= (%00010101 << overUnder)
  WriteAGReg8 (INT_GEN_CFG_XL, tempRegValue)
  if (duration > 0)
    duration := $80 | (duration & $7F)
  else
    duration := $00
  WriteAGReg8 (INT_GEN_DUR_XL, duration)
  ReadAGReg (INT1_CTRL, @tempRegValue, 1)
  tempRegValue |= $40
  WriteAGReg8 (INT1_CTRL, tempRegValue)

PUB setAccelScale(aScl) | tempRegValue
' Sets the full-scale range of the Accelerometer.
' This function can be called to set the scale of the Accelerometer to 2, 4, 8, or 16 g's.

  if (aScl <> 2) and (aScl <> 4) and (aScl <> 8) and (aScl <> 16)
    aScl := 2
  __aRes := 32768/aScl
  __settings_accel_scale := aScl
  ' We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
  ReadAGReg (CTRL_REG6_XL, @tempRegValue, 1)
  ' Mask out accel scale bits:
  tempRegValue &= $E7
  case(aScl)
    4 :
      tempRegValue |= ($2 << 3)
    8 :
      tempRegValue |= ($3 << 3)
    16 :
      tempRegValue |= ($1 << 3)
    OTHER :
  WriteAGReg8 (CTRL_REG6_XL, tempRegValue)

PUB setGyroCalibration(gxBias, gyBias, gzBias)
' Manually set gyroscope calibration offset values
  __gBiasRaw[X_AXIS] := gxBias
  __gBiasRaw[Y_AXIS] := gyBias
  __gBiasRaw[Z_AXIS] := gzBias

PUB setGyroInterrupt(axis, threshold, duration, overUnder, andOr) | tempRegValue, gyroThs, gyroThsH, gyroThsL
' Configures the Gyroscope interrupt output to the INT_A/G pin.
  overUnder &= $01
  tempRegValue := 0
  ReadAGReg (CTRL_REG4, @tempRegValue, 1)
  tempRegValue &= $FD
  WriteAGReg8 (CTRL_REG4, tempRegValue)
  WriteAGReg8 (CTRL_REG4, tempRegValue)
  ReadAGReg (INT_GEN_CFG_G, @tempRegValue, 1)
  if andOr
    tempRegValue |= $80
  else
    tempRegValue &= $7F
  gyroThs := 0
  gyroThsH := 0
  gyroThsL := 0
  gyroThs := __gRes * threshold

  if gyroThs > 16383
    gyroThs := 16383
  if gyroThs < -16384
    gyroThs := -16384
  gyroThsL := (gyroThs & $FF)
  gyroThsH := (gyroThs >> 8) & $7F

  case(axis)
    X_AXIS :
      tempRegValue |= (1 <<(0 + overUnder))
      WriteAGReg8 (INT_GEN_THS_XH_G, gyroThsH)
      WriteAGReg8 (INT_GEN_THS_XL_G, gyroThsL)
    Y_AXIS :
      tempRegValue |= (1 <<(2 + overUnder))
      WriteAGReg8 (INT_GEN_THS_YH_G, gyroThsH)
      WriteAGReg8 (INT_GEN_THS_YL_G, gyroThsL)
    Z_AXIS :
      tempRegValue |= (1 <<(4 + overUnder))
      WriteAGReg8 (INT_GEN_THS_ZH_G, gyroThsH)
      WriteAGReg8 (INT_GEN_THS_ZL_G, gyroThsL)
    OTHER :
      WriteAGReg8 (INT_GEN_THS_XH_G, gyroThsH)
      WriteAGReg8 (INT_GEN_THS_XL_G, gyroThsL)
      WriteAGReg8 (INT_GEN_THS_YH_G, gyroThsH)
      WriteAGReg8 (INT_GEN_THS_YL_G, gyroThsL)
      WriteAGReg8 (INT_GEN_THS_ZH_G, gyroThsH)
      WriteAGReg8 (INT_GEN_THS_ZL_G, gyroThsL)
      tempRegValue |= (%00010101 << overUnder)
  WriteAGReg8 (INT_GEN_CFG_G, tempRegValue)
  if (duration > 0)
    duration := $80 | (duration & $7F)
  else
    duration := $00
  WriteAGReg8 (INT_GEN_DUR_G, duration)
  ReadAGReg (INT1_CTRL, @tempRegValue, 1)
  tempRegValue |= $80
  WriteAGReg8 (INT1_CTRL, tempRegValue)

PUB setGyroScale(gScl) | ctrl1RegValue
' Sets the full-scale range of the Gyroscope.
  if ((gScl <> 245) and (gScl <> 500) and (gScl <> 2000))
    gScl := 245
  __settings_gyro_scale := gScl
  __gRes := 32768/gScl
  ' Read current value of CTRL_REG1_G:, ctrl1RegValue

  ReadAGReg (CTRL_REG1_G, @ctrl1RegValue, 1)
  ' Mask out scale bits (3 & 4):
  ctrl1RegValue &= $E7
  case(gScl)
    500 :
      ctrl1RegValue |= ($1 << 3)
    2000 :
      ctrl1RegValue |= ($3 << 3)
    OTHER :
  WriteAGReg8 (CTRL_REG1_G, ctrl1RegValue)

PUB setMagCalibration(mxBias, myBias, mzBias) | k, msb, lsb
' Manually set magnetometer calibration offset values
' (non-volatile)
  __mBiasRaw[X_AXIS] := mxBias
  __mBiasRaw[Y_AXIS] := myBias
  __mBiasRaw[Z_AXIS] := mzBias

  repeat k from 0 to 2
    msb := (__mBiasRaw[k] & $FF00) >> 8
    lsb := __mBiasRaw[k] & $00FF

    WriteMReg8(OFFSET_X_REG_L_M + (2 * k), lsb)
    WriteMReg8(OFFSET_X_REG_H_M + (2 * k), msb)

PUB setMagInterrupt(axis, threshold, lowHigh) | tempCfgValue, tempSrcValue, magThs, magThsL, magThsH 'PARTIAL

  lowHigh &= $01
  tempCfgValue := $00
  tempCfgValue |= (lowHigh << 2)
  tempCfgValue |= $03
  tempSrcValue := $00
  magThs := 0
  magThsL := 0
  magThsH := 0
  magThs := __mRes * threshold

  if (magThs < 0)
    magThs := -1 * magThs
  if (magThs > 32767)
    magThs := 32767
  magThsL := magThs & $FF
  magThsH := (magThs >> 8) & $7F
  WriteMReg8(INT_THS_L_M, magThsL)
  WriteMReg8(INT_THS_H_M, magThsH)
  case axis
    X_AXIS :
      tempCfgValue |= ((1 << 7) | 2)
    Y_AXIS :
      tempCfgValue |= ((1 << 6) | 2)
    Z_AXIS :
      tempCfgValue |= ((1 << 5) | 2)
    OTHER :
      tempCfgValue |= (%11100010)
  WriteMReg8(INT_CFG_M, tempCfgValue)

PUB setMagScale(mScl) | temp
' Set the full-scale range of the magnetometer
'  if (mScl <> 4) and (mScl <> 8) and (mScl <> 12) and (mScl <> 16)
'    mScl := 4      ' Don't think this IF is needed...
'                   ...seems it's validated by the CASE block below
  ' We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:, temp
  ReadMReg (CTRL_REG2_M, @temp, 1)
  ' Then mask out the mag scale bits:
  temp &= $FF ^($3 << 5)
  case(mScl)
    8:
      temp |= ($1 << 5)
      __settings_mag_scale := 8
      __mRes := 3448'.28
    12:
      temp |= ($2 << 5)
      __settings_mag_scale := 12
      __mRes := 2298'.85
    16:
      temp |= ($3 << 5)
      __settings_mag_scale := 16
      __mRes := 1724'.14
    OTHER :
      __settings_mag_scale := 4
      __mRes := 6896'.55
  WriteMReg8(CTRL_REG2_M, temp)

PUB SetPrecision(digits)
'Set fixed-point math precision
'for ALL sensors
  SetPrecisionAccel (digits)
  SetPrecisionGyro (digits)
  SetPrecisionMag (digits)

PUB SetPrecisionAccel(digits) | scale_factor
'Set fixed-point math precision
  scale_factor := 1

  if digits < 0 or digits > 4
    digits := 3
  repeat digits
    scale_factor *= 10
  _accel_pre := scale_factor

PUB SetPrecisionGyro(digits) | scale_factor
'Set fixed-point math precision
  scale_factor := 1

  if digits < 0 or digits > 4
    digits := 3
  repeat digits
    scale_factor *= 10
  _gyro_pre := scale_factor

PUB SetPrecisionMag(digits) | scale_factor
'Set fixed-point math precision
  scale_factor := 1

  if digits < 0 or digits > 4
    digits := 3
  repeat digits
    scale_factor *= 10
  _mag_pre := scale_factor

PUB tempAvailable | status

  ReadAGReg (STATUS_REG_1, @status, 1)
  return ((status & (1 << 2)) >> 2)

PUB ReadAGReg(reg, ptr, count) | i
'Validate register and read word from Accel/Gyro device
'Allow only registers that are
'Not 'reserved' (ST states reading should only be performed on registers listed in
' their datasheet to guarantee proper behavior of the device)
  if count > 0
    case reg
      $04..$0D, $0F..$24, $26..$37:
        reg |= $80
        low(_cs_ag_pin)
        spi.shiftout(_sdio_pin, _scl_pin, spi#MSBFIRST, 8, reg)
        repeat i from 0 to count-1
          byte[ptr][i] := spi.shiftin(_sdio_pin, _scl_pin, spi#MSBPRE, 8)
        high(_cs_ag_pin)
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
        low(_cs_m_pin)
        spi.shiftout(_sdio_pin, _scl_pin, spi#MSBFIRST, 8, reg)
        repeat i from 0 to count-1
          byte[ptr][i] := spi.shiftin(_sdio_pin, _scl_pin, spi#MSBPRE, 8)
        high(_cs_m_pin)
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

{PUB WriteAGReg16(reg, writeword)
'Validate register and write word to Accel/Gyro device
'Allow only registers that are
'1. Writeable
'2. One byte of a word-sized register
'3. Not 'reserved' (ST claims writing to these can
' permanently damage the device)
  writeword &= $FFFF
  case reg
    $18, $1A, $1C, $31, $33, $35:
      SPIwriteWord(_cs_ag_pin, reg, writeword, 2)
    OTHER:
      return 0
}
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

{PUB WriteMReg16(reg, writeword)
'Validate register and write word to Magnetometer device
'Allow only registers that are
'1. Writeable
'2. One byte of a word-sized register
'3. Not 'reserved' (ST claims writing to these can
' permanently damage the device)
  writeword &= $FFFF
  case reg
    $05, $07, $09, $28, $2A, $2C, $32:
      SPIwriteBytes(_cs_m_pin, reg, writeword, 2)
    OTHER:
      return 0
}
PRI SPIwriteBytes(csPin, subAddress, data, count) | bytecnt
' SPI: Write byte _data_ to SPI device at _subAddress_ on Propeller I/O pin _csPin_
  low(csPin)
  spi.shiftout(_sdio_pin, _scl_pin, spi#MSBFIRST, 8, subAddress & $3F)
  repeat bytecnt from 0 to count-1
    spi.shiftout(_sdio_pin, _scl_pin, spi#MSBFIRST, 8, data.byte[bytecnt])
  high(csPin)

{PRI SPIwriteWord(csPin, subAddress, data)
' SPI: Write word _data_ to SPI device at _subAddress_ on Propeller I/O pin _csPin_
  low(csPin)
  spi.shiftout(_sdio_pin, _scl_pin, spi#MSBFIRST, 8, subAddress & $3F)
  spi.shiftout(_sdio_pin, _scl_pin, spi#MSBFIRST, 16, data)
  high(csPin)
}
PRI SPIreadBytes(csPin, subAddress, dest, count) | rAddress, i
' SPI: Read _count_ bytes from SPI device at _subAddress_ on Propeller I/O pin _csPin_ into pointer _dest_
  ' To indicate a read, set bit 0 (msb) of first byte to 1, rAddress
  rAddress := $80 | (subAddress & $3F)
  ' Mag SPI port is different. If we're reading multiple bytes, 
  ' set bit 1 to 1. The remaining six bytes are the address to be read
  if (csPin == _cs_m_pin) and count > 1
    rAddress |= $40
  low(csPin)
  spi.shiftout(_sdio_pin, _scl_pin, spi#MSBFIRST, 8, rAddress)
  repeat i from 0 to count-1
    byte[dest][i] := spi.shiftin(_sdio_pin, _scl_pin, spi#MSBPRE, 8)
  high(csPin)

PRI High(pin)
' Abbreviated way to bring an output pin high
    dira[pin]~~
    outa[pin]~~
    
PRI Low(pin)
' Abbreviated way to bring an output pin low
    dira[pin]~~
    outa[pin]~

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

