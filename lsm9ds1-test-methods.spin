{{ This code is a test bed for the methods converted from the
    Simple Library for Parallax's LSM9DS1 IMU module. Once methods
    have been tested and appear to work like the C equivalents,
    they can be copied into the lsm9ds1-test_procedure.spin top-level
    object.
}}

CON

    _clkmode = xtal1 + pll16x
    _xinfreq = 5_000_000
   
CON
'' LSM9DS1 Register mappings to their symbolic names
  ACT_THS = $04
  ACT_DUR = $05
  INT_GEN_CFG_XL = $06
  INT_GEN_THS_X_XL = $07
  INT_GEN_THS_Y_XL = $08
  INT_GEN_THS_Z_XL = $09
  INT_GEN_DUR_XL = $0A
  REFERENCE_G = $0B
  INT1_CTRL = $0C
  INT2_CTRL = $0D
  WHO_AM_I_XG = $0F
  CTRL_REG1_G = $10
  CTRL_REG2_G = $11
  CTRL_REG3_G = $12
  ORIENT_CFG_G = $13
  INT_GEN_SRC_G = $14
  OUT_TEMP_L = $15
  OUT_TEMP_H = $16
  STATUS_REG_0 = $17
  OUT_X_L_G = $18
  OUT_X_H_G = $19
  OUT_Y_L_G = $1A
  OUT_Y_H_G = $1B
  OUT_Z_L_G = $1C
  OUT_Z_H_G = $1D
  CTRL_REG4 = $1E
  CTRL_REG5_XL = $1F
  CTRL_REG6_XL = $20
  CTRL_REG7_XL = $21
  CTRL_REG8 = $22
  CTRL_REG9 = $23
  CTRL_REG10 = $24
  INT_GEN_SRC_XL = $26
  STATUS_REG_1 = $27
  OUT_X_L_XL = $28
  OUT_X_H_XL = $29
  OUT_Y_L_XL = $2A
  OUT_Y_H_XL = $2B
  OUT_Z_L_XL = $2C
  OUT_Z_H_XL = $2D
  FIFO_CTRL = $2E
  FIFO_SRC = $2F
  INT_GEN_CFG_G = $30
  INT_GEN_THS_XH_G = $31
  INT_GEN_THS_XL_G = $32
  INT_GEN_THS_YH_G = $33
  INT_GEN_THS_YL_G = $34
  INT_GEN_THS_ZH_G = $35
  INT_GEN_THS_ZL_G = $36
  INT_GEN_DUR_G = $37
  OFFSET_X_REG_L_M = $05
  OFFSET_X_REG_H_M = $06
  OFFSET_Y_REG_L_M = $07
  OFFSET_Y_REG_H_M = $08
  OFFSET_Z_REG_L_M = $09
  OFFSET_Z_REG_H_M = $0A
  WHO_AM_I_M = $0F
  CTRL_REG1_M = $20
  CTRL_REG2_M = $21
  CTRL_REG3_M = $22
  CTRL_REG4_M = $23
  CTRL_REG5_M = $24
  STATUS_REG_M = $27
  OUT_X_L_M = $28
  OUT_X_H_M = $29
  OUT_Y_L_M = $2A
  OUT_Y_H_M = $2B
  OUT_Z_L_M = $2C
  OUT_Z_H_M = $2D
  INT_CFG_M = $30
  INT_SRC_M = $30
  INT_THS_L_M = $32
  INT_THS_H_M = $33
  WHO_AM_I_AG_RSP = $68
  WHO_AM_I_M_RSP = $3D
  FIFO_OFF = 0
  FIFO_THS = 1
  FIFO_CONT_TRIGGER = 3
  FIFO_OFF_TRIGGER = 4
  FIFO_CONT = 5
  X_AXIS = 0
  Y_AXIS = 1
  Z_AXIS = 2
  ALL_AXIS = 3
  CELSIUS = 0
  FAHRENHEIT = 1
  KELVIN = 2

  MSBFIRST = 1
  MSBPRE = 0
  LSBFIRST = 0
  LSBPRE = 1
  MSBPOST = 2
  LSBPOST = 3

  SCL_PIN = 5
  SDIO_PIN = 6
  CS_AG_PIN = 7
  CS_M_PIN = 8
  INT_AG_PIN = 9
  INT_M_PIN = 10
  VCC_PIN = 5
  GND_PIN = 4
  LEDRED = 3
  LEDYELLOW = 0
  LEDGREEN = 2
  LEDBLUE = 1
  BUTTON_PIN = 0

  XL_RAW = 0
  XL_CAL = 1
  GY_RAW = 2
  GY_CAL = 3
  M_RAW = 4
  M_CAL = 5
  
VAR

  long __autoCalc

  long __settings_gyro_scale, __gRes, __gBias[3], __gBiasRaw[3]
  long __gx, __gy, __gz

  long __settings_accel_scale, __aRes, __aBias[3], __aBiasRaw[3]
  long __ax, __ay, __az
  
  long __settings_mag_scale , __mRes, __mBias[3], __mBiasRaw[3]
  long __mx, __my, __mz

  long delay
  long stack[100]
  
OBJ
'' Either SPI_Asm or SPIN_Spin may be used.
    spi:    "SPI_Asm" 'Faster, but consumes 1 COG
'    spi:    "SPI_Spin" 'Slower, but uses 0 COGs
    ser:    "com.serial.terminal" '1 COG
    time:   "time" '0 COG
    math:   "math.float" '1 COG
    fs:     "string.float" '0 COG
    
PUB main | i, choice, testmode
  
  math.Start
  fs.SetPrecision (3)
  spi.start (10{For SPI_Asm: 1-129 works, for SPI_Spin: 7-129 works}, 0{Must be 0})
  ser.Start (115_200)
  delay := 30 '30ms delay for terminal logging

  dira[SCL_PIN] := 0
  dira[SDIO_PIN] := 0
  dira[CS_AG_PIN] := 0
  dira[CS_M_PIN] := 0
  dira[INT_AG_PIN] := 0
  dira[INT_M_PIN] := 0

  dira[LEDRED] := 1
  dira[LEDYELLOW] := 1
  dira[LEDGREEN] := 1
  dira[LEDBLUE] := 1


  ser.Str (string("WHOAMI: $"))
  ser.Hex (imu_init(SCL_PIN, SDIO_PIN, CS_AG_PIN, CS_M_PIN), 4)
  ser.NewLine
  repeat until ser.CharIn
  
  __autoCalc := TRUE
  
  testmode := M_CAL 'Which sensor to test, and what kind of output
  imu_clearGyroInterrupt
  imu_clearAccelInterrupt
  imu_clearMagInterrupt

  imu_setAccelScale(8) '2, 4, 8, 16
  imu_setGyroScale(500) '245, 500, 2000
  imu_setMagScale(8) '4, 8, 12, 16
  
  imu_setMagCalibration (0, 0, 0) 'To reset cal values stored on device
'  imu_setMagCalibration (11400, -6400, 16000)
'  imu_setMagCalibration (500, 0, 0)
'  imu_setAccelCalibration (-210, 0, -320)
'  imu_setGyroCalibration (-38, -20, -45)

  repeat
    choice := prompt(string("Calibrate magnetometer? "))
    case choice
      "y", "Y":

      OTHER:
        quit

    ser.Str (string("Calibrating magnetometer...", ser#NL))
    imu_calibrateMag
    repeat i from 0 to 2
      ser.Dec (__mBiasRaw[i])
      ser.Chars (32, 5)
    ser.NewLine
    choice := prompt(string("Calibrate again? "))
  
    case choice
      "y", "Y":
        imu_setMagCalibration (0, 0, 0)
        ser.NewLine
      OTHER:
        quit


  imu_calibrateAG

  
  
  case testmode
    XL_RAW:
      i:=cognew(printRawXL, @stack)
      repeat
        repeat while not imu_accelAvailable
          outa[LEDRED]~~
          outa[LEDGREEN]~
        outa[LEDRED]~
        outa[LEDGREEN]~~
        imu_readAccel(@__ax, @__ay, @__az)
    XL_CAL:
      i:=cognew(printCalcXL, @stack)
      repeat
        repeat while not imu_accelAvailable
          outa[LEDRED]~~
          outa[LEDGREEN]~
        outa[LEDRED]~
        outa[LEDGREEN]~~
        imu_readAccelCalculated(@__ax, @__ay, @__az)
    GY_RAW:
      i:=cognew(printRawG, @stack)
      repeat
        repeat while not imu_gyroAvailable
          outa[LEDRED]~~
          outa[LEDYELLOW]~
        outa[LEDRED]~
        outa[LEDYELLOW]~~
        imu_readGyro(@__gx, @__gy, @__gz)
    GY_CAL:
      i:=cognew(printCalcG, @stack)
      repeat
        repeat while not imu_gyroAvailable
          outa[LEDRED]~~
          outa[LEDYELLOW]~
        outa[LEDRED]~
        outa[LEDYELLOW]~~
        imu_readGyroCalculated(@__gx, @__gy, @__gz)
    M_RAW:
'      imu_calibrateMag
      i:=cognew(printRawM, @stack)
      repeat
        repeat while not imu_magAvailable
          outa[LEDRED]~~
          outa[LEDBLUE]~
        outa[LEDRED]~
        outa[LEDBLUE]~~
        imu_readMag(@__mx, @__my, @__mz)
    M_CAL:
'      imu_calibrateMag
      i:=cognew(printCalcM, @stack)
      repeat
        repeat while not imu_magAvailable
          outa[LEDRED]~~
          outa[LEDBLUE]~
        outa[LEDRED]~
        outa[LEDBLUE]~~
        imu_readMagCalculated(@__mx, @__my, @__mz)
    OTHER:
      ser.Str (string("Error: Invalid test mode specified.",13))
      led

PUB printRawM
'' Print raw magnetometer values
  repeat
    ser.Dec (__mx)
    repeat 5
      ser.Char (32)
    ser.dec (__my)
    repeat 5
      ser.Char (32)
    ser.dec (__mz)
    repeat 5
      ser.Char (32)
    ser.NewLine
    time.MSleep (delay)

PUB printCalcM
'' Print calculated magnetometer values, in Gauss
  repeat
    ser.str (fs.floattostring(__mx))
'    ser.Str (fs.floattostring(uTesla(__mx)))
'    ser.Str (string("uT"))
    repeat 5
      ser.Char (32)
    ser.str (fs.floattostring(__my))
'    ser.Str (fs.floattostring(uTesla(__my)))
'    ser.Str (string("uT"))
    repeat 5
      ser.Char (32)
    ser.str (fs.floattostring(__mz))
'    ser.Str (fs.floattostring(uTesla(__mz)))
'    ser.Str (string("uT"))
    repeat 5
      ser.Char (32)
    ser.NewLine
    time.MSleep (delay)

PUB printRawXL
'' Print raw accelerometer values
  repeat
    ser.Dec (__ax)
    repeat 5
      ser.Char (32)
    ser.dec (__ay)
    repeat 5
      ser.Char (32)
    ser.dec (__az)
    repeat 5
      ser.Char (32)
    ser.NewLine
    time.MSleep (delay)

PUB printCalcXL
'' Print calculated accelerometer values, in G's
  repeat
    ser.str (fs.floattostring(__ax))
    repeat 5
      ser.Char (32)
    ser.str (fs.floattostring(__ay))
    repeat 5
      ser.Char (32)
    ser.str (fs.floattostring(__az))
    repeat 5
      ser.Char (32)
    ser.NewLine
    time.MSleep (delay)

PUB thresh_XL
'' React to an accelerometer interrupt
'' previously set in the IMU

  dira[LEDRED]~~
  dira[LEDGREEN]~~
  
  repeat
    if ina[INT_AG_PIN]
      outa[LEDGREEN]~~
      outa[LEDRED]~
      ser.Str (fs.floattostring(__ax))
      ser.NewLine
      'time.MSleep (50)
    else
      outa[LEDGREEN]~
      outa[LEDRED]~~
    time.MSleep (delay)

PUB printRawG
'' Print raw Gyroscope values
  repeat
    ser.Dec (__gx)
    repeat 5
      ser.Char (32)
    ser.dec (__gy)
    repeat 5
      ser.Char (32)
    ser.dec (__gz)
    repeat 5
      ser.Char (32)
    ser.NewLine
    time.MSleep (delay)

PUB printCalcG
'' Print calculated Gyroscope values, in Degrees Per Second
  repeat
    ser.str (fs.floattostring(__gx))
    repeat 5
      ser.Char (32)
    ser.str (fs.floattostring(__gy))
    repeat 5
      ser.Char (32)
    ser.str (fs.floattostring(__gz))
    repeat 5
      ser.Char (32)
    ser.NewLine
    time.MSleep (delay)

PUB thresh_G
'' React to a gyroscope interrupt
'' previously set in the IMU

  dira[LEDRED]~~
  dira[LEDGREEN]~~
  
  repeat
    if ina[INT_AG_PIN]
      outa[LEDGREEN]~~
      outa[LEDRED]~
      ser.Str (fs.floattostring(__gy))
      ser.NewLine
      time.MSleep (50)
    else
      outa[LEDGREEN]~
      outa[LEDRED]~~

PUB imu_init(pinSCL, pinSDIO, pinAG, pinM) | xgTest, mTest, whoAmICombined 'WORKS
'' Initialize the IMU
  high(pinAG)
  high(pinM)
  low(pinSCL) ' Pin output state to low
  time.msleep(1)
  ' Set both the Accel/Gyro and Mag to 3-wire SPI mode
  imu_SPIwriteByte(pinAG, CTRL_REG8, %00001100)
  imu_SPIwriteByte(pinM, CTRL_REG3_M, %10000100)
  ' To verify communication, we can read from the WHO_AM_I register of
  ' each device. Store those in a variable so we can return them., xgTest, mTest
  imu_SPIreadBytes(pinM, WHO_AM_I_M, @mTest, 1) ' Read the gyro WHO_AM_I
  imu_SPIreadBytes(pinAG, WHO_AM_I_XG, @xgTest, 1) ' Read the accel/mag WHO_AM_I, whoAmICombined
  whoAmICombined := (xgTest << 8) | mTest
  if (whoAmICombined <> ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP))
    return 0
  'Init Gyro
  imu_SPIwriteByte(pinAG, CTRL_REG1_G, $C0)
  imu_SPIwriteByte(pinAG, CTRL_REG2_G, $00)
  imu_SPIwriteByte(pinAG, CTRL_REG3_G, $00)
  imu_SPIwriteByte(pinAG, CTRL_REG4, $38)
  imu_SPIwriteByte(pinAG, ORIENT_CFG_G, $00)
  'Init Accel
  imu_SPIwriteByte(pinAG, CTRL_REG5_XL, $38)
  imu_SPIwriteByte(pinAG, CTRL_REG6_XL, $C0)
  imu_SPIwriteByte(pinAG, CTRL_REG7_XL, $00)
  'Init Mag
  imu_SPIwriteByte(pinM, CTRL_REG2_M, $00)
  imu_SPIwriteByte(pinM, CTRL_REG4_M, $0C)
  imu_SPIwriteByte(pinM, CTRL_REG5_M, $00)
  'Set Scales
  imu_setGyroScale(500)
  imu_setAccelScale(8)
  imu_setMagScale(12)
  ' Once everything is initialized, return the WHO_AM_I registers we read:
  return whoAmICombined

PUB imu_setMagCalibration(mxBias, myBias, mzBias) | k, msb, lsb 'WORKS
'' Manually set magnetometer calibration offset values
'' (non-volatile)
  __mBiasRaw[X_AXIS] := mxBias
  __mBiasRaw[Y_AXIS] := myBias
  __mBiasRaw[Z_AXIS] := mzBias

  repeat k from 0 to 2
    msb := (__mBiasRaw[k] & $FF00) >> 8
    lsb := __mBiasRaw[k] & $00FF
    imu_SPIwriteByte(CS_M_PIN, OFFSET_X_REG_L_M + (2 * k), lsb)
    imu_SPIwriteByte(CS_M_PIN, OFFSET_X_REG_H_M + (2 * k), msb)

PUB imu_setAccelCalibration(axBias, ayBias, azBias) 'WORKS
'' Manually set accelerometer calibration offset values
  __aBiasRaw[X_AXIS] := axBias
  __aBiasRaw[Y_AXIS] := ayBias
  __aBiasRaw[Z_AXIS] := azBias

PUB imu_setGyroCalibration(gxBias, gyBias, gzBias) 'WORKS
'' Manually set gyroscope calibration offset values
  __gBiasRaw[X_AXIS] := gxBias
  __gBiasRaw[Y_AXIS] := gyBias
  __gBiasRaw[Z_AXIS] := gzBias

PUB imu_calibrateMag | i, j, k, mx, my, mz, magMin[3], magMax[3], magTemp[3], msb, lsb 'WORKS
'' Calibrates the Magnetometer on the LSM9DS1 IMU module.
  repeat i from 0 to 128
    repeat while not imu_magAvailable ''Wait until new data available
    imu_readMag(@mx, @my, @mz)
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
    imu_SPIwriteByte(CS_M_PIN, OFFSET_X_REG_L_M + (2 * k), lsb)
    imu_SPIwriteByte(CS_M_PIN, OFFSET_X_REG_H_M + (2 * k), msb)

PUB imu_magAvailable | status 'WORKS
'' Polls the Magnetometer status register to check if new data is available.
  imu_SPIreadBytes(CS_M_PIN, STATUS_REG_M, @status, 1)
  return ((status & (1 << 3)) >> 3)


PUB imu_setMagScale(mScl) | temp 'WORKS
'' Set the full-scale range of the magnetometer
  if (mScl <> 4) and (mScl <> 8) and (mScl <> 12) and (mScl <> 16)
    mScl := 4
  ' We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:, temp
  imu_SPIreadBytes(CS_M_PIN, CTRL_REG2_M, @temp, 1)
  ' Then mask out the mag scale bits:
  temp &= $FF ^($3 << 5)
  case(mScl)
    8 :
      temp |= ($1 << 5)
      __settings_mag_scale := 8
      __mRes := 3448.28
    12 :
      temp |= ($2 << 5)
      __settings_mag_scale := 12
      __mRes := 2298.85
    16 :
      temp |= ($3 << 5)
      __settings_mag_scale := 16
      __mRes := 1724.14
    OTHER :
      __settings_mag_scale := 4
      __mRes := 6896.55
  imu_SPIwriteByte(CS_M_PIN, CTRL_REG2_M, temp)

PUB imu_readMag(mx, my, mz) | temp[2], tempX, tempY, tempZ 'WORKS
'' Reads the Magnetometer output registers.

'' We'll read six bytes from the mag into temp
  imu_SPIreadBytes(CS_M_PIN, OUT_X_L_M, @temp, 6) ' Read 6 bytes, beginning at OUT_X_L_M
  tempX := (temp.byte[1] << 8) | temp.byte[0] ' Store x-axis values into mx
  tempY := (temp.byte[3] << 8) | temp.byte[2] ' Store y-axis values into my
  tempZ := (temp.byte[5] << 8) | temp.byte[4] ' Store z-axis values into mz
  long[mx] := ~~tempX
  long[my] := ~~tempY
  long[mz] := ~~tempZ

PUB imu_readMagCalculated(mx, my, mz) | tempX, tempY, tempZ 'WORKS
'' Reads the Magnetometer output registers and scales the outputs to Gauss'.
  imu_readMag(@tempX, @tempY, @tempZ)
  long[mx] := math.DivF (math.FloatF(tempX), __mRes)'(tempX) / __mRes
  long[my] := math.DivF (math.FloatF(tempY), __mRes)'(tempY) / __mRes
  long[mz] := math.DivF (math.FloatF(tempZ), __mRes)'(tempZ) / __mRes
  if (__autoCalc)
    long[mx] -= __mBiasRaw[X_AXIS]
    long[my] -= __mBiasRaw[Y_AXIS]
    long[mz] -= __mBiasRaw[Z_AXIS]

PUB imu_clearMagInterrupt | tempRegValue 'UNTESTED
'' Clears out any interrupts set up on the Magnetometer and
'' resets all Magnetometer interrupt registers to their default values
  imu_SPIwriteByte(CS_M_PIN, INT_THS_L_M, $00)
  imu_SPIwriteByte(CS_M_PIN, INT_THS_H_M, $00)
  imu_SPIwriteByte(CS_M_PIN, INT_SRC_M, $00)
  imu_SPIwriteByte(CS_M_PIN, INT_CFG_M, $00)

PUB imu_accelAvailable | status 'WORKS
'' Polls the Accelerometer status register to check if new data is available.
  imu_SPIreadBytes(CS_AG_PIN, STATUS_REG_1, @status, 1)
  return (status & (1 << 0))

PUB imu_calibrateAG | data[2], samples, ii, ax, ay, az, gx, gy, gz, aBiasRawTemp[3], gBiasRawTemp[3], tempF, tempS 'PARTIAL
'' Calibrates the Accelerometer and Gyroscope on the LSM9DS1 IMU module.
  samples := 0
  ' Turn on FIFO and set threshold to 32 samples
  imu_SPIreadBytes(CS_AG_PIN, CTRL_REG9, @tempF, 1)
  tempF |= (1 << 1)
  imu_SPIwriteByte(CS_AG_PIN, CTRL_REG9, tempF)
  imu_SPIwriteByte(CS_AG_PIN, FIFO_CTRL, (((FIFO_THS & $7) << 5) | $1F))
  repeat while samples < $1F
    imu_SPIreadBytes(CS_AG_PIN, FIFO_SRC, @tempS, 1) ' Read number of stored samples
'    ser.Str (string("tempS: "))
'    ser.dec (tempS.byte[0])
'    ser.NewLine
    samples := tempS.byte[0] & $3F
'  ser.Str (string("samples: "))
'  ser.Dec (samples)
'  ser.NewLine
  repeat ii from 0 to samples-1 'while (ii < byte[@samples])
    ' Read the gyro data stored in the FIFO
    imu_readGyro(@gx, @gy, @gz)
    gBiasRawTemp[0] += gx
    gBiasRawTemp[1] += gy
    gBiasRawTemp[2] += gz
    imu_readAccel(@ax, @ay, @az)
    aBiasRawTemp[0] += ax
    aBiasRawTemp[1] += ay
    aBiasRawTemp[2] += az - math.TruncFInt(__aRes) ' Assumes sensor facing up!
'  ser.Str (string("aBiasRawTemp[0]="))
'  ser.dec (aBiasRawTemp[0])
'  ser.NewLine
'  ser.Str (string("aBiasRawTemp[1]="))
'  ser.dec (aBiasRawTemp[1])
'  ser.NewLine
'  ser.Str (string("aBiasRawTemp[2]="))
'  ser.str (fs.FloatToString(aBiasRawTemp[2]))
'  ser.Dec (aBiasRawTemp[2])
'  ser.NewLine
'  ser.Str (string("ii:",13))
  repeat ii from 0 to 2'while (ii < 3)
'    ser.Dec (ii)
'    ser.NewLine
    __gBiasRaw[ii] := gBiasRawTemp[ii] / samples
    __gBias[ii] := (__gBiasRaw[ii]) / __gRes
    __aBiasRaw[ii] := aBiasRawTemp[ii] / samples
    __aBias[ii] := __aBiasRaw[ii] / math.TruncFInt (__aRes)
  __autoCalc := 1
  'Disable FIFO
  imu_SPIreadBytes(CS_AG_PIN, CTRL_REG9, @tempF, 1)
  tempF &= !(1 << 1)
  imu_SPIwriteByte(CS_AG_PIN, CTRL_REG9, tempF)
  imu_SPIwriteByte(CS_AG_PIN, FIFO_CTRL, ((FIFO_OFF & $7) << 5))
'  waitforkey  

PUB imu_setAccelScale(aScl) | tempRegValue 'WORKS
'' Sets the full-scale range of the Accelerometer.
'' This function can be called to set the scale of the Accelerometer to 2, 4, 8, or 16 g's.

  if (aScl <> 2) and (aScl <> 4) and (aScl <> 8) and (aScl <> 16)
    aScl := 2
  __aRes := math.DivF (32768.0, math.FloatF (aScl))
  __settings_accel_scale := aScl
  ' We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
  imu_SPIreadBytes(CS_AG_PIN, CTRL_REG6_XL, @tempRegValue, 1)
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
  imu_SPIwriteByte(CS_AG_PIN, CTRL_REG6_XL, tempRegValue)

PUB imu_readAccel(ax, ay, az) | temp[2], tempX, tempY, tempZ 'WORKS
''Reads the Accelerometer output registers

'' We'll read six bytes from the accelerometer into temp  , tempX, tempY, tempZ
  imu_SPIreadBytes(CS_AG_PIN, OUT_X_L_XL, @temp, 6) ' Read 6 bytes, beginning at OUT_X_L_XL
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

PUB imu_readAccelCalculated(ax, ay, az) | tempX, tempY, tempZ 'WORKS
'' Reads the Accelerometer output registers and scales the outputs to g's (1 g = 9.8 m/s/s)
  imu_readAccel(@tempX, @tempY, @tempZ)
  long[ax] := math.DivF (math.FloatF(tempX), __aRes)'(tempX) / __aRes
  long[ay] := math.DivF (math.FloatF(tempY), __aRes)'(tempY) / __aRes
  long[az] := math.DivF (math.FloatF(tempZ), __aRes)'(tempZ) / __aRes

PUB imu_setAccelInterrupt(axis, threshold, duration, overUnder, andOr) | tempRegValue, accelThs, accelThsH, tempThs 'WORKS
''Configures the Accelerometer interrupt output to the INT_A/G pin.
  overUnder &= $01
  andOr &= $01
  tempRegValue := 0
  imu_SPIreadBytes(CS_AG_PIN, CTRL_REG4, @tempRegValue, 1) ' Make sure interrupt is NOT latched
  tempRegValue &= $FD
  imu_SPIwriteByte(CS_AG_PIN, CTRL_REG4, tempRegValue)
  imu_SPIreadBytes(CS_AG_PIN, INT_GEN_CFG_XL, @tempRegValue, 1)
  if (andOr)
    tempRegValue |= $80
  else
    tempRegValue &= $7F
  if (threshold < 0)
    threshold := -1 * threshold
  accelThs := 0
  tempThs := 0
  tempThs := math.TruncFInt(math.MulF(__aRes, threshold)) >> 7
  accelThs := tempThs & $FF

  case(axis)
    X_AXIS:
      tempRegValue |= (1 <<(0 + overUnder))
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_X_XL, accelThs)
    Y_AXIS:
      tempRegValue |= (1 <<(2 + overUnder))
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_Y_XL, accelThs)
    Z_AXIS:
      tempRegValue |= (1 <<(4 + overUnder))
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_Z_XL, accelThs)
    OTHER:
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_X_XL, accelThs)
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_Y_XL, accelThs)
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_Z_XL, accelThs)
      tempRegValue |= (%00010101 << overUnder)
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_CFG_XL, tempRegValue)
  if (duration > 0)
    duration := $80 | (duration & $7F)
  else
    duration := $00
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_DUR_XL, duration)
  imu_SPIreadBytes(CS_AG_PIN, INT1_CTRL, @tempRegValue, 1)
  tempRegValue |= $40
  imu_SPIwriteByte(CS_AG_PIN, INT1_CTRL, tempRegValue)

PUB imu_clearAccelInterrupt | tempRegValue 'WORKS
'' Clears out any interrupts set up on the Accelerometer and
'' resets all Accelerometer interrupt registers to their default values.
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_X_XL, $00)
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_Y_XL, $00)
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_Z_XL, $00)
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_CFG_XL, $00)
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_DUR_XL, $00)
  imu_SPIreadBytes(CS_AG_PIN, INT1_CTRL, @tempRegValue, 1)
  tempRegValue &= $BF
  imu_SPIwriteByte(CS_AG_PIN, INT1_CTRL, tempRegValue)

PUB imu_gyroAvailable | status 'WORKS
'' Polls the Gyroscope status register to check if new data is available
  imu_SPIreadBytes(CS_AG_PIN, STATUS_REG_1, @status, 1)
  return ((status & (1 << 1)) >> 1)

PUB imu_setGyroScale(gScl) | ctrl1RegValue 'WORKS
'' Sets the full-scale range of the Gyroscope.
  if ((gScl <> 245) and (gScl <> 500) and (gScl <> 2000))
    gScl := 245
  __settings_gyro_scale := gScl
  __gRes := math.DivF (32768.0, math.FloatF (gScl))
  ' Read current value of CTRL_REG1_G:, ctrl1RegValue
  imu_SPIreadBytes(CS_AG_PIN, CTRL_REG1_G, @ctrl1RegValue, 1)
  ' Mask out scale bits (3 & 4):
  ctrl1RegValue &= $E7
  case(gScl)
    500 :
      ctrl1RegValue |= ($1 << 3)
    2000 :
      ctrl1RegValue |= ($3 << 3)
    OTHER :
  imu_SPIwriteByte(CS_AG_PIN, CTRL_REG1_G, ctrl1RegValue)

PUB imu_readGyro(gx, gy, gz) | temp[2], tempX, tempY, tempZ 'WORKS
'' Reads the Gyroscope output registers.

'' We'll read six bytes from the gyro into temp
  imu_SPIreadBytes(CS_AG_PIN, OUT_X_L_G, @temp, 6) ' Read 6 bytes, beginning at OUT_X_L_G
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

PUB imu_readGyroCalculated(gx, gy, gz) | tempX, tempY, tempZ 'WORKS
'' Reads the Gyroscope output registers and scales the outputs to degrees of rotation per second (DPS).

'' Return the gyro raw reading times our pre-calculated DPS / (ADC tick):, tempX, tempY, tempZ
  imu_readGyro(@tempX, @tempY, @tempZ)
  long[gx] := math.DivF (math.floatf(tempX), __gRes)
  long[gy] := math.DivF (math.floatf(tempY), __gRes)
  long[gz] := math.DivF (math.floatf(tempZ), __gRes)

PUB imu_setGyroInterrupt(axis, threshold, duration, overUnder, andOr) | tempRegValue, gyroThs, gyroThsH, gyroThsL 'WORKS
'' Configures the Gyroscope interrupt output to the INT_A/G pin.
  overUnder &= $01
  tempRegValue := 0
  imu_SPIreadBytes(CS_AG_PIN, CTRL_REG4, @tempRegValue, 1) ' Make sure interrupt is NOT latched
  tempRegValue &= $FD
  imu_SPIwriteByte(CS_AG_PIN, CTRL_REG4, tempRegValue)
  imu_SPIreadBytes(CS_AG_PIN, INT_GEN_CFG_G, @tempRegValue, 1)
  if andOr
    tempRegValue |= $80
  else
    tempRegValue &= $7F
  gyroThs := 0'word
  gyroThsH := 0'byte
  gyroThsL := 0'byte
  gyroThs := math.truncfint(math.mulf(__gRes, math.FloatF (threshold)))

  if gyroThs > 16383'(gyroThs > 16383)
    gyroThs := 16383
  if gyroThs < -16384'(gyroThs < -16384)
    gyroThs := -16384
  gyroThsL := (gyroThs & $FF)
  gyroThsH := (gyroThs >> 8) & $7F

  case(axis)
    X_AXIS :
      tempRegValue |= (1 <<(0 + overUnder))
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_XH_G, gyroThsH)
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_XL_G, gyroThsL)
    Y_AXIS :
      tempRegValue |= (1 <<(2 + overUnder))
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_YH_G, gyroThsH)
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_YL_G, gyroThsL)
    Z_AXIS :
      tempRegValue |= (1 <<(4 + overUnder))
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_ZH_G, gyroThsH)
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_ZL_G, gyroThsL)
    OTHER :
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_XH_G, gyroThsH)
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_XL_G, gyroThsL)
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_YH_G, gyroThsH)
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_YL_G, gyroThsL)
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_ZH_G, gyroThsH)
      imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_ZL_G, gyroThsL)
      tempRegValue |= (%00010101 << overUnder)
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_CFG_G, tempRegValue)
  if (duration > 0)
    duration := $80 | (duration & $7F)
  else
    duration := $00
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_DUR_G, duration)
  imu_SPIreadBytes(CS_AG_PIN, INT1_CTRL, @tempRegValue, 1)
  tempRegValue |= $80
  imu_SPIwriteByte(CS_AG_PIN, INT1_CTRL, tempRegValue)

PUB imu_clearGyroInterrupt | tempRegValue 'WORKS
'' Clears out any interrupts set up on the Gyroscope and resets all Gyroscope interrupt registers to their default values.
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_XH_G, $00)
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_XL_G, $00)
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_YH_G, $00)
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_YL_G, $00)
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_ZH_G, $00)
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_THS_ZL_G, $00)
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_CFG_G, $00)
  imu_SPIwriteByte(CS_AG_PIN, INT_GEN_DUR_G, $00)
  imu_SPIreadBytes(CS_AG_PIN, INT1_CTRL, @tempRegValue, 1)
  tempRegValue &= $7F
  imu_SPIwriteByte(CS_AG_PIN, INT1_CTRL, @tempRegValue)


PUB imu_SPIwriteByte(csPin, subAddress, data) 'WORKS
'' SPI: Write byte _data_ to SPI device at _subAddress_ on Propeller I/O pin _csPin_
  low(csPin)
  spi.shiftout(SDIO_PIN, SCL_PIN, spi#MSBFIRST, 8, subAddress & $3F)
  spi.shiftout(SDIO_PIN, SCL_PIN, spi#MSBFIRST, 8, data)
  high(csPin)

PUB imu_SPIreadBytes(csPin, subAddress, dest, count) | rAddress, i 'WORKS
'' SPI: Read _count_ bytes from SPI device at _subAddress_ on Propeller I/O pin _csPin_ into pointer _dest_
  ' To indicate a read, set bit 0 (msb) of first byte to 1, rAddress
  rAddress := $80 | (subAddress & $3F)
  ' Mag SPI port is different. If we're reading multiple bytes, 
  ' set bit 1 to 1. The remaining six bytes are the address to be read
  if (csPin == CS_M_PIN) and count > 1
    rAddress |= $40
  low(csPin)
  spi.shiftout(SDIO_PIN, SCL_PIN, spi#MSBFIRST, 8, rAddress)
  repeat i from 0 to count-1
    byte[dest][i] := spi.shiftin(SDIO_PIN, SCL_PIN, spi#MSBPRE, 8)
  high(csPin)


PRI high(pin)
'' Abbreviated way to bring an output pin high
    dira[pin]~~
    outa[pin]~~
    
PRI low(pin)
'' Abbreviated way to bring an output pin low
    dira[pin]~~
    outa[pin]~

{{ Some methods used as debugging aids, etc }}
PRI led
'' Blink Red LED at 2Hz, forever
  dira[LEDRED]~~
  repeat
    !outa[LEDRED]
    waitcnt(cnt+clkfreq/2)

PRI waitforkey
  
  ser.NewLine
  ser.Str (string("Press any key to continue...",13))
  repeat until ser.CharIn

PRI prompt(message) : response
  
  ser.NewLine
  ser.Str (message)
  
  repeat until response := ser.CharIn


PRI uTesla(Gauss): uTeslas
''Given magnetic field in Gauss, returns the equivalent micro-Teslas (uT)
  return math.MulF (Gauss, 100.0)
