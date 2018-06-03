{
    --------------------------------------------
    Filename: LSM9DS1-Demo.spin
    Author: Jesse Burt
    Copyright (c) 2018
    See end of file for terms of use.
    --------------------------------------------
}

CON

  _clkmode = cfg#_clkmode
  _xinfreq = cfg#_xinfreq

  SCL_PIN       = 0
  SDIO_PIN      = 1
  CS_AG_PIN     = 2
  CS_M_PIN      = 3
  INT_AG_PIN    = 4
  INT_M_PIN     = 5

  DISP_HELP     = 1
  PRINT_REGS    = 2
  DISP_GYRAW    = 3
  DISP_ACCRAW   = 4
  DISP_MAGRAW   = 5
  DISP_TEMPRAW  = 6
  DISP_GYCAL    = 7
  DISP_ACCCAL   = 8
  DISP_MAGCAL   = 9
  DISP_TEMPCAL  = 10


OBJ

  cfg       : "core.con.client.flip"
  ser       : "com.serial.terminal"
  time      : "time"
  imu       : "sensor.imu.tri.lsm9ds1"
  int       : "string.integer"

VAR

  long  _keyDaemon_stack[100]
  byte  _keyDaemon_cog, _imu_cog, _ser_cog
  byte  _demo_state
  byte  _max_cols

PUB Main

  Setup
  imu.setMagScale (12)
  _demo_state := DISP_HELP

  DisplayTempCalc
  repeat
    case _demo_state
      PRINT_REGS:   PrintRegs
      DISP_ACCRAW:  DisplayAccelRaw
      DISP_GYRAW:   DisplayGyroRaw
      DISP_MAGRAW:  DisplayMagRaw
      DISP_TEMPRAW: DisplayTempRaw
      DISP_ACCCAL:  DisplayAccelCalc
      DISP_GYCAL:   DisplayGyroCalc
      DISP_MAGCAL:  DisplayMagCalc
      DISP_TEMPCAL: DisplayTempCalc
      DISP_HELP:  Help
      OTHER:
        _demo_state := DISP_HELP

PUB Setup

  _ser_cog := ser.Start (115_200)
  ser.Clear
  _imu_cog := imu.Start (SCL_PIN, SDIO_PIN, CS_AG_PIN, CS_M_PIN, INT_AG_PIN, INT_M_PIN)
  _keyDaemon_cog := cognew(keyDaemon, @_keyDaemon_stack)
  _max_cols := 4

  if _imu_cog
    if imu.whoAmI == $683D
      ser.Str (string("LSM9DS1 found!", ser#NL))
    else
      ser.Str (string("LSM9DS1 not found - halting!", ser#NL))
      imu.Stop
      ser.Stop
      cogstop(_keyDaemon_cog)
      repeat
  else
    ser.Str (string("LSM9DS1 object unable to start SPI cog - halting!", ser#NL))
    imu.Stop
    ser.Stop
    cogstop(_keyDaemon_cog)
    repeat
  waitkey

PUB Help

  ser.Clear
  ser.Str (string("Keys: ", ser#NL, ser#NL))
  ser.Str (string("h, H:  This help screen", ser#NL))
  ser.Str (string("p, P:  Display register contents", ser#NL))
  ser.Str (string("g, G:  Display Gyro (Raw/Cal) Data", ser#NL))
  ser.Str (string("a, A:  Display Accel (Raw/Cal) Data", ser#NL))
  ser.Str (string("m, M:  Display Mag (Raw/Cal) Data", ser#NL))
  ser.Str (string("t, T:  Display Temp (Raw/Cal) Data", ser#NL))

  repeat until _demo_state <> DISP_HELP

PUB DisplayAccelCalc | ax, ay, az

  repeat until _demo_state <> DISP_ACCCAL
    ser.Clear
    ser.Str (string("Display Calculated Accel data (milli-g's):", ser#NL))
    imu.readAccelCalculated (@ax, @ay, @az)
    ser.Str (string("AX: "))
    ser.Str (int.DecPadded (ax, 6))
    ser.Str (string("  AY: "))
    ser.Str (int.DecPadded (ay, 6))
    ser.Str (string("  AZ: "))
    ser.Str (int.DecPadded (az, 6))
    time.MSleep (100)

PUB DisplayGyroCalc| gx, gy, gz

  repeat until _demo_state <> DISP_GYCAL
    ser.Clear
    ser.Str (string("Display Calculated Gyro data (milli-degrees per second):", ser#NL))
    imu.readGyroCalculated (@gx, @gy, @gz)
    ser.Str (string("GX: "))
    ser.Str (int.DecPadded(gx, 7))
    ser.Str (string("  GY: "))
    ser.Str (int.DecPadded(gy, 7))
    ser.Str (string("  GZ: "))
    ser.Str (int.DecPadded(gz, 7))
    time.MSleep (100)

PUB DisplayMagCalc| mx, my, mz

  repeat until _demo_state <> DISP_MAGCAL
    ser.Clear
    ser.Str (string("Display calculated Mag data (milli-gauss):", ser#NL))
    imu.readMagCalculated (@mx, @my, @mz)
    ser.Str (string("MX: "))
    ser.Str (int.DecPadded(mx, 6))
    ser.Str (string("  MY: "))
    ser.Str (int.DecPadded(my, 6))
    ser.Str (string("  MZ: "))
    ser.Str (int.DecPadded(mz, 6))
    time.MSleep (100)

PUB DisplayTempCalc | temp

  repeat until _demo_state <> DISP_TEMPCAL
    ser.Clear
    ser.Str (string("Display Calculated Temp data (milli-degrees Celsius):", ser#NL))
    imu.readTempCalculated (@temp, imu#CELSIUS)
    ser.Str (string("Temp: "))
    ser.Str (int.DecPadded(temp, 6))
    time.MSleep (100)

PUB DisplayTempRaw | t

  repeat until _demo_state <> DISP_TEMPRAW
    ser.Clear
    ser.Str (string("Display RAW Temp data:", ser#NL))
    imu.readTemp (@t)
    ser.Str (string("Temp: "))
    ser.Str (int.DecPadded(t, 6))
    time.MSleep (100)
  
PUB DisplayMagRaw | mx, my, mz

  repeat until _demo_state <> DISP_MAGRAW
    ser.Clear
    ser.Str (string("Display RAW Mag data:", ser#NL))
    imu.readMag (@mx, @my, @mz)
    ser.Str (string("MX: "))
    ser.Str (int.DecPadded(mx, 6))
    ser.Str (string("  MY: "))
    ser.Str (int.DecPadded(my, 6))
    ser.Str (string("  MZ: "))
    ser.Str (int.DecPadded(mz, 6))
    time.MSleep (100)

PUB DisplayAccelRaw | ax, ay, az

  repeat until _demo_state <> DISP_ACCRAW
    ser.Clear
    ser.Str (string("Display RAW Accel data:", ser#NL))
    imu.readAccel (@ax, @ay, @az)
    ser.Str (string("AX: "))
    ser.Str (int.DecPadded(ax, 6))
    ser.Str (string("  AY: "))
    ser.Str (int.DecPadded(ay, 6))
    ser.Str (string("  AZ: "))
    ser.Str (int.DecPadded(az, 6))
    time.MSleep (100)

PUB DisplayGyroRaw | gx, gy, gz

  repeat until _demo_state <> DISP_GYRAW
    ser.Clear
    ser.Str (string("Display RAW Gyro data:", ser#NL))
    imu.readGyro (@gx, @gy, @gz)
    ser.Str (string("GX: "))
    ser.Str (int.DecPadded(gx, 6))
    ser.Str (string("  GY: "))
    ser.Str (int.DecPadded(gy, 6))
    ser.Str (string("  GZ: "))
    ser.Str (int.DecPadded(gz, 6))
    time.MSleep (100)

PUB PrintRegs | rec_size, table_offs, icol, regval_tmp

  repeat until _demo_state <> PRINT_REGS
    ser.Clear
    rule (80, 10, ".")  
    rec_size := 18
    icol := 0
  
    ser.Str (string("A/G register map:", ser#NL))
    repeat table_offs from 1 to (ag_regmap*17) step rec_size
      ser.Str (@ag_regmap[table_offs+1])
      ser.Str (string("= "))
      imu.ReadAGReg (ag_regmap[table_offs], @regval_tmp, 1)
      ser.Hex (regval_tmp, 2)
      ser.Str (string(" | "))
      icol++
      if icol == _max_cols
        ser.NewLine
        icol := 0
  
    icol := 0
    ser.NewLine
    ser.NewLine
  
    ser.Str (string("M register map:", ser#NL))
    repeat table_offs from 1 to (m_regmap*17) step rec_size
      ser.Str (@m_regmap[table_offs+1])
      ser.Str (string("= "))
      imu.ReadMReg (m_regmap[table_offs], @regval_tmp, 1)
      ser.hex (regval_tmp, 2)
      ser.Str (string(" | "))
      icol++
      if icol == _max_cols
        ser.NewLine
        icol := 0
    time.MSleep (500)

PUB keyDaemon | key_cmd

  repeat
    repeat until key_cmd := ser.CharIn
    case key_cmd
      "h", "H": _demo_state := DISP_HELP
      "p", "P": _demo_state := PRINT_REGS
      "g", "G":
        case _demo_state
          DISP_GYRAW:
            _demo_state := DISP_GYCAL
          OTHER:
            _demo_state := DISP_GYRAW
      "a", "A":
        case _demo_state
          DISP_ACCRAW:
            _demo_state := DISP_ACCCAL
          OTHER:
            _demo_state := DISP_ACCRAW
      "m", "M":
        case _demo_state
          DISP_MAGRAW:
            _demo_state := DISP_MAGCAL
          OTHER:
            _demo_state := DISP_MAGRAW
      "t", "T":
        case _demo_state
          DISP_TEMPRAW:
            _demo_state := DISP_TEMPCAL
          OTHER:
            _demo_state := DISP_TEMPRAW
      OTHER   : _demo_state := DISP_HELP

PUB rule(cols, ind, hash_char) | i
''Method to draw a rule on a terminal
  repeat i from 0 to cols-1
    case i
      0:
        ser.char(":")
      OTHER:
        ifnot i//ind
          ser.Char (":")
        else
          ser.Char (hash_char)
  ser.NewLine

PUB waitkey

  ser.Str (string("Press any key", ser#NL))
  ser.CharIn

DAT
  
  ag_regmap  byte 54
  byte $04, "ACT_THS         ", 0
  byte $05, "ACT_DUR         ", 0
  byte $06, "INT_GEN_CFG_XL  ", 0
  byte $07, "INT_GEN_THS_X_XL", 0
  byte $08, "INT_GEN_THS_Y_XL", 0
  byte $09, "INT_GEN_THS_Z_XL", 0
  byte $0A, "INT_GEN_DUR_XL  ", 0
  byte $0B, "REFERENCE_G     ", 0
  byte $0C, "INT1_CTRL       ", 0
  byte $0D, "INT2_CTRL       ", 0
  byte $0F, "WHO_AM_I_XG     ", 0
  byte $10, "CTRL_REG1_G     ", 0
  byte $11, "CTRL_REG2_G     ", 0
  byte $12, "CTRL_REG3_G     ", 0
  byte $13, "ORIENT_CFG_G    ", 0
  byte $14, "INT_GEN_SRC_G   ", 0
  byte $15, "OUT_TEMP_L      ", 0
  byte $16, "OUT_TEMP_H      ", 0
  byte $17, "STATUS_REG_0    ", 0
  byte $18, "OUT_X_L_G       ", 0
  byte $19, "OUT_X_H_G       ", 0
  byte $1A, "OUT_Y_L_G       ", 0
  byte $1B, "OUT_Y_H_G       ", 0
  byte $1C, "OUT_Z_L_G       ", 0
  byte $1D, "OUT_Z_H_G       ", 0
  byte $1E, "CTRL_REG4       ", 0
  byte $1F, "CTRL_REG5_XL    ", 0
  byte $20, "CTRL_REG6_XL    ", 0
  byte $21, "CTRL_REG7_XL    ", 0
  byte $22, "CTRL_REG8       ", 0
  byte $23, "CTRL_REG9       ", 0
  byte $24, "CTRL_REG10      ", 0       
  byte $26, "INT_GEN_SRC_XL  ", 0
  byte $27, "STATUS_REG_1    ", 0     
  byte $28, "OUT_X_L_XL      ", 0       
  byte $29, "OUT_X_H_XL      ", 0       
  byte $2A, "OUT_Y_L_XL      ", 0       
  byte $2B, "OUT_Y_H_XL      ", 0       
  byte $2C, "OUT_Z_L_XL      ", 0       
  byte $2D, "OUT_Z_H_XL      ", 0       
  byte $2E, "FIFO_CTRL       ", 0        
  byte $2F, "FIFO_SRC        ", 0         
  byte $30, "INT_GEN_CFG_G   ", 0    
  byte $31, "INT_GEN_THS_XH_G", 0 
  byte $32, "INT_GEN_THS_XL_G", 0 
  byte $33, "INT_GEN_THS_YH_G", 0 
  byte $34, "INT_GEN_THS_YL_G", 0 
  byte $35, "INT_GEN_THS_ZH_G", 0
  byte $36, "INT_GEN_THS_ZL_G", 0 
  byte $37, "INT_GEN_DUR_G   ", 0    
  byte $00, "                ", 0

  m_regmap  byte 24
  byte $05, "OFFSET_X_REG_L_M", 0 
  byte $06, "OFFSET_X_REG_H_M", 0 
  byte $07, "OFFSET_Y_REG_L_M", 0 
  byte $08, "OFFSET_Y_REG_H_M", 0 
  byte $09, "OFFSET_Z_REG_L_M", 0 
  byte $0A, "OFFSET_Z_REG_H_M", 0 
  byte $0F, "WHO_AM_I_M      ", 0       
  byte $20, "CTRL_REG1_M     ", 0      
  byte $21, "CTRL_REG2_M     ", 0      
  byte $22, "CTRL_REG3_M     ", 0      
  byte $23, "CTRL_REG4_M     ", 0      
  byte $24, "CTRL_REG5_M     ", 0      
  byte $27, "STATUS_REG_M    ", 0     
  byte $28, "OUT_X_L_M       ", 0        
  byte $29, "OUT_X_H_M       ", 0        
  byte $2A, "OUT_Y_L_M       ", 0        
  byte $2B, "OUT_Y_H_M       ", 0        
  byte $2C, "OUT_Z_L_M       ", 0        
  byte $2D, "OUT_Z_H_M       ", 0        
  byte $30, "INT_CFG_M       ", 0        
  byte $30, "INT_SRC_M       ", 0        
  byte $32, "INT_THS_L_M     ", 0      
  byte $33, "INT_THS_H_M     ", 0      
  byte $00, "                ", 0

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
