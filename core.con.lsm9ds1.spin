{
    --------------------------------------------
    Filename: core.con.lsm9ds1.spin
    Author: Jesse Burt
    Description: LSM9DS1 low-level constants
    Copyright (c) 2019
    Started Feb 9, 2019
    Updated Feb 10, 2019
    See end of file for terms of use.
    --------------------------------------------
}

CON
' SPI Configuration
    CPOL                    = 0
    CLK_DELAY               = 10
    MOSI_BITORDER           = 5             'MSBFIRST
    MISO_BITORDER           = 0             'MSBPRE
    WHOAMI                  = $683D

' LSM9DS1 Register map
    ACT_THS                 = $04
    ACT_DUR                 = $05
    INT_GEN_CFG_XL          = $06
    INT_GEN_THS_X_XL        = $07
    INT_GEN_THS_Y_XL        = $08
    INT_GEN_THS_Z_XL        = $09
    INT_GEN_DUR_XL          = $0A
    REFERENCE_G             = $0B
    INT1_CTRL               = $0C
    INT2_CTRL               = $0D
    WHO_AM_I_XG             = $0F

    CTRL_REG1_G             = $10
    CTRL_REG1_G_MASK        = $FB
        FLD_ODR             = 5
        FLD_FS              = 3
        FLD_BW              = 0
        BITS_ODR            = %111
        BITS_FS             = %11
        BITS_BW             = %11
        MASK_ODR            = CTRL_REG1_G_MASK ^ (BITS_ODR << FLD_ODR)
        MASK_FS             = CTRL_REG1_G_MASK ^ (BITS_FS << FLD_FS)
        MASK_BW             = CTRL_REG1_G_MASK ^ (BITS_BW << FLD_BW)

    CTRL_REG2_G             = $11
    CTRL_REG2_G_MASK        = $0F
        FLD_INT_SEL         = 2
        FLD_OUT_SEL         = 0
        BITS_INT_SEL        = %11
        BITS_OUT_SEL        = %11
        MASK_INT_SEL        = CTRL_REG2_G_MASK ^ (BITS_INT_SEL << FLD_INT_SEL)
        MASK_OUT_SEL        = CTRL_REG2_G_MASK ^ (BITS_OUT_SEL << FLD_OUT_SEL)

    CTRL_REG3_G             = $12
    CTRL_REG3_G_MASK        = $CF
        FLD_LP_MODE         = 7
        FLD_HP_EN           = 6
        FLD_HPCF_G          = 0
        BITS_HPCF_G         = %1111
        BITS_LP_MODE        = %1
        MASK_LP_MODE        = CTRL_REG3_G_MASK ^ (BITS_LP_MODE << FLD_LP_MODE)

    ORIENT_CFG_G            = $13
    INT_GEN_SRC_G           = $14
    OUT_TEMP_L              = $15
    OUT_TEMP_H              = $16
    STATUS_REG              = $17
        STATUS_REG_MASK     = $7F
        FLD_IG_XL           = 6
        FLD_IG_G            = 5
        FLD_INACT           = 4
        FLD_BOOT_STATUS     = 3
        FLD_TDA             = 2
        FLD_GDA             = 1
        FLD_XLDA            = 0

    OUT_X_L_G               = $18
    OUT_X_H_G               = $19
    OUT_Y_L_G               = $1A
    OUT_Y_H_G               = $1B
    OUT_Z_L_G               = $1C
    OUT_Z_H_G               = $1D

    CTRL_REG4               = $1E
    CTRL_REG4_MASK          = $3B
        FLD_ZEN_G           = 5
        FLD_YEN_G           = 4
        FLD_XEN_G           = 3
        FLD_LIR_XL1         = 1
        FLD_4D_XL1          = 0
        BITS_EN_G           = %111
        MASK_ZEN_G          = CTRL_REG4_MASK ^ (1 << FLD_ZEN_G)
        MASK_YEN_G          = CTRL_REG4_MASK ^ (1 << FLD_YEN_G)
        MASK_XEN_G          = CTRL_REG4_MASK ^ (1 << FLD_XEN_G)
        MASK_EN_G           = CTRL_REG4_MASK ^ (BITS_EN_G << FLD_XEN_G)
        MASK_LIR_XL1        = CTRL_REG4_MASK ^ (1 << FLD_LIR_XL1)
        MASK_4D_XL1         = CTRL_REG4_MASK ^ (1 << FLD_4D_XL1)

    CTRL_REG5_XL            = $1F
    CTRL_REG5_XL_MASK       = $F8
        FLD_DEC_1           = 7
        FLD_DEC_0           = 6
        FLD_ZEN_xL          = 5
        FLD_YEN_XL          = 4
        FLD_XEN_XL          = 3
        BITS_EN_XL          = %111
        MASK_DEC_1          = CTRL_REG5_XL_MASK ^ (1 << FLD_DEC_1)
        MASK_DEC_0          = CTRL_REG5_XL_MASK ^ (1 << FLD_DEC_0)
        MASK_ZEN_XL         = CTRL_REG5_XL_MASK ^ (1 << FLD_ZEN_XL)
        MASK_YEN_XL         = CTRL_REG5_XL_MASK ^ (1 << FLD_YEN_XL)
        MASK_XEN_XL         = CTRL_REG5_XL_MASK ^ (1 << FLD_XEN_XL)
        MASK_EN_XL          = CTRL_REG5_XL_MASK ^ (BITS_EN_XL << FLD_XEN_XL)

    CTRL_REG6_XL            = $20
    CTRL_REG6_XL_MASK       = $FF
        FLD_ODR_XL          = 5
        FLD_FS_XL           = 3
        FLD_BW_SCAL         = 2
        FLD_BW_XL           = 0
        BITS_ODR_XL         = %111
        BITS_FS_XL          = %11
        BITS_BW_SCAL        = %1
        BITS_BW_XL          = %11
        MASK_ODR_XL         = CTRL_REG6_XL_MASK ^ (BITS_ODR_XL << FLD_ODR_XL)
        MASK_FS_XL          = CTRL_REG6_XL_MASK ^ (BITS_FS_XL << FLD_FS_XL)
        MASK_BW_SCAL        = CTRL_REG6_XL_MASK ^ (BITS_BW_SCAL << FLD_BW_SCAL)
        MASK_BW_XL          = CTRL_REG6_XL_MASK ^ (BITS_BW_XL << FLD_BW_XL)

    CTRL_REG7_XL            = $21
    CTRL_REG7_XL_MASK       = $E5
        FLD_HR              = 7
        FLD_DCF             = 5
        FLD_FDS             = 2
        FLD_HPIS1           = 0
        BITS_HR             = %1
        BITS_DCF            = %11
        BITS_FDS            = %1
        BITS_HPIS1          = %1
        MASK_HR             = CTRL_REG7_XL_MASK ^ (BITS_HR << FLD_HR)
        MASK_DCF            = CTRL_REG7_XL_MASK ^ (BITS_DCF << FLD_DCF)
        MASK_FDS            = CTRL_REG7_XL_MASK ^ (BITS_FDS << FLD_FDS)
        MASK_HPIS1          = CTRL_REG7_XL_MASK ^ (BITS_HPIS1 << FLD_HPIS1)

    CTRL_REG8               = $22
    CTRL_REG8_MASK          = $FF
        FLD_BOOT            = 7
        FLD_BDU             = 6
        FLD_H_LACTIVE       = 5
        FLD_PP_OD           = 4
        FLD_SIM             = 3
        FLD_IF_ADD_INC      = 2
        FLD_BLE             = 1
        FLD_SW_RESET        = 0
        MASK_H_LACTIVE      = CTRL_REG8_MASK ^ (1 << FLD_H_LACTIVE)
        MASK_BDU            = CTRL_REG8_MASK ^ (1 << FLD_BDU)
        MASK_BLE            = CTRL_REG8_MASK ^ (1 << FLD_BLE)
        MASK_SW_RESET       = CTRL_REG8_MASK ^ (1 << FLD_SW_RESET)

    CTRL_REG9               = $23
    CTRL_REG9_MASK          = $5F
        FLD_SLEEP_G         = 6
        FLD_FIFO_TEMP_EN    = 4
        FLD_DRDY_MASK_BIT   = 3
        FLD_I2C_DISABLE     = 2
        FLD_FIFO_EN         = 1
        FLD_STOP_ON_FTH     = 0
        BITS_SLEEP_G        = %1
        BITS_FIFO_TEMP_EN   = %1
        BITS_DRDY_MASK_BIT  = %1
        BITS_I2C_DISABLE    = %1
        BITS_FIFO_EN        = %1
        BITS_STOP_ON_FTH    = %1
        MASK_SLEEP_G        = CTRL_REG9_MASK ^ (BITS_SLEEP_G << FLD_SLEEP_G)
        MASK_FIFO_TEMP_EN   = CTRL_REG9_MASK ^ (BITS_FIFO_TEMP_EN << FLD_FIFO_TEMP_EN)
        MASK_DRDY_MASK_BIT  = CTRL_REG9_MASK ^ (BITS_DRDY_MASK_BIT << FLD_DRDY_MASK_BIT)
        MASK_I2C_DISABLE    = CTRL_REG9_MASK ^ (BITS_I2C_DISABLE << FLD_I2C_DISABLE)
        MASK_FIFO_EN        = CTRL_REG9_MASK ^ (BITS_FIFO_EN << FLD_FIFO_EN)
        MASK_STOP_ON_FTH    = CTRL_REG9_MASK ^ (BITS_STOP_ON_FTH << FLD_STOP_ON_FTH)

    CTRL_REG10              = $24
    INT_GEN_SRC_XL          = $26
    INT_GEN_SRC_XL_MASK     = $7F
        FLD_IA_XL           = 6
        FLD_ZH_XL           = 5
        FLD_ZL_XL           = 4
        FLD_YH_XL           = 3
        FLD_YL_XL           = 2
        FLD_XH_XL           = 1
        FLD_XL_XL           = 0


    STATUS_REG_1            = $27
    OUT_X_L_XL              = $28
    OUT_X_H_XL              = $29
    OUT_Y_L_XL              = $2A
    OUT_Y_H_XL              = $2B
    OUT_Z_L_XL              = $2C
    OUT_Z_H_XL              = $2D
    FIFO_CTRL               = $2E
    FIFO_SRC                = $2F
    INT_GEN_CFG_G           = $30
    INT_GEN_THS_XH_G        = $31
    INT_GEN_THS_XL_G        = $32
    INT_GEN_THS_YH_G        = $33
    INT_GEN_THS_YL_G        = $34
    INT_GEN_THS_ZH_G        = $35
    INT_GEN_THS_ZL_G        = $36
    INT_GEN_DUR_G           = $37
    OFFSET_X_REG_L_M        = $05
    OFFSET_X_REG_H_M        = $06
    OFFSET_Y_REG_L_M        = $07
    OFFSET_Y_REG_H_M        = $08
    OFFSET_Z_REG_L_M        = $09
    OFFSET_Z_REG_H_M        = $0A
    WHO_AM_I_M              = $0F
    CTRL_REG1_M             = $20
    CTRL_REG2_M             = $21
    CTRL_REG3_M             = $22
    CTRL_REG4_M             = $23
    CTRL_REG5_M             = $24
    STATUS_REG_M            = $27
    OUT_X_L_M               = $28
    OUT_X_H_M               = $29
    OUT_Y_L_M               = $2A
    OUT_Y_H_M               = $2B
    OUT_Z_L_M               = $2C
    OUT_Z_H_M               = $2D
    INT_CFG_M               = $30
    INT_SRC_M               = $30
    INT_THS_L_M             = $32
    INT_THS_H_M             = $33
    WHO_AM_I_AG_RSP         = $68
    WHO_AM_I_M_RSP          = $3D

    FIFO_OFF                = 0
    FIFO_THS                = 1
    FIFO_CONT_TRIGGER       = 3
    FIFO_OFF_TRIGGER        = 4
    FIFO_CONT               = 5

PUB Null
'' This is not a top-level object
