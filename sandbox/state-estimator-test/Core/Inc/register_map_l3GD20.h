#ifndef REGISTER_MAP_L3GD20_H
#define REGISTER_MAP_L3GD20_H

#define L3GD20_WHO_AM_I                                     0b11010100

#define L3GD20_REG_WHO_AM_I                                 0x0F
#define L3GD20_REG_CTRL_REG1                                0x20
#define L3GD20_REG_CTRL_REG2                                0x21
#define L3GD20_REG_CTRL_REG3                                0x22
#define L3GD20_REG_CTRL_REG4                                0x23
#define L3GD20_REG_CTRL_REG5                                0x24
#define L3GD20_REG_REFERENCE                                0x25
#define L3GD20_REG_OUT_TEMP                                 0x26
#define L3GD20_REG_STATUS_REG                               0x27
#define L3GD20_REG_OUT_X_L                                  0x28
#define L3GD20_REG_OUT_X_H                                  0x29
#define L3GD20_REG_OUT_Y_L                                  0x2A
#define L3GD20_REG_OUT_Y_H                                  0x2B
#define L3GD20_REG_OUT_Z_L                                  0x2C
#define L3GD20_REG_OUT_Z_H                                  0x2D
#define L3GD20_REG_FIFO_CTRL_REG                            0x2E
#define L3GD20_REG_FIFO_SRC_REG                             0x2F
#define L3GD20_REG_INT1_CFG                                 0x30
#define L3GD20_REG_INT1_SRC                                 0x31
#define L3GD20_REG_INT1_THS_XH                              0x32
#define L3GD20_REG_INT1_THS_XL                              0x33
#define L3GD20_REG_INT1_THS_YH                              0x34
#define L3GD20_REG_INT1_THS_YL                              0x35
#define L3GD20_REG_INT1_THS_ZH                              0x36
#define L3GD20_REG_INT1_THS_ZL                              0x37
#define L3GD20_REG_INT1_DURATION                            0x38

#define L3GD20_CTRL_REG1_DR_ODR_95HZ                        (0x00<<6)
#define L3GD20_CTRL_REG1_DR_ODR_190HZ                       (0x01<<6)
#define L3GD20_CTRL_REG1_DR_ODR_380HZ                       (0x02<<6)
#define L3GD20_CTRL_REG1_DR_ODR_760HZ                       (0x03<<6)
#define L3GD20_CTRL_REG1_BW_CUTOFF_LPF1_32HZ                (0x00<<4)
#define L3GD20_CTRL_REG1_BW_CUTOFF_LPF1_54HZ                (0x01<<4)
#define L3GD20_CTRL_REG1_BW_CUTOFF_LPF1_78HZ                (0x02<<4)
#define L3GD20_CTRL_REG1_BW_CUTOFF_LPF1_93HZ                (0x03<<4)
#define L3GD20_CTRL_REG1_PD_OPERATING_MODE_POWER_DOWN       (0x00<<3)
#define L3GD20_CTRL_REG1_PD_OPERATING_MODE_SLEEP            (0x01<<3)
#define L3GD20_CTRL_REG1_PD_OPERATING_MODE_NORMAL           (0x01<<3)
#define L3GD20_CTRL_REG1_XEN_ENABLE                         (0x01<<2)
#define L3GD20_CTRL_REG1_YEN_ENABLE                         (0x01<<1)
#define L3GD20_CTRL_REG1_ZEN_ENABLE                         (0x01<<0)
#define L3GD20_CTRL_REG1_XEN_DISABLE                        (0x00<<2)
#define L3GD20_CTRL_REG1_YEN_DISABLE                        (0x00<<1)
#define L3GD20_CTRL_REG1_ZEN_DISABLE                        (0x00<<0)

#define L3GD20_CTRL_REG2_HPM_NORMAL_MODE                    (0x00<<4)
#define L3GD20_CTRL_REG2_HPM_REFERENCE_SIGNAL_FOR_FILTERING (0x01<<4)
#define L3GD20_CTRL_REG2_HPM_AUTORESET_ON_INTERRUPT_EVENT   (0x03<<4)
#define L3GD20_CTRL_REG2_HPCF_CUTOFF_0000                   (0x00<<0)
#define L3GD20_CTRL_REG2_HPCF_CUTOFF_0001                   (0x01<<0)
#define L3GD20_CTRL_REG2_HPCF_CUTOFF_0010                   (0x02<<0)
#define L3GD20_CTRL_REG2_HPCF_CUTOFF_0011                   (0x03<<0)
#define L3GD20_CTRL_REG2_HPCF_CUTOFF_0100                   (0x04<<0)
#define L3GD20_CTRL_REG2_HPCF_CUTOFF_0101                   (0x05<<0)
#define L3GD20_CTRL_REG2_HPCF_CUTOFF_0111                   (0x07<<0)
#define L3GD20_CTRL_REG2_HPCF_CUTOFF_1000                   (0x08<<0)
#define L3GD20_CTRL_REG2_HPCF_CUTOFF_1001                   (0x09<<0)

#define L3GD20_CTRL_REG3_I1_INT1_ENABLE                     (0x01<<7)
#define L3GD20_CTRL_REG3_I1_INT1_DISABLE                    (0x00<<7)
#define L3GD20_CTRL_REG3_I1_BOOT_STATUS_AVAILABLE_ENABLE    (0x01<<6)
#define L3GD20_CTRL_REG3_I1_BOOT_STATUS_AVAILABLE_DISABLE   (0x00<<6)
#define L3GD20_CTRL_REG3_H_LACTIVE_LOW                      (0x01<<5)
#define L3GD20_CTRL_REG3_H_LACTIVE_HIGH                     (0x00<<5)
#define L3GD20_CTRL_REG3_PP_OD_PUSH_PULL                    (0x00<<4)
#define L3GD20_CTRL_REG3_PP_OD_OPEN_DRAIN                   (0x01<<4)
#define L3GD20_CTRL_REG3_I2_DRDY_ENABLE                     (0x01<<3)
#define L3GD20_CTRL_REG3_I2_DRDY_DISABLE                    (0x00<<3)
#define L3GD20_CTRL_REG3_I2_WTM_FIFO_WATERMARK_ENABLE       (0x01<<2)
#define L3GD20_CTRL_REG3_I2_WTM_FIFO_WATERMARK_DISABLE      (0x00<<2)
#define L3GD20_CTRL_REG3_I2_ORUN_FIFO_OVERRUN_ENABLE        (0x01<<1)
#define L3GD20_CTRL_REG3_I2_ORUN_FIFO_OVERRUN_DISABLE       (0x00<<1)
#define L3GD20_CTRL_REG3_I2_EMPTY_FIFO_EMPTY_ENABLE         (0x01<<0)
#define L3GD20_CTRL_REG3_I2_EMPTY_FIFO_EMPTY_DISABLE        (0x00<<0)

#define L3GD20_CTRL_REG4_BDU_ENABLE                         (0x01<<7)
#define L3GD20_CTRL_REG4_BDU_DISABLE                        (0x00<<7)
#define L3GD20_CTRL_REG4_BLE_LITTLE_ENDIAN                  (0x00<<6)
#define L3GD20_CTRL_REG4_BLE_BIG_ENDIAN                     (0x01<<6)
#define L3GD20_CTRL_REG4_FS_FULL_SCALE_250DPS               (0x00<<4)
#define L3GD20_CTRL_REG4_FS_FULL_SCALE_500DPS               (0x01<<4)
#define L3GD20_CTRL_REG4_FS_FULL_SCALE_2000DPS              (0x02<<4)
#define L3GD20_CTRL_REG4_SIM_4_WIRE_INTERFACE               (0x00<<0)
#define L3GD20_CTRL_REG4_SIM_3_WIRE_INTERFACE               (0x01<<0)

#define L3GD20_CTRL_REG5_BOOT                               (0x01<<7)
#define L3GD20_CTRL_REG5_FIFO_EN_FIFO_ENABLE                (0x01<<6)
#define L3GD20_CTRL_REG5_FIFO_EN_FIFO_DISABLE               (0x00<<6)
#define L3GD20_CTRL_REG5_HPEN_HIGH_PASS_FILTER_ENABLE       (0x01<<4)
#define L3GD20_CTRL_REG5_HPEN_HIGH_PASS_FILTER_DISABLE      (0x00<<4)
#define L3GD20_CTRL_REG5_INT1_SEL_00                        (0x00<<2)
#define L3GD20_CTRL_REG5_INT1_SEL_01                        (0x01<<2)
#define L3GD20_CTRL_REG5_INT1_SEL_10                        (0x02<<2)
#define L3GD20_CTRL_REG5_INT1_SEL_11                        (0x03<<2)
#define L3GD20_CTRL_REG5_OUT_SEL_00                         (0x00<<0)
#define L3GD20_CTRL_REG5_OUT_SEL_01                         (0x01<<0)
#define L3GD20_CTRL_REG5_OUT_SEL_10                         (0x02<<0)
#define L3GD20_CTRL_REG5_OUT_SEL_11                         (0x03<<0)

#define L3GD20_STATUS_REG_ZYXOR                             (0x01<<7)
#define L3GD20_STATUS_REG_ZOR                               (0x01<<6)
#define L3GD20_STATUS_REG_YOR                               (0x01<<5)
#define L3GD20_STATUS_REG_XOR                               (0x01<<4)
#define L3GD20_STATUS_REG_ZYXDA                             (0x01<<3)
#define L3GD20_STATUS_REG_ZDA                               (0x01<<2)
#define L3GD20_STATUS_REG_YDA                               (0x01<<1)
#define L3GD20_STATUS_REG_XDA                               (0x01<<0)

#define L3GD20_FIFO_CTRL_REG_FM_MODE_BAYPASS                (0x00<<5)
#define L3GD20_FIFO_CTRL_REG_FM_MODE_FIFO                   (0x01<<5)
#define L3GD20_FIFO_CTRL_REG_FM_MODE_STREAM                 (0x02<<5)
#define L3GD20_FIFO_CTRL_REG_FM_MODE_STREAM_TO_FIFO         (0x03<<5)
#define L3GD20_FIFO_CTRL_REG_FM_MODE_BAYPASS_TO_STREAM      (0x04<<5)
#define L3GD20_FIFO_CTRL_REG_WTM(watermark)                 ((watermark)<<0)

#define L3GD20_FIFO_SRC_REG_WTM                             (0x01<<7)
#define L3GD20_FIFO_SRC_REG_OVRN                            (0x01<<6)
#define L3GD20_FIFO_SRC_REG_EMPTY                           (0x01<<5)
#define L3GD20_FIFO_SRC_REG_FSS                             (0x1F<<0)

#define L3GD20_INT1_CFG_AND_OR_COMBINATION_OR               (0x00<<7)
#define L3GD20_INT1_CFG_AND_OR_COMBINATION_AND              (0x01<<7)
#define L3GD20_INT1_CFG_LIR_NOT_LATCHED                     (0x00<<6)
#define L3GD20_INT1_CFG_LIR_LATCHED                         (0x01<<6)
#define L3GD20_INT1_CFG_ZHIE_ENABLE                         (0x01<<5)
#define L3GD20_INT1_CFG_ZHIE_DISABLE                        (0x00<<5)
#define L3GD20_INT1_CFG_ZLIE_ENABLE                         (0x01<<4)
#define L3GD20_INT1_CFG_ZLIE_DISABLE                        (0x00<<4)
#define L3GD20_INT1_CFG_YHIE_ENABLE                         (0x01<<3)
#define L3GD20_INT1_CFG_YHIE_DISABLE                        (0x00<<3)
#define L3GD20_INT1_CFG_YLIE_ENABLE                         (0x01<<2)
#define L3GD20_INT1_CFG_YLIE_DISABLE                        (0x00<<2)
#define L3GD20_INT1_CFG_XHIE_ENABLE                         (0x01<<1)
#define L3GD20_INT1_CFG_XHIE_DISABLE                        (0x00<<1)
#define L3GD20_INT1_CFG_XLIE_ENABLE                         (0x01<<0)
#define L3GD20_INT1_CFG_XLIE_DISABLE                        (0x00<<0)

#define L3GD20_INT1_SRC_IA                                  (0x01<<6)
#define L3GD20_INT1_SRC_ZH                                  (0x01<<5)
#define L3GD20_INT1_SRC_ZL                                  (0x01<<4)
#define L3GD20_INT1_SRC_YH                                  (0x01<<3)
#define L3GD20_INT1_SRC_YL                                  (0x01<<2)
#define L3GD20_INT1_SRC_XH                                  (0x01<<1)
#define L3GD20_INT1_SRC_XL                                  (0x01<<0)

#define L3GD20_DURATION_WAIT_ENABLE                         (0x01<<7)
#define L3GD20_DURATION_WAIT_DISABLE                        (0x00<<7)
#define L3GD20_DURATION_D(time)                             ((time)<<0)

#endif
