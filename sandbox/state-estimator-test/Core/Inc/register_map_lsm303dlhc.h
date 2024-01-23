#ifndef REGISTER_MAP_LSM303DLHC_H
#define REGISTER_MAP_LSM303DLHC_H

#define LSM303DLHC_A_I2C_ADDRESS    0b00110010
#define LSM303DLHC_M_I2C_ADDRESS    0b00111100

#define LSM303DLHC_IRA_REG_M_VALUE  0b01001000
#define LSM303DLHC_IRB_REG_M_VALUE  0b00110100
#define LSM303DLHC_IRC_REG_M_VALUE  0b00110011

#define LSM303DLHC_CTRL_REG1_A      0x20
#define LSM303DLHC_CTRL_REG2_A      0x21
#define LSM303DLHC_CTRL_REG3_A      0x22
#define LSM303DLHC_CTRL_REG4_A      0x23
#define LSM303DLHC_CTRL_REG5_A      0x24
#define LSM303DLHC_CTRL_REG6_A      0x25
#define LSM303DLHC_REFERENCE_A      0x26
#define LSM303DLHC_STATUS_REG_A     0x27
#define LSM303DLHC_OUT_X_L_A        0x28
#define LSM303DLHC_OUT_X_H_A        0x29
#define LSM303DLHC_OUT_Y_L_A        0x2A
#define LSM303DLHC_OUT_Y_H_A        0x2B
#define LSM303DLHC_OUT_Z_L_A        0x2C
#define LSM303DLHC_OUT_Z_H_A        0x2D
#define LSM303DLHC_FIFO_CTRL_REG_A  0x2E
#define LSM303DLHC_FIFO_SRC_REG_A   0x2F
#define LSM303DLHC_INT1_CFG_A       0x30
#define LSM303DLHC_INT1_SRC_A       0x31
#define LSM303DLHC_INT1_THS_A       0x32
#define LSM303DLHC_INT1_DURATION_A  0x33
#define LSM303DLHC_INT2_CFG_A       0x34
#define LSM303DLHC_INT2_SRC_A       0x35
#define LSM303DLHC_INT2_THS_A       0x36
#define LSM303DLHC_INT2_DURATION_A  0x37
#define LSM303DLHC_CLICK_CFG_A      0x38
#define LSM303DLHC_CLICK_SRC_A      0x39
#define LSM303DLHC_CLICK_THS_A      0x3A
#define LSM303DLHC_TIME_LIMIT_A     0x3B
#define LSM303DLHC_TIME_LATENCY_A   0x3C
#define LSM303DLHC_TIME_WINDOW_A    0x3D
#define LSM303DLHC_CRA_REG_M        0x00
#define LSM303DLHC_CRB_REG_M        0x01
#define LSM303DLHC_MR_REG_M         0x02
#define LSM303DLHC_OUT_X_H_M        0x03
#define LSM303DLHC_OUT_X_L_M        0x04
#define LSM303DLHC_OUT_Y_H_M        0x05
#define LSM303DLHC_OUT_Y_L_M        0x06
#define LSM303DLHC_OUT_Z_H_M        0x07
#define LSM303DLHC_OUT_Z_L_M        0x08
#define LSM303DLHC_SR_REG_M         0x09
#define LSM303DLHC_IRA_REG_M        0x0A
#define LSM303DLHC_IRB_REG_M        0x0B
#define LSM303DLHC_IRC_REG_M        0x0C
#define LSM303DLHC_TEMP_OUT_H_M     0x31
#define LSM303DLHC_TEMP_OUT_L_M     0x32

#define LSM303DLHC_CTRL_REG1_A_ODR_MODE_POWER_DOWN                                      (0x00<<4)
#define LSM303DLHC_CTRL_REG1_A_ODR_MODE_NORMAL_LOW_POWER_1HZ                            (0x01<<4)
#define LSM303DLHC_CTRL_REG1_A_ODR_MODE_NORMAL_LOW_POWER_10HZ                           (0x02<<4)
#define LSM303DLHC_CTRL_REG1_A_ODR_MODE_NORMAL_LOW_POWER_25HZ                           (0x03<<4)
#define LSM303DLHC_CTRL_REG1_A_ODR_MODE_NORMAL_LOW_POWER_50HZ                           (0x04<<4)
#define LSM303DLHC_CTRL_REG1_A_ODR_MODE_NORMAL_LOW_POWER_100HZ                          (0x05<<4)
#define LSM303DLHC_CTRL_REG1_A_ODR_MODE_NORMAL_LOW_POWER_200HZ                          (0x06<<4)
#define LSM303DLHC_CTRL_REG1_A_ODR_MODE_NORMAL_LOW_POWER_400HZ                          (0x07<<4)
#define LSM303DLHC_CTRL_REG1_A_ODR_MODE_LOW_POWER_1620HZ                                (0x08<<4)
#define LSM303DLHC_CTRL_REG1_A_ODR_MODE_NORMAL_1344HZ_LOW_POWER_5376HZ                  (0x09<<4)
#define LSM303DLHC_CTRL_REG1_A_LPEN_NORMAL_MODE                                         (0x00<<3)
#define LSM303DLHC_CTRL_REG1_A_LPEN_LOW_POWER_MODE                                      (0x01<<3)
#define LSM303DLHC_CTRL_REG1_A_ZEN_ENABLE                                               (0x01<<2)
#define LSM303DLHC_CTRL_REG1_A_YEN_ENABLE                                               (0x01<<1)
#define LSM303DLHC_CTRL_REG1_A_XEN_ENABLE                                               (0x01<<0)
#define LSM303DLHC_CTRL_REG1_A_ZEN_DISABLE                                              (0x00<<2)
#define LSM303DLHC_CTRL_REG1_A_YEN_DISABLE                                              (0x00<<1)
#define LSM303DLHC_CTRL_REG1_A_XEN_DISABLE                                              (0x00<<0)

#define LSM303DLHC_CTRL_REG2_A_HPM_NORMAL_MODE_RESET_READING_HP_RESET_FILTER            (0x00<<6)
#define LSM303DLHC_CTRL_REG2_A_HPM_REFERENCE_SIGNAL_FOR_FILTERING                       (0x01<<6)
#define LSM303DLHC_CTRL_REG2_A_HPM_NORMAL_MODE                                          (0x02<<6)
#define LSM303DLHC_CTRL_REG2_A_HPM_AUTORESET_ON_INTERRUPT_EVENT                         (0x03<<6)
#define LSM303DLHC_CTRL_REG2_A_HPCF // ?
#define LSM303DLHC_CTRL_REG2_A_FDS_INTERNAL_FILTER_BAYPASSED                            (0x00<<3)
#define LSM303DLHC_CTRL_REG2_A_FDS_SEND_DATA_FROM_INTERNAL_FILTER_TO_OUTPUT_AND_FIFO    (0x01<<3)
#define LSM303DLHC_CTRL_REG2_A_HPCLICK_FILTER_BAPASSED                                  (0x00<<2)
#define LSM303DLHC_CTRL_REG2_A_HPCLICK_FILTER_ENABLED                                   (0x01<<2)
#define LSM303DLHC_CTRL_REG2_A_HPIS2_FILTER_BAPASSED                                    (0x00<<1)
#define LSM303DLHC_CTRL_REG2_A_HPIS2_FILTER_ENABLED                                     (0x01<<1)
#define LSM303DLHC_CTRL_REG2_A_HPIS1_FILTER_BAPASSED                                    (0x00<<0)
#define LSM303DLHC_CTRL_REG2_A_HPIS1_FILTER_ENABLED                                     (0x01<<0)

#define LSM303DLHC_CTRL_REG3_A_I1_CLICK_DISABLE                                         (0x00<<7)
#define LSM303DLHC_CTRL_REG3_A_I1_CLICK_ENABLE                                          (0x01<<7)
#define LSM303DLHC_CTRL_REG3_A_I1_AOI1_DISABLE                                          (0x00<<6)
#define LSM303DLHC_CTRL_REG3_A_I1_AOI1_ENABLE                                           (0x01<<6)
#define LSM303DLHC_CTRL_REG3_A_I1_AOI2_DISABLE                                          (0x00<<5)
#define LSM303DLHC_CTRL_REG3_A_I1_AOI2_ENABLE                                           (0x01<<5)
#define LSM303DLHC_CTRL_REG3_A_I1_DRDY1_DISABLE                                         (0x00<<4)
#define LSM303DLHC_CTRL_REG3_A_I1_DRDY1_ENABLE                                          (0x01<<4)
#define LSM303DLHC_CTRL_REG3_A_I1_DRDY2_DISABLE                                         (0x00<<3)
#define LSM303DLHC_CTRL_REG3_A_I1_DRDY2_ENABLE                                          (0x01<<3)
#define LSM303DLHC_CTRL_REG3_A_I1_WTM_FIFO_WATERMARK_DISABLE                            (0x00<<2)
#define LSM303DLHC_CTRL_REG3_A_I1_WTM_FIFO_WATERMARK_ENAVLE                             (0x01<<2)
#define LSM303DLHC_CTRL_REG3_A_I1_OVERRUN_FIFO_OVERRUN_DISABLE                          (0x00<<1)
#define LSM303DLHC_CTRL_REG3_A_I1_OVERRUN_FIFO_OVERRUN_ENABLE                           (0x01<<1)

#define LSM303DLHC_CTRL_REG4_A_BDU_ENABLE                                               (0x01<<7)
#define LSM303DLHC_CTRL_REG4_A_BDU_DISABLE                                              (0x00<<7)
#define LSM303DLHC_CTRL_REG4_A_BLE_LITTLE_ENDIAN                                        (0x00<<6)
#define LSM303DLHC_CTRL_REG4_A_BLE_BIG_ENDIAN                                           (0x01<<6)
#define LSM303DLHC_CTRL_REG4_A_FS_FULL_SCALE_2G                                         (0x00<<4)
#define LSM303DLHC_CTRL_REG4_A_FS_FULL_SCALE_4G                                         (0x01<<4)
#define LSM303DLHC_CTRL_REG4_A_FS_FULL_SCALE_8G                                         (0x02<<4)
#define LSM303DLHC_CTRL_REG4_A_FS_FULL_SCALE_16G                                        (0x03<<4)
#define LSM303DLHC_CTRL_REG4_A_HR_HIGH_RESOLUTION_DISABLE                               (0x00<<3)
#define LSM303DLHC_CTRL_REG4_A_HR_HIGH_RESOLUTION_ENABLE                                (0x01<<3)
#define LSM303DLHC_CTRL_REG4_A_SIM_4_WIRE_INTERFACE                                     (0x00<<0)
#define LSM303DLHC_CTRL_REG4_A_SIM_3_WIRE_INTERFACE                                     (0x01<<0)

#define LSM303DLHC_CTRL_REG5_A_BOOT                                                     (0x01<<7)
#define LSM303DLHC_CTRL_REG5_A_FIFO_EN_FIFO_ENABLE                                      (0x01<<6)
#define LSM303DLHC_CTRL_REG5_A_FIFO_EN_FIFO_DISABLE                                     (0x00<<6)
#define LSM303DLHC_CTRL_REG5_A_LIR_INT1_NOT_LATCHED                                     (0x00<<3)
#define LSM303DLHC_CTRL_REG5_A_LIR_INT1_LATCHED                                         (0x01<<3)
#define LSM303DLHC_CTRL_REG5_A_D4D_INT1_ENABLE                                          (0x01<<2)
#define LSM303DLHC_CTRL_REG5_A_D4D_INT1_DISABLE                                         (0x00<<2)
#define LSM303DLHC_CTRL_REG5_A_LIR_INT2_NOT_LATCHED                                     (0x00<<1)
#define LSM303DLHC_CTRL_REG5_A_LIR_INT2_LATCHED                                         (0x01<<1)
#define LSM303DLHC_CTRL_REG5_A_D4D_INT2_ENABLE                                          (0x01<<0)
#define LSM303DLHC_CTRL_REG5_A_D4D_INT2_DISABLE                                         (0x00<<0)

#define LSM303DLHC_CTRL_REG6_A_I2_CLICKEN_DISABLE                                       (0x00<<7)
#define LSM303DLHC_CTRL_REG6_A_I2_CLICKEN_ENABLE                                        (0x01<<7)
#define LSM303DLHC_CTRL_REG6_A_I2_INT1_INTERRUPT_1_ON_PAD2_DISABLE                      (0x00<<6)
#define LSM303DLHC_CTRL_REG6_A_I2_INT1_INTERRUPT_1_ON_PAD2_ENABLE                       (0x01<<6)
#define LSM303DLHC_CTRL_REG6_A_I2_INT2_INTERRUPT_2_ON_PAD2_DISABLE                      (0x00<<5)
#define LSM303DLHC_CTRL_REG6_A_I2_INT2_INTERRUPT_2_ON_PAD2_ENABLE                       (0x01<<5)
#define LSM303DLHC_CTRL_REG6_A_BOOT_I2_REBOOT_ON_PAD2_DISABLE                           (0x00<<4)
#define LSM303DLHC_CTRL_REG6_A_BOOT_I2_REBOOT_ON_PAD2_ENABLE                            (0x01<<4)
#define LSM303DLHC_CTRL_REG6_A_P2_ACT_ACTIVE_FUNCTION_STATUS_ON_PAD2_DISABLE            (0x00<<3)
#define LSM303DLHC_CTRL_REG6_A_P2_ACT_ACTIVE_FUNCTION_STATUS_ON_PAD2_ENABLE             (0x01<<3)
#define LSM303DLHC_CTRL_REG6_A_H_LACTIVE_ACTIVE_HIGH                                    (0x00<<1)
#define LSM303DLHC_CTRL_REG6_A_H_LACTIVE_ACTIVE_LOW                                     (0x01<<1)

#define LSM303DLHC_STATUS_REG_A_ZYXOR                                                   (0x01<<7)
#define LSM303DLHC_STATUS_REG_A_ZOR                                                     (0x01<<6)
#define LSM303DLHC_STATUS_REG_A_YOR                                                     (0x01<<5)
#define LSM303DLHC_STATUS_REG_A_XOR                                                     (0x01<<4)
#define LSM303DLHC_STATUS_REG_A_ZYXDA                                                   (0x01<<3)
#define LSM303DLHC_STATUS_REG_A_ZDA                                                     (0x01<<2)
#define LSM303DLHC_STATUS_REG_A_YDA                                                     (0x01<<1)
#define LSM303DLHC_STATUS_REG_A_XDA                                                     (0x01<<0)

// ...

#define LSM303DLHC_CRA_REG_M_TEMP_EN_TEMPERATURE_SENSOR_ENABLE                          (0x01<<7)
#define LSM303DLHC_CRA_REG_M_TEMP_EN_TEMPERATURE_SENSOR_DISABLE                         (0x00<<7)
#define LSM303DLHC_CRA_REG_M_DO_MINIMUM_DATA_OUTPUT_RATE_0_75HZ                         (0x00<<2)
#define LSM303DLHC_CRA_REG_M_DO_MINIMUM_DATA_OUTPUT_RATE_1_5HZ                          (0x01<<2)
#define LSM303DLHC_CRA_REG_M_DO_MINIMUM_DATA_OUTPUT_RATE_3_0HZ                          (0x02<<2)
#define LSM303DLHC_CRA_REG_M_DO_MINIMUM_DATA_OUTPUT_RATE_7_5HZ                          (0x03<<2)
#define LSM303DLHC_CRA_REG_M_DO_MINIMUM_DATA_OUTPUT_RATE_15HZ                           (0x04<<2)
#define LSM303DLHC_CRA_REG_M_DO_MINIMUM_DATA_OUTPUT_RATE_30HZ                           (0x05<<2)
#define LSM303DLHC_CRA_REG_M_DO_MINIMUM_DATA_OUTPUT_RATE_75HZ                           (0x06<<2)
#define LSM303DLHC_CRA_REG_M_DO_MINIMUM_DATA_OUTPUT_RATE_220HZ                          (0x07<<2)

#define LSM303DLHC_CRB_REG_M_GN_SENSOR_INPUT_RAGE_1_3GAUSS                              (0x01<<5)
#define LSM303DLHC_CRB_REG_M_GN_SENSOR_INPUT_RAGE_1_9GAUSS                              (0x02<<5)
#define LSM303DLHC_CRB_REG_M_GN_SENSOR_INPUT_RAGE_2_5GAUSS                              (0x03<<5)
#define LSM303DLHC_CRB_REG_M_GN_SENSOR_INPUT_RAGE_4_0GAUSS                              (0x04<<5)
#define LSM303DLHC_CRB_REG_M_GN_SENSOR_INPUT_RAGE_4_7GAUSS                              (0x05<<5)
#define LSM303DLHC_CRB_REG_M_GN_SENSOR_INPUT_RAGE_5_6GAUSS                              (0x06<<5)
#define LSM303DLHC_CRB_REG_M_GN_SENSOR_INPUT_RAGE_8_1GAUSS                              (0x07<<5)

#define LSM303DLHC_MR_REG_M_MD_OPERATING_MODE_CONTINUOUS_CONVERSION                     (0x00<<0)
#define LSM303DLHC_MR_REG_M_MD_OPERATING_MODE_SINGLE_CONVERSION                         (0x01<<0)
#define LSM303DLHC_MR_REG_M_MD_OPERATING_MODE_SLEEP                                     (0x02<<0)

#define LSM303DLHC_SR_REG_M_LOCK                                                        (0x01<<1)
#define LSM303DLHC_SR_REG_M_DRDY                                                        (0x01<<0)

#endif
