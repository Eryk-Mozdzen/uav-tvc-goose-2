#ifndef QMC5883L_REGS_H
#define QMC5883L_REGS_H

#define QMC5883L_ADDR				    0x0D

#define QMC5883L_REG_DATA_OUTPUT_X_LSB  0x00
#define QMC5883L_REG_DATA_OUTPUT_X_MSB  0x01
#define QMC5883L_REG_DATA_OUTPUT_Y_LSB  0x02
#define QMC5883L_REG_DATA_OUTPUT_Y_MSB  0x03
#define QMC5883L_REG_DATA_OUTPUT_Z_LSB  0x04
#define QMC5883L_REG_DATA_OUTPUT_Z_MSB  0x05
#define QMC5883L_REG_STATUS     		0x06
#define QMC5883L_REG_TOUT_1             0x07
#define QMC5883L_REG_TOUT_2             0x08
#define QMC5883L_REG_CONTROL_1          0x09
#define QMC5883L_REG_CONTROL_2          0x0A
#define QMC5883L_REG_SET_RESET          0x0B
#define QMC5883L_REG_CHIP_ID            0x0D

#define QMC5883L_STATUS_DRDY                (0x01<<0)
#define QMC5883L_STATUS_OVL                 (0x01<<1)
#define QMC5883L_STATUS_DOR                 (0x01<<2)

#define QMC5883L_CONFIG_1_MODE_STANDBY		(0x00<<0)
#define QMC5883L_CONFIG_1_MODE_CONTINOUS	(0x01<<0)
#define QMC5883L_CONFIG_1_ODR_10HZ	        (0x00<<2)
#define QMC5883L_CONFIG_1_ODR_50HZ	        (0x01<<2)
#define QMC5883L_CONFIG_1_ODR_100HZ		    (0x02<<2)
#define QMC5883L_CONFIG_1_ODR_200HZ	        (0x03<<2)
#define QMC5883L_CONFIG_1_RNG_2G		    (0x00<<4)
#define QMC5883L_CONFIG_1_RNG_8G		    (0x01<<4)
#define QMC5883L_CONFIG_1_OSR_512           (0x00<<6)
#define QMC5883L_CONFIG_1_OSR_256	        (0x01<<6)
#define QMC5883L_CONFIG_1_OSR_128	        (0x02<<6)
#define QMC5883L_CONFIG_1_OSR_64	        (0x03<<6)

#define QMC5883L_CONFIG_2_INT_ENB_ENABLE	(0x00<<0)
#define QMC5883L_CONFIG_2_INT_ENB_DISABLE	(0x01<<0)
#define QMC5883L_CONFIG_2_ROL_PNT_NORMAL	(0x00<<6)
#define QMC5883L_CONFIG_2_ROL_PNT_ENABLE	(0x01<<6)
#define QMC5883L_CONFIG_2_SOFT_RST	        (0x01<<7)

#define QMC5883L_SET_RESET_RECOMMENDED	    0x01

#define QMC5883L_CHIP_ID_VALUE	            0xFF

#endif
