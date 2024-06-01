#ifndef HMC5883L_REGS_H
#define HMC5883L_REGS_H

#define HMC5883L_ADDR				0x1E

#define HMC5883L_REG_CONFIG_A		0x00
#define HMC5883L_REG_CONFIG_B		0x01
#define HMC5883L_REG_MODE			0x02
#define HMC5883L_REG_DATA_X_MSB		0x03
#define HMC5883L_REG_DATA_X_LSB		0x04
#define HMC5883L_REG_DATA_Z_MSB		0x05
#define HMC5883L_REG_DATA_Z_LSB		0x06
#define HMC5883L_REG_DATA_Y_MSB		0x07
#define HMC5883L_REG_DATA_Y_LSB		0x08
#define HMC5883L_REG_STATUS			0x09
#define HMC5883L_REG_IDENT_A		0x0A
#define HMC5883L_REG_IDENT_B		0x0B
#define HMC5883L_REG_IDENT_C		0x0C

#define HMC5883L_CONFIG_A_SAMPLES_1		(0x00<<5)
#define HMC5883L_CONFIG_A_SAMPLES_2		(0x01<<5)
#define HMC5883L_CONFIG_A_SAMPLES_4		(0x02<<5)
#define HMC5883L_CONFIG_A_SAMPLES_8 	(0x03<<5)
#define HMC5883L_CONFIG_A_RATE_0_75HZ	(0x00<<2)
#define HMC5883L_CONFIG_A_RATE_1_5HZ	(0x01<<2)
#define HMC5883L_CONFIG_A_RATE_3HZ		(0x02<<2)
#define HMC5883L_CONFIG_A_RATE_7_5HZ	(0x03<<2)
#define HMC5883L_CONFIG_A_RATE_15HZ		(0x04<<2)
#define HMC5883L_CONFIG_A_RATE_30HZ		(0x05<<2)
#define HMC5883L_CONFIG_A_RATE_75		(0x06<<2)
#define HMC5883L_CONFIG_A_MEAS_NORMAL	(0x00<<0)
#define HMC5883L_CONFIG_A_MEAS_POSITIVE	(0x01<<0)
#define HMC5883L_CONFIG_A_MEAS_NEGATIVE	(0x02<<0)

#define HMC5883L_CONFIG_B_RANGE_0_88GA	(0x00<<5)
#define HMC5883L_CONFIG_B_RANGE_1_3GA	(0x01<<5)
#define HMC5883L_CONFIG_B_RANGE_1_9GA	(0x02<<5)
#define HMC5883L_CONFIG_B_RANGE_2_5GA	(0x03<<5)
#define HMC5883L_CONFIG_B_RANGE_4GA		(0x04<<5)
#define HMC5883L_CONFIG_B_RANGE_4_7GA	(0x05<<5)
#define HMC5883L_CONFIG_B_RANGE_5_6GA	(0x06<<5)
#define HMC5883L_CONFIG_B_RANGE_8_1GA	(0x07<<5)

#define HMC5883L_MODE_CONTINOUS			(0x00<<0)
#define HMC5883L_MODE_SINGLE			(0x01<<0)
#define HMC5883L_MODE_IDLE				(0x02<<0)

#endif
