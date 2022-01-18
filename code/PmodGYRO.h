#ifndef PMODGYRO_H
#define PMODGYRO_H

// Register Addresses
#define GYRO_WHO_AM_I      0x0F
#define GYRO_CTRL_REG1     0x20
#define GYRO_CTRL_REG2     0x21
#define GYRO_CTRL_REG3     0x22
#define GYRO_CTRL_REG4     0x23
#define GYRO_CTRL_REG5     0x24
#define GYRO_REFERENCE     0x25
#define GYRO_OUT_TEMP      0x26
#define GYRO_STATUS_REG    0x27
#define GYRO_OUT_X_L       0x28
#define GYRO_OUT_X_H       0x29
#define GYRO_OUT_Y_L       0x2A
#define GYRO_OUT_Y_H       0x2B
#define GYRO_OUT_Z_L       0x2C
#define GYRO_OUT_Z_H       0x2D
#define GYRO_FIFO_CTRL_REG 0x2E
#define GYRO_FIFO_SRC_REG  0x2F
#define GYRO_INT1_CFG      0x30
#define GYRO_INT1_SRC      0x31
#define GYRO_INT1_TSH_XH   0x32
#define GYRO_INT1_TSH_XL   0x33
#define GYRO_INT1_TSH_YH   0x34
#define GYRO_INT1_TSH_YL   0x35
#define GYRO_INT1_TSH_ZH   0x36
#define GYRO_INT1_TSH_ZL   0x37
#define GYRO_INT1_DURATION 0x38

// CTRL_REG1
#define GYRO_REG1_DR1 0x80
#define GYRO_REG1_DR0 0x40
#define GYRO_REG1_BW1 0x20
#define GYRO_REG1_BW0 0x10
#define GYRO_REG1_PD  0x08
#define GYRO_REG1_ZEN 0x04
#define GYRO_REG1_YEN 0x02
#define GYRO_REG1_XEN 0x01

// CTRL_REG3
#define GYRO_REG3_I1_INT1   0x80
#define GYRO_REG3_I1_BOOT   0x40
#define GYRO_REG3_H_LACTIVE 0x20
#define GYRO_REG3_PP_OD     0x10
#define GYRO_REG3_I2_DRDY   0x08
#define GYRO_REG3_I2_WTM    0x04
#define GYRO_REG3_I2_ORUN   0x02
#define GYRO_REG3_I2_EMPTY  0x01

// GYRO_INT1_CFG
#define GYRO_INT1_ANDOR 0x80
#define GYRO_INT1_LIR   0x40
#define GYRO_INT1_ZHIE  0x20
#define GYRO_INT1_ZLIE  0x10
#define GYRO_INT1_YHIE  0x08
#define GYRO_INT1_YLIE  0x04
#define GYRO_INT1_XHIE  0x02
#define GYRO_INT1_XLIE  0x01

// GYRO Data Mode
#define GYRO_BYPASS 	0x00
#define GYRO_FIFO		0x10
#define GYRO_STREAM 	0x20


#endif // PMODGYRO_H
