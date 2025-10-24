#include "lsm303dlhc_acc.h"

static HAL_StatusTypeDef acc_write_u8(uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDR_8B, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, 50);
}

static HAL_StatusTypeDef acc_read(uint8_t reg, uint8_t *buf, uint16_t len) {
    uint8_t sub = (len > 1) ? (reg | 0x80u) : reg; // auto-increment
    return HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDR_8B, sub,
                            I2C_MEMADD_SIZE_8BIT, buf, len, 50);
}

bool lsm303_acc_read_reg(uint8_t reg, uint8_t *val) {
    if (!val) return false;
    *val = 0;
    return acc_read(reg, val, 1) == HAL_OK;
}

bool lsm303_acc_init_50hz(void)
{
    uint8_t who = 0;
    if (acc_read(LSM303_WHO_AM_I_A, &who, 1) != HAL_OK) return false;
    if (who != 0x33) return false;

    // 50Hz, XYZ enable
    if (acc_write_u8(LSM303_CTRL_REG1_A, 0x47) != HAL_OK) return false;
    // BDU=1, HR=1, ±2g
    if (acc_write_u8(LSM303_CTRL_REG4_A, 0x88) != HAL_OK) return false;

    // optional cleanup
    (void)acc_write_u8(LSM303_CTRL_REG2_A, 0x00);
    (void)acc_write_u8(LSM303_CTRL_REG3_A, 0x00);
    (void)acc_write_u8(LSM303_CTRL_REG5_A, 0x00);

    HAL_Delay(10);
    return true;
}

bool lsm303_acc_read_mg(float *ax, float *ay, float *az)
{
    if (!ax || !ay || !az) return false;
    uint8_t raw[6];
    if (acc_read(LSM303_OUT_X_L_A, raw, 6) != HAL_OK) return false;

    // High-resolution: 12-bit left-justified
    int16_t x = (int16_t)((raw[1] << 8) | raw[0]) >> 4;
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]) >> 4;
    int16_t z = (int16_t)((raw[5] << 8) | raw[4]) >> 4;

    // 1 mg/LSB in ±2g HR
    *ax = (float)x;
    *ay = (float)y;
    *az = (float)z;
    return true;
}
