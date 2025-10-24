#pragma once
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;
/* ---- I2C address (7-bit) ----
   Set to 0x19 if SA0/SDO is tied HIGH (on Disco it is).
   If your board ties SA0 LOW, change to 0x18. */
#ifndef LSM303_ACC_ADDR_7B
#define LSM303_ACC_ADDR_7B  0x19
#endif
#define LSM303_ACC_ADDR_8B  (LSM303_ACC_ADDR_7B << 1)

/* ---- Registers ---- */
#define LSM303_WHO_AM_I_A   0x0F
#define LSM303_CTRL_REG1_A  0x20
#define LSM303_CTRL_REG2_A  0x21
#define LSM303_CTRL_REG3_A  0x22
#define LSM303_CTRL_REG4_A  0x23
#define LSM303_CTRL_REG5_A  0x24
#define LSM303_OUT_X_L_A    0x28  // auto-increment from here

/* Bring in the I2C handle from main.c */
extern I2C_HandleTypeDef hi2c1;

/* API */
bool lsm303_acc_init_50hz(void);
bool lsm303_acc_read_mg(float *ax, float *ay, float *az);
bool lsm303_acc_read_reg(uint8_t reg, uint8_t *val);
