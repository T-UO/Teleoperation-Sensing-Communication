// BNO055 Position Sensor

#pragma once

#include "nrf_twi_mngr.h"

// Chip addresses
static const uint8_t BNO055_ADDRESS = 0x28;

// Measurement data type
typedef struct {
  float roll;
  float pitch;
  float yaw;
} bno055_euler_t;

// Register definitions for BNO055 chip
typedef enum {
  BNO055_CHIP_ID_ADDR = 0x00,
  BNO055_EULER_YAW_L_ADDR = 0x1A,
  BNO055_EULER_YAW_H_ADDR = 0x1B,
  BNO055_EULER_ROLL_L_ADDR = 0x1C,
  BNO055_EULER_ROLL_H_ADDR = 0x1D,
  BNO055_EULER_PITCH_L_ADDR = 0x1E,
  BNO055_EULER_PITCH_H_ADDR = 0x1F,
  BNO055_CALIB_STAT_ADDR  = 0x35,
  BNO055_OPR_MODE_ADDR = 0x3D,
  BNO055_PWR_MODE_ADDR = 0x3E,
  BNO055_SYS_TRIGGER_ADDR = 0x3F,
} bno055_reg_t;

// Operating modes
typedef enum {
  OPR_MODE_CONFIG = 0x00,
  OPR_MODE_IMU = 0x08, // fusion mode accel + gyro, no mag
  OPR_MODE_NDOF = 0x0C, // 9 dof full fusion
} bno055_opr_mode_t;

// Function prototypes
void bno055_init(const nrf_twi_mngr_t*);
bno055_euler_t get_wrist_angle(void);
uint8_t get_calibration_status(void);