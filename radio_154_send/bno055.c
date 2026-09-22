// BNO055 driver for Microbit_v2 -- wrist orientation sensor
//
// Initializes sensor and communicates over I2C
// Capable of reading temperature, acceleration, and magnetic field strength

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "bno055.h"
#include "nrf_delay.h"

#define NRF_TWI_MNGR_NO_STOP 0x01

// Pointer to an initialized I2C instance to use for transactions
static const nrf_twi_mngr_t *i2c_manager = NULL;
static bool bno055_verified = false;

// Helper function to perform a 1-byte I2C read of a given register
static uint8_t i2c_reg_read(uint8_t reg_addr) {
  uint8_t rx_buf = 0;
  nrf_twi_mngr_transfer_t const read_transfer[] = {
      // TODO: implement me
      NRF_TWI_MNGR_WRITE(BNO055_ADDRESS, &reg_addr, 1, NRF_TWI_MNGR_NO_STOP),
      NRF_TWI_MNGR_READ(BNO055_ADDRESS, &rx_buf, 1, 0),
  };
  ret_code_t result =
      nrf_twi_mngr_perform(i2c_manager, NULL, read_transfer, 2, NULL);
  if (result != NRF_SUCCESS) {
    // Likely error codes:
    //  NRF_ERROR_INTERNAL            (0x0003) - something is wrong with the
    //  driver itself NRF_ERROR_INVALID_ADDR        (0x0010) - buffer passed was
    //  in Flash instead of RAM NRF_ERROR_BUSY                (0x0011) - driver
    //  was busy with another transfer still NRF_ERROR_DRV_TWI_ERR_OVERRUN
    //  (0x8200) - data was overwritten during the transaction
    //  NRF_ERROR_DRV_TWI_ERR_ANACK   (0x8201) - i2c device did not acknowledge
    //  its address NRF_ERROR_DRV_TWI_ERR_DNACK   (0x8202) - i2c device did not
    //  acknowledge a data byte
    printf("I2C transaction failed! Error: %lX\n", result);
  }

  return rx_buf;
}

// Helper function to perform a 1-byte I2C write of a given register
static void i2c_reg_write(uint8_t reg_addr, uint8_t data) {
  uint8_t const values[] = {
      reg_addr,
      data,
  };
  nrf_twi_mngr_transfer_t const read_transfer[] = {
      NRF_TWI_MNGR_WRITE(BNO055_ADDRESS, &values, 2, 0),
  };
  ret_code_t result =
      nrf_twi_mngr_perform(i2c_manager, NULL, read_transfer, 1, NULL);
  if (result != NRF_SUCCESS) {
    printf("I2C transaction failed! Error: %lX\n", result);
  }
}

static bool i2c_burst_read(uint8_t start_reg, uint8_t *p_rx_buf, uint8_t len) {
  nrf_twi_mngr_transfer_t const read_transfer[] = {
      NRF_TWI_MNGR_WRITE(BNO055_ADDRESS, &start_reg, 1, NRF_TWI_MNGR_NO_STOP),
      NRF_TWI_MNGR_READ(BNO055_ADDRESS, p_rx_buf, len, 0),
  };
  ret_code_t result =
      nrf_twi_mngr_perform(i2c_manager, NULL, read_transfer, 2, NULL);
  if (result != NRF_SUCCESS) {
    printf("I2C burst read failed! Error: %lX\n", result);
    return false;
  }
  return true;
}

// Initialize and configure BNO055
// i2c - pointer to already initialized and enabled twim instance
void bno055_init(const nrf_twi_mngr_t *i2c) {
  i2c_manager = i2c;

  // chip configuration fusion mode - autocontrol
  //  ---Initialize Chip---
  i2c_reg_write(BNO055_OPR_MODE_ADDR, OPR_MODE_CONFIG);
  nrf_delay_ms(25);

  i2c_reg_write(BNO055_SYS_TRIGGER_ADDR, 0x20); // reset sensor
  nrf_delay_ms(750);

  uint8_t chip_id = i2c_reg_read(BNO055_CHIP_ID_ADDR);
  if (chip_id == 0xA0) {
    printf("BNO055 verified - chip ID: 0x%X\n", chip_id);
    bno055_verified = true;
  } else {
    printf("BNO055 identity conflict, expected 0xA0, got: 0x%X\n", chip_id);
    bno055_verified = false;
  }

  i2c_reg_write(BNO055_PWR_MODE_ADDR, 0x00); // normal Power Mode
  nrf_delay_ms(10);

  i2c_reg_write(BNO055_SYS_TRIGGER_ADDR,
                0x80); // use adafruit crystal oscillator
  nrf_delay_ms(600);

  i2c_reg_write(BNO055_OPR_MODE_ADDR, OPR_MODE_IMU); // set to IMU fusion mode
  nrf_delay_ms(50);
}

bno055_euler_t get_wrist_angle(void) {

  bno055_euler_t angles = {0.0f, 0.0f, 0.0f};
  uint8_t raw_buf[6] = {0};
  static uint16_t read_count = 0;
  static uint16_t zero_count = 0;

  bool read_ok = i2c_burst_read(BNO055_EULER_YAW_L_ADDR, raw_buf, 6);
  read_count++;
  if (!read_ok) {
    if ((read_count % 10) == 0) {
      printf("BNO055 wrist read failing; WR/WP/WY will be zero\n");
    }
    return angles;
  }

  int16_t yaw_raw = (int16_t)((raw_buf[1] << 8) | raw_buf[0]);
  int16_t roll_raw = (int16_t)((raw_buf[3] << 8) | raw_buf[2]);
  int16_t pitch_raw = (int16_t)((raw_buf[5] << 8) | raw_buf[4]);

  if (yaw_raw == 0 && roll_raw == 0 && pitch_raw == 0) {
    zero_count++;
    if ((zero_count % 10) == 0) {
      uint8_t cal = get_calibration_status();
      uint8_t mode = i2c_reg_read(BNO055_OPR_MODE_ADDR);
      printf("BNO055 wrist Euler still all zero; verified=%u cal=0x%02X mode=0x%02X\n",
             bno055_verified ? 1 : 0, cal, mode);
    }
  } else {
    zero_count = 0;
  }

  angles.yaw = (float)yaw_raw / 16.0f;
  angles.roll = (float)roll_raw / 16.0f;
  angles.pitch = (float)pitch_raw / 16.0f;

  // printf("wrist r: %.2f deg, p: %.2f deg, y: %.2f deg\n", angles.roll,
  // angles.pitch, angles.yaw);
  return angles;
}

uint8_t get_calibration_status(void) {
  /* 0: uncalibrated
      1: partially calibrated
      2: mostly calibrated
      3: fully calibrated
      we need to check if the sensors are ready for teleoperation

      uint8_t status = bno055_get_calibration_status();

      uint8_t sys_cal  = (status >> 6) & 0x03;
      uint8_t gyro_cal = (status >> 4) & 0x03;
      uint8_t acc_cal  = (status >> 2) & 0x03;

      We need gyro & acc to be 3, sys doesn't really matter
  */
  return i2c_reg_read(BNO055_CALIB_STAT_ADDR);
}
