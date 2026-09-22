// BNO085 Position Sensor

#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_timer.h"
#include "microbit_v2.h"
#include "nrf_delay.h"
#include "nrf_twi_mngr.h"
#include "nrfx_uarte.h"

#define FOREARM_RX_PIN EDGE_P12
#define UPPERARM_RX_PIN EDGE_P16
#define DEG_SCALE 0.01f
#define MG_TO_MS2 0.00980665f
#define UARTE_DISABLE 0
#define UARTE_ENABLE 8

extern const nrfx_uarte_t UARTE_INST;

// Measurement data type
typedef struct {
  float yaw;
  float pitch;
  float roll;
  float x_accel;
  float y_accel;
  float z_accel;
} bno085_rvc_t;

typedef enum { SENSOR_FOREARM, SENSOR_UPPERARM } active_sensor_t;

// Function prototypes
void uarte_init(uint32_t pin);
bool bno085_rvc_packet(const uint8_t *, bno085_rvc_t *);
void bno085_select_rx_pin(uint32_t);
void get_bno085_data(void);
void get_bno085_latest(bno085_rvc_t *, bno085_rvc_t *);
void bno085_switch_sensor(void);
