#pragma once

#include <stdint.h>
#include "nrf.h"

#define FSR_PIN 1 // AIN0

void init_fsr(void);
int16_t read_fsr(void);
