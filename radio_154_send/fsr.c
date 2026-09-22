#include "fsr.h"

void init_fsr(void) {
    NRF_SAADC->ENABLE = 1;
    NRF_SAADC->CH[0].PSELP = FSR_PIN;
    NRF_SAADC->CH[0].PSELN = 0;

    NRF_SAADC->CH[0].CONFIG = (0 << 0) | (0 << 4)  // bypass internal resistors
                            | (2 << 8)             // 1/4 gain
                            | (1 << 12)            // VDD/4 reference
                            | (2 << 16)            // 10 us acquisition time
                            | (0 << 20);           // single-ended mode
    NRF_SAADC->RESOLUTION = 2; // 12-bit
}

int16_t read_fsr(void) {
    static volatile int16_t result = 0;

    NRF_SAADC->RESULT.PTR = (uint32_t)&result;
    NRF_SAADC->RESULT.MAXCNT = 1;

    NRF_SAADC->TASKS_START = 1;
    while (NRF_SAADC->EVENTS_STARTED == 0);
    NRF_SAADC->EVENTS_STARTED = 0;

    NRF_SAADC->TASKS_SAMPLE = 1;
    while (NRF_SAADC->EVENTS_END == 0);
    NRF_SAADC->EVENTS_END = 0;

    NRF_SAADC->TASKS_STOP = 1;
    while (NRF_SAADC->EVENTS_STOPPED == 0);
    NRF_SAADC->EVENTS_STOPPED = 0;

    if (result < 0) {
        result = 0;
    }
    return result;
}
