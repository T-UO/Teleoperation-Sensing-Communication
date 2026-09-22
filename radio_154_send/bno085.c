/**
 * @file bno085.c
 * @brief Driver packet parser for BNO085 UART-RVC mode on nRF52833.
 *
 * @note Adapted from the Adafruit BNO08x RVC Arduino Library (v1.0.0)
 *       by Bryan Siepert for Adafruit Industries.
 *       Original source: https://github.com/adafruit/Adafruit_BNO08x_RVC.git
 *
 * Ported from Arduino C++ to native nRF C for use with the BBC micro:bit v2.
 */
#include "bno085.h"
#include "nrf_nvic.h"

static active_sensor_t current_sensor = SENSOR_FOREARM;
static bno085_rvc_t forearm_data = {0};
static bno085_rvc_t upperarm_data = {0};
static bool forearm_ready = false;
static bool upperarm_ready = false;
static uint8_t packet_buffer[19];
static uint8_t rx_byte = 0;
static uint8_t buffer_index = 0;

volatile bno085_rvc_t latest_forearm_data = {0};
volatile bno085_rvc_t latest_upperarm_data = {0};

// UART initialization function
void uarte_init(uint32_t pin_num) {
  nrfx_uarte_config_t uarte_config = {
      .pseltxd = NRF_UARTE_PSEL_DISCONNECTED,
      .pselrxd = pin_num,
      .pselcts = NRF_UARTE_PSEL_DISCONNECTED,
      .pselrts = NRF_UARTE_PSEL_DISCONNECTED,
      .p_context = NULL,
      .hwfc = NRF_UARTE_HWFC_DISABLED,
      .parity = NRF_UARTE_PARITY_EXCLUDED,
      .baudrate = NRF_UARTE_BAUDRATE_115200,
      .interrupt_priority = NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY,
  };

  // Note: without a callback handler, transfers are blocking
  nrfx_err_t result = nrfx_uarte_init(&UARTE_INST, &uarte_config, NULL);
  APP_ERROR_CHECK(result);
}

// helper function to get bno085 data
bool bno085_rvc_packet(const uint8_t *buffer, bno085_rvc_t *output) {
  if (!buffer || !output) {
    return false;
  }

  if (buffer[0] != 0xAA ||
      buffer[1] != 0xAA) { // each packet prefixed with header 0xAAAA
    return false;
  }

  uint8_t csum = 0; // checksum is sum of bytes 2 thru 17
  for (uint8_t i = 2; i < 18; i++) {
    csum += buffer[i];
  }

  if (csum != buffer[18]) {
    return false;
  }

  int16_t raw_data[6];
  for (uint8_t i = 0; i < 6; i++) {
    uint8_t low_byte = buffer[3 + (i * 2)];
    uint8_t high_byte = buffer[3 + (i * 2) + 1];

    raw_data[i] = (int16_t)(low_byte | (high_byte << 8));
  }

  output->yaw = (float)raw_data[0] * DEG_SCALE;
  output->pitch = (float)raw_data[1] * DEG_SCALE;
  output->roll = (float)raw_data[2] * DEG_SCALE;

  output->x_accel = (float)raw_data[3] * MG_TO_MS2;
  output->y_accel = (float)raw_data[4] * MG_TO_MS2;
  output->z_accel = (float)raw_data[5] * MG_TO_MS2;

  return true;
}

// helper function for UART virtualization
void bno085_select_rx_pin(uint32_t pin) {
  NRF_UARTE1->ENABLE = UARTE_DISABLE; // disable uart
  NRF_UARTE1->PSEL.RXD = pin;         // select new pin
  NRF_UARTE1->ENABLE = UARTE_ENABLE;  // enable new pin
}

// gets data for both sensors utilizing UART virtualization
void get_bno085_data(void) {
  // check UART status
  nrfx_err_t result = nrfx_uarte_rx(&UARTE_INST, &rx_byte, 1);

  if (result != NRFX_SUCCESS) { // clear error flag and reset buffer if there's
                                // a hardware error
    NRF_UARTE1->EVENTS_ERROR = 0;
    volatile uint32_t dummy = NRF_UARTE1->ERRORSRC;
    NRF_UARTE1->ERRORSRC = dummy;

    buffer_index = 0;
    nrf_delay_us(10);
    return;
  }

  if (result == NRFX_SUCCESS) { // if no errors

    if (buffer_index == 0) { // check data header 0xAAAA to proceed with parsing
      if (rx_byte == 0xAA) {
        packet_buffer[buffer_index++] = rx_byte;
      }
    } else if (buffer_index == 1) {
      if (rx_byte == 0xAA) {
        packet_buffer[buffer_index++] = rx_byte;
      } else {
        buffer_index = 0;
      }
    } else {
      packet_buffer[buffer_index++] = rx_byte;

      if (buffer_index == 19) { // 19 bytes per packet
        bool packet_valid = false;

        if (current_sensor == SENSOR_FOREARM) {
          packet_valid = bno085_rvc_packet(
              packet_buffer, &forearm_data); // forearm packet received
          if (packet_valid)
            forearm_ready = true;
        } else {
          packet_valid = bno085_rvc_packet(
              packet_buffer, &upperarm_data); // upperarm packet received
          if (packet_valid)
            upperarm_ready = true;
        }

        buffer_index = 0;

        if (packet_valid) {

          if (current_sensor == SENSOR_FOREARM) {
            latest_forearm_data = forearm_data;
          } else {
            latest_upperarm_data = upperarm_data;
          }

          // if (forearm_ready && upperarm_ready) {
          //         printf("sensor1: yaw: %.2f, pitch: %.2f, roll: %.2f |
          //         sensor2: yaw: %.2f, pitch: %.2f, roll: %.2f\n",
          //                forearm_data.yaw, forearm_data.pitch,
          //                forearm_data.roll, upperarm_data.yaw,
          //                upperarm_data.pitch, upperarm_data.roll);

          //         forearm_ready = false;
          //         upperarm_ready = false;
          //     }
        } else {
          //   printf("packet sync/checksum error on %s\n",
          //        (current_sensor == SENSOR_FOREARM) ? "forearm" :
          //        "upperarm");
        }
      }
    }
  }
}

void bno085_switch_sensor(void) {
  nrfx_uarte_uninit(&UARTE_INST);

  if (current_sensor == SENSOR_FOREARM) {
    current_sensor = SENSOR_UPPERARM;
    uarte_init(UPPERARM_RX_PIN);
  } else {
    current_sensor = SENSOR_FOREARM;
    uarte_init(FOREARM_RX_PIN);
  }

  buffer_index = 0;
  nrf_delay_ms(1);
}

void get_bno085_latest(bno085_rvc_t *forearm_latest,
                       bno085_rvc_t *upperarm_latest) {
  __disable_irq();
  *forearm_latest = latest_forearm_data;
  *upperarm_latest = latest_upperarm_data;
  __enable_irq();
}
