// Radio 15.4 send app
//
// Sends wireless packets via the 802.15.4 radio

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bno055.h"
#include "bno085.h"
#include "fsr.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

// Pin configurations
#include "microbit_v2.h"

#include "nrf_802154.h"

#define PSDU_MAX_SIZE (127) // Max length of a packet
#define FCS_LENGTH (2)      // Length of the Frame Control Sequence
#define FSR_PRESSED_THRESHOLD 3000
#define PACKET_PERIOD_MS 20

#include "nrf_twi_mngr.h"

const nrfx_uarte_t UARTE_INST = NRFX_UARTE_INSTANCE(1);

NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// callback fn when tx starts

void nrf_802154_tx_started(const uint8_t *p_frame) { printf("tx started\n"); }

// callback fn when tx fails
void nrf_802154_transmit_failed(const uint8_t *p_frame,
                                nrf_802154_tx_error_t error) {
  printf("tx failed error %u!\n", error);
}

// callback fn for successful tx
void nrf_802154_transmitted_raw(const uint8_t *p_frame, uint8_t *p_ack,
                                int8_t power, uint8_t lqi) {
  printf("frame was transmitted!\n");
}

int main(void) {

  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = I2C_QWIIC_SCL;
  i2c_config.sda = I2C_QWIIC_SDA;

  nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  nrf_delay_ms(500);

  bno055_init(&twi_mngr_instance);
  nrf_delay_ms(1000);
  init_fsr();

  // Initialize forearm BNO085 over UART
  uarte_init(FOREARM_RX_PIN);
  nrf_delay_ms(1000);
  printf("Board started!\n");

  // Initialize.
  nrf_gpio_cfg_output(LED_MIC);

  // Configure 154 radio
  printf("About to init\n");
  nrf_802154_init();
  printf("Done with init\n");
  nrf_802154_channel_set(11);
  uint8_t src_pan_id[] = {0xcd, 0xab};
  nrf_802154_pan_id_set(src_pan_id);
  printf("Radio configured!\n");

  // Addresses (source and destination)
  uint8_t src_extended_addr[] = {0xdc, 0xa9, 0x35, 0x7b,
                                 0x73, 0x36, 0xce, 0xf4};
  nrf_802154_extended_address_set(src_extended_addr);
  uint8_t dst_extended_addr[] = {0x50, 0xbe, 0xca, 0xc3,
                                 0x3c, 0x36, 0xce, 0xf4};

  // TX Packet

  //  uint8_t pkt[PSDU_MAX_SIZE];
  //  pkt[0] = 26 + FCS_LENGTH; /* Length for nrf_transmit (length of pkt + FCS)

  uint8_t pkt[PSDU_MAX_SIZE];
  pkt[1] = 0x01;                          /* Frame Control Field */
  pkt[2] = 0xcc;                          /* Frame Control Field */
  pkt[3] = 0x00;                          /* Sequence number */
  pkt[4] = 0xff;                          /* Destination PAN ID 0xffff */
  pkt[5] = 0xff;                          /* Destination PAN ID */
  memcpy(&pkt[6], dst_extended_addr, 8);  /* Destination extended address */
  memcpy(&pkt[14], src_pan_id, 2);        /* Source PAN ID */
  memcpy(&pkt[16], src_extended_addr, 8); /* Source extended address */

  //  const char *payload = "S1:90,S2:45,S3:120,S4:80,S5:60,S6:20";
  //  const char *payload = "Tagbo";

  char payload[120];
  //  float roll = 12.34;
  //  float yaw = 7.56;
  //  float pitch = -9.0;

  // Enter main loop.
  while (1) {

    // Wrist BNO055 data
    bno055_euler_t wrist = get_wrist_angle();

    // Forearm BNO085 data
    for (int i = 0; i < 40; i++) {
      get_bno085_data();
    }

    bno085_switch_sensor();

    bno085_rvc_t forearm;
    bno085_rvc_t upperarm;
    get_bno085_latest(&forearm, &upperarm);
    int16_t fsr_value = read_fsr();
    int gripper_closed = fsr_value > FSR_PRESSED_THRESHOLD ? 1 : 0;

    snprintf(payload, sizeof(payload),
             "WR:%.1f,WP:%.1f,WY:%.1f,FR:%.1f,FP:%.1f,FY:%.1f,UR:%.1f,UP:%.1f,"
             "UY:%.1f,G:%d",
             wrist.roll, wrist.pitch, wrist.yaw, forearm.roll, forearm.pitch,
             forearm.yaw, upperarm.roll, upperarm.pitch, upperarm.yaw,
             gripper_closed);

    //    static int count = 0;

    //  float roll = 10.0f + count;
    //  float pitch = 20.0f + count;
    //    float yaw = 30.0f + count;

    //  snprintf(payload, sizeof(payload), "R:%.2f,P:%.2f,Y:%.2f", roll, pitch,
    //         yaw);

    uint8_t payload_len = strlen(payload);

    pkt[0] = 23 + payload_len + FCS_LENGTH;
    pkt[3]++; // sequence number
    memcpy(&pkt[24], payload, payload_len);

    printf("TX: %s\n", payload);

    if (!nrf_802154_transmit_raw(pkt, true)) {
      printf("Failure to send radio packet!\n");
    } else {
      printf("Sent a radio packet!\n");
    }

    //   nrf_gpio_pin_toggle(LED_MIC);
    // count++;
    nrf_delay_ms(PACKET_PERIOD_MS);
  }
}
