// Radio 15.4 receive app
//
// Receives wireless IMU packets via the 802.15.4 radio, converts them to raw
// Feetech WritePosEx packets, and prints those packets over USB serial for the
// host sync_engine to forward to the robot arm.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

// Pin configurations
#include "microbit_v2.h"

#include "nrf_802154.h"

#define PSDU_MAX_SIZE (127) // Max length of a packet
#define FCS_LENGTH (2)      // Length of the Frame Control Sequence
#define LATEST_PAYLOAD_SIZE 128
#define COMMAND_PERIOD_MS 33
#define FEETECH_INST_WRITE 0x03
#define FEETECH_ACC 0x29
#define MOVE_SPEED 200
#define MOVE_ACCEL 20

#define SERVO_SHOULDER_PAN 1
#define SERVO_SHOULDER_LIFT 2
#define SERVO_ELBOW_FLEX 3
#define SERVO_WRIST_FLEX 4
#define SERVO_WRIST_ROLL 5
#define SERVO_GRIPPER 6

#define ARM_COUNTS_PER_DEG 11.378f
#define ARM_DEADBAND_DEG 0.5f
#define ARM_MAX_STEP_TICKS 40
#define PITCH_DEG_MIN -90.0f
#define PITCH_DEG_MAX 90.0f
#define ROLL_DEG_MIN -180.0f
#define ROLL_DEG_MAX 180.0f
#define SHOULDER_PAN_HOME 2047
#define SHOULDER_PAN_MIN 1000
#define SHOULDER_PAN_MAX 2900
#define ARM_UP_DEFAULT_MIN -60.0f
#define ARM_UP_DEFAULT_MAX -40.0f
#define ARM_UP_FULL_FLEX 110.0f
#define ARM_UP_SERVO2_TRAVEL_DEG 150.0f
#define ARM_UP_SERVO3_TRAVEL_DEG 150.0f
#define SHOULDER_LIFT_HOME 2050
#define SHOULDER_LIFT_MIN 2025
#define SHOULDER_LIFT_MAX 3800
#define ELBOW_FLEX_HOME 2052
#define ELBOW_FLEX_MIN 300
#define ELBOW_FLEX_MAX 3895
#define WRIST_FLEX_MIN 2100
#define WRIST_FLEX_MAX 4095
#define WRIST_ROLL_MIN 100
#define WRIST_ROLL_MAX 4095
#define GRIPPER_OPEN 2090
#define GRIPPER_CLOSED 3504

typedef struct {
  float wr;
  float wp;
  float wy;
  float fr;
  float fp;
  float fy;
  float ur;
  float up;
  float uy;
  int gripper;
  bool has_gripper;
} radio_pose_t;

typedef struct {
  char payload[LATEST_PAYLOAD_SIZE];
  uint8_t len;
  volatile bool pending;
} latest_payload_t;

static int shoulder_lift_cur = SHOULDER_LIFT_HOME;
static int elbow_flex_cur = ELBOW_FLEX_HOME;
static bool forearm_yaw_zero_captured = false;
static float forearm_yaw_zero = 0.0f;
static latest_payload_t latest = {0};

static uint16_t map_angle(float deg, float dmin, float dmax, uint16_t pmin,
                          uint16_t pmax) {
  if (deg < dmin) deg = dmin;
  if (deg > dmax) deg = dmax;
  float t = (deg - dmin) / (dmax - dmin);
  return (uint16_t)((float)pmin + t * (float)(pmax - pmin));
}

static int clampi(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static int step_toward(int cur, int target) {
  int delta = target - cur;
  if (delta > ARM_MAX_STEP_TICKS) delta = ARM_MAX_STEP_TICKS;
  if (delta < -ARM_MAX_STEP_TICKS) delta = -ARM_MAX_STEP_TICKS;
  return cur + delta;
}

static float upper_pitch_flex_fraction(float upper_pitch) {
  if (upper_pitch < ARM_UP_DEFAULT_MIN) return 0.0f;
  if (upper_pitch <= ARM_UP_DEFAULT_MAX) return 0.0f;
  if (upper_pitch >= ARM_UP_FULL_FLEX) return 1.0f;
  return (upper_pitch - ARM_UP_DEFAULT_MAX) /
         (ARM_UP_FULL_FLEX - ARM_UP_DEFAULT_MAX);
}

static uint8_t feetech_checksum(const uint8_t *packet, uint8_t start,
                                uint8_t end_exclusive) {
  uint8_t sum = 0;
  for (uint8_t i = start; i < end_exclusive; i++) {
    sum += packet[i];
  }
  return (uint8_t)~sum;
}

static void emit_byte(uint8_t b) { putchar((int)b); }

static void emit_write_pos_ex(uint8_t id, uint16_t position, uint16_t speed,
                              uint8_t accel) {
  uint8_t packet[14];
  uint8_t i = 0;

  packet[i++] = 0xFF;
  packet[i++] = 0xFF;
  packet[i++] = id;
  packet[i++] = 0x0A;
  packet[i++] = FEETECH_INST_WRITE;
  packet[i++] = FEETECH_ACC;
  packet[i++] = accel;
  packet[i++] = position & 0xFF;
  packet[i++] = (position >> 8) & 0xFF;
  packet[i++] = 0x00;
  packet[i++] = 0x00;
  packet[i++] = speed & 0xFF;
  packet[i++] = (speed >> 8) & 0xFF;
  packet[i++] = feetech_checksum(packet, 2, 13);

  for (uint8_t j = 0; j < sizeof(packet); j++) {
    emit_byte(packet[j]);
  }
  fflush(stdout);
}

static void emit_wrist_servo_packets(float wrist_roll, float wrist_pitch) {
  uint16_t flex_pos =
      map_angle(wrist_pitch, PITCH_DEG_MIN, PITCH_DEG_MAX, WRIST_FLEX_MIN,
                WRIST_FLEX_MAX);
  uint16_t roll_pos =
      map_angle(wrist_roll, ROLL_DEG_MIN, ROLL_DEG_MAX, WRIST_ROLL_MIN,
                WRIST_ROLL_MAX);

  emit_write_pos_ex(SERVO_WRIST_FLEX, flex_pos, MOVE_SPEED, MOVE_ACCEL);
  emit_write_pos_ex(SERVO_WRIST_ROLL, roll_pos, MOVE_SPEED, MOVE_ACCEL);
}

static void emit_forearm_yaw_servo_packet(float forearm_yaw) {
  if (!forearm_yaw_zero_captured) {
    forearm_yaw_zero = forearm_yaw;
    forearm_yaw_zero_captured = true;
    return;
  }

  float d_forearm_yaw = forearm_yaw - forearm_yaw_zero;
  if (d_forearm_yaw > -ARM_DEADBAND_DEG &&
      d_forearm_yaw < ARM_DEADBAND_DEG) {
    d_forearm_yaw = 0.0f;
  }

  int target1 = clampi((int)(SHOULDER_PAN_HOME +
                             d_forearm_yaw * ARM_COUNTS_PER_DEG),
                       SHOULDER_PAN_MIN, SHOULDER_PAN_MAX);
  emit_write_pos_ex(SERVO_SHOULDER_PAN, (uint16_t)target1, MOVE_SPEED,
                    MOVE_ACCEL);
}

static void emit_gripper_servo_packet(int closed) {
  emit_write_pos_ex(SERVO_GRIPPER, closed ? GRIPPER_CLOSED : GRIPPER_OPEN,
                    MOVE_SPEED, MOVE_ACCEL);
}

static void emit_upper_pitch_servo_packets(float upper_pitch) {
  float upper_flex = upper_pitch_flex_fraction(upper_pitch);
  float delta = ARM_UP_SERVO2_TRAVEL_DEG * upper_flex * ARM_COUNTS_PER_DEG;
  int target2 = clampi((int)(SHOULDER_LIFT_HOME + delta), SHOULDER_LIFT_MIN,
                       SHOULDER_LIFT_MAX);
  int target3 = clampi((int)(ELBOW_FLEX_HOME - delta), ELBOW_FLEX_MIN,
                       ELBOW_FLEX_MAX);

  shoulder_lift_cur = step_toward(shoulder_lift_cur, target2);
  elbow_flex_cur = step_toward(elbow_flex_cur, target3);

  emit_write_pos_ex(SERVO_SHOULDER_LIFT, (uint16_t)shoulder_lift_cur,
                    MOVE_SPEED, MOVE_ACCEL);
  emit_write_pos_ex(SERVO_ELBOW_FLEX, (uint16_t)elbow_flex_cur, MOVE_SPEED,
                    MOVE_ACCEL);
}

static bool parse_radio_pose(const char *payload, radio_pose_t *pose) {
  *pose = (radio_pose_t){0};
  int parsed =
      sscanf(payload,
             "WR:%f,WP:%f,WY:%f,FR:%f,FP:%f,FY:%f,UR:%f,UP:%f,UY:%f,G:%d",
             &pose->wr, &pose->wp, &pose->wy, &pose->fr, &pose->fp, &pose->fy,
             &pose->ur, &pose->up, &pose->uy, &pose->gripper);
  pose->has_gripper = parsed == 10;
  return parsed >= 9;
}

static void emit_pose_packets(const radio_pose_t *pose) {
  emit_wrist_servo_packets(pose->wr, pose->wp);
  emit_forearm_yaw_servo_packet(pose->fy);
  emit_upper_pitch_servo_packets(pose->up);
  if (pose->has_gripper) {
    emit_gripper_servo_packet(pose->gripper != 0);
  }
}

static void process_payload(const char *payload) {
  radio_pose_t pose;
  if (parse_radio_pose(payload, &pose)) {
    emit_pose_packets(&pose);
  }
}

static bool take_latest_payload(char *payload) {
  bool has_payload = false;

  __disable_irq();
  if (latest.pending) {
    memcpy(payload, latest.payload, latest.len + 1);
    latest.pending = false;
    has_payload = true;
  }
  __enable_irq();

  return has_payload;
}

// callback fn for successful rx
void nrf_802154_received_raw(uint8_t *p_data, int8_t power, uint8_t lqi) {
  (void)power;
  (void)lqi;
  nrf_gpio_pin_toggle(/*20*/ LED_MIC);

  if (p_data[0] >= 23 + FCS_LENGTH) {
    uint8_t payload_len = p_data[0] - 23 - FCS_LENGTH;
    if (payload_len < LATEST_PAYLOAD_SIZE) {
      memcpy(latest.payload, &p_data[24], payload_len);
      latest.payload[payload_len] = '\0';
      latest.len = payload_len;
      latest.pending = true;
    }
  }

  nrf_802154_buffer_free_raw(p_data);
}

int main(void) {
  printf("RECEIVER STARTED\n");
  printf("Board started!\n");
  printf("UART WORKING\n");

  // Initialize.
  nrf_gpio_cfg_output(LED_MIC);

  // Configure 154 radio
  printf("About to init\n");
  nrf_802154_init();
  printf("Radio init complete\n");

  printf("Done with init\n");
  nrf_802154_channel_set(11);
  printf("Channel set\n");

  nrf_802154_auto_ack_set(false);
  nrf_802154_promiscuous_set(true);
  uint8_t src_pan_id[] = {0xcd, 0xab};
  nrf_802154_pan_id_set(src_pan_id);
  printf("PAN ID set\n");

  printf("Radio configured!\n");

  // Addresses (source and destination)
  uint8_t extended_addr[] = {0x50, 0xbe, 0xca, 0xc3, 0x3c, 0x36, 0xce, 0xf4};
  nrf_802154_extended_address_set(extended_addr);
  printf("Address set\n");
  if (nrf_802154_receive()) {
    printf("Entered receive mode\n");
  } else {
    printf("Could not enter receive mode\n");
  }

  while (true) {
    char payload[LATEST_PAYLOAD_SIZE];
    if (take_latest_payload(payload)) {
      process_payload(payload);
    }
    nrf_delay_ms(COMMAND_PERIOD_MS);
  }
}
