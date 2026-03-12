/*
  Teensy 4.1 (DEV-16771) Jetson <-> MCU controller for Stewart + Lift
  Protocol (little-endian):
    Header: 0xAA 0x55
    msg_type: u8
    seq: u16
    len: u16
    payload
    crc16: u16 (CCITT-FALSE)

  Incoming:
    0x01 CMD_BODY payload <ffffB>
      height_m, roll_rad, pitch_rad, yaw_rad, flags(estop bit0)
    0x02 CMD_ENABLE payload <B>
      enable (0/1)

  Outgoing:
    0x81 STATE_BODY payload <ffffH>
      height_m, roll_rad, pitch_rad, yaw_rad, fault_code

  Notes:
  - This sketch is a framework with RoboClaw-first placeholders.
  - Replace read*/write* functions with real RoboClaw API calls.
  - This file is pre-configured for Teensy 4.1 pin mapping.
  - Teensy 4.1 pins are 3.3V only (do NOT apply 5V logic to IO pins).
*/

#include <Arduino.h>

// ===== Teensy 4.1 mapping =====
// SerialUSB (Serial) is useful for debug logs to PC.
// Jetson command link can be Serial1 TTL or USB CDC; see JETSON_LINK_SERIAL.
static const bool JETSON_LINK_SERIAL1 = false;  // false: use USB Serial, true: use Serial1 (pins RX1/TX1)

// ===== RoboClaw transport & mapping =====
// Set to true to indicate low-level motor path should use RoboClaw APIs.
static const bool USE_ROBOCLAW = true;
static const uint32_t ROBOCLAW_BAUD = 115200;
// Example packet serial addresses (adjust to your actual RoboClaw address setup).
static const uint8_t RC_ADDR_LIFT = 0x80;
static const uint8_t RC_ADDR_LEG[6] = {0x81, 0x82, 0x83, 0x84, 0x85, 0x86};
// Motor channel mapping: 1->M1, 2->M2
static const uint8_t RC_CH_LIFT = 1;
static const uint8_t RC_CH_LEG[6] = {1, 1, 1, 1, 1, 1};

// Encoder/feedback placeholder pins (replace by your hardware interface):
static const uint8_t ESTOP_PIN = 22;  // active low recommended with pullup

// ===== Protocol =====
static const uint8_t HDR0 = 0xAA;
static const uint8_t HDR1 = 0x55;
static const uint8_t MSG_CMD_BODY = 0x01;
static const uint8_t MSG_CMD_ENABLE = 0x02;
static const uint8_t MSG_STATE_BODY = 0x81;

// ===== Control config =====
static const uint32_t CONTROL_HZ = 250;      // low-level loop
static const uint32_t TX_STATE_HZ = 25;      // state publish
static const uint32_t CMD_TIMEOUT_MS = 100;  // comm watchdog

static const float HEIGHT_MIN_M = 0.00f;
static const float HEIGHT_MAX_M = 0.30f;
static const float ROLL_MAX_RAD = 0.26f;   // +/-15 deg
static const float PITCH_MIN_RAD = -0.35f; // -20 deg
static const float PITCH_MAX_RAD = 0.44f;  // +25 deg
static const float YAW_MAX_RAD = 0.35f;    // +/-20 deg

// ===== Fault bits =====
static const uint16_t FAULT_NONE = 0;
static const uint16_t FAULT_ESTOP = 1 << 0;
static const uint16_t FAULT_TIMEOUT = 1 << 1;
static const uint16_t FAULT_LIMIT = 1 << 2;
static const uint16_t FAULT_DISABLED = 1 << 3;

struct CmdBody {
  float height_m;
  float roll_rad;
  float pitch_rad;
  float yaw_rad;
  uint8_t flags;
};

struct StateBody {
  float height_m;
  float roll_rad;
  float pitch_rad;
  float yaw_rad;
  uint16_t fault_code;
};

struct PID {
  float kp, ki, kd;
  float i_term;
  float last_err;
  float out_min, out_max;
};

static CmdBody g_cmd = {0.12f, 0, 0, 0, 0};
static StateBody g_state = {0.12f, 0, 0, 0, 0};
static bool g_enabled = false;
static bool g_estop = false;
static uint32_t g_last_cmd_ms = 0;

static float g_leg_target[6] = {0, 0, 0, 0, 0, 0};
static float g_leg_meas[6] = {0, 0, 0, 0, 0, 0};
static float g_lift_target = 0.12f;
static float g_lift_meas = 0.12f;

static PID g_lift_pid = {300.0f, 20.0f, 3.0f, 0, 0, -1.0f, 1.0f};
static PID g_leg_pid[6];

// ===== Utility =====
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

uint16_t crc16_ccitt_false(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

float pid_step(PID& pid, float err, float dt) {
  pid.i_term += err * dt;
  // anti-windup via output clamp
  float d = (err - pid.last_err) / max(dt, 1e-4f);
  float out = pid.kp * err + pid.ki * pid.i_term + pid.kd * d;
  out = clampf(out, pid.out_min, pid.out_max);
  pid.last_err = err;
  return out;
}

// ===== Hardware abstraction (replace with your drivers) =====
float readLiftPositionM() {
  // TODO (RoboClaw): read encoder/position from RC_ADDR_LIFT + RC_CH_LIFT
  // Convert encoder ticks to meters.
  return g_lift_meas;
}

void writeLiftCommand(float cmd_norm) {
  // TODO (RoboClaw): send signed speed/power command to lift motor.
  g_lift_meas += cmd_norm * 0.0008f;
  g_lift_meas = clampf(g_lift_meas, HEIGHT_MIN_M, HEIGHT_MAX_M);
}

float readLegLengthM(int i) {
  // TODO (RoboClaw): read encoder/position from RC_ADDR_LEG[i] + RC_CH_LEG[i]
  // Convert encoder ticks to actuator length in meters.
  return g_leg_meas[i];
}

void writeLegCommand(int i, float cmd_norm) {
  // TODO (RoboClaw): send signed speed/power command for leg i.
  g_leg_meas[i] += cmd_norm * 0.0005f;
}

bool readEstopPin() {
  // Active-low estop input.
  return digitalRead(ESTOP_PIN) == LOW;
}

// ===== Stewart mapping (placeholder) =====
void computeStewartTargets(float roll, float pitch, float yaw) {
  // TODO: replace with real Stewart IK from geometry.
  // Placeholder: symmetric linear map around nominal leg length.
  const float L0 = 0.180f;
  const float k = 0.010f;
  g_leg_target[0] = L0 + k * (+roll + pitch);
  g_leg_target[1] = L0 + k * (+roll - pitch);
  g_leg_target[2] = L0 + k * (-roll + pitch);
  g_leg_target[3] = L0 + k * (-roll - pitch);
  g_leg_target[4] = L0 + k * (+yaw);
  g_leg_target[5] = L0 + k * (-yaw);
}

void enterSafeHold() {
  g_state.fault_code |= FAULT_TIMEOUT;
  g_lift_target = 0.10f;  // safe height
  g_cmd.roll_rad = 0.0f;
  g_cmd.pitch_rad = 0.0f;
  g_cmd.yaw_rad = 0.0f;
}

void applyLimits() {
  bool limited = false;
  if (g_cmd.height_m < HEIGHT_MIN_M || g_cmd.height_m > HEIGHT_MAX_M) limited = true;
  if (g_cmd.roll_rad < -ROLL_MAX_RAD || g_cmd.roll_rad > ROLL_MAX_RAD) limited = true;
  if (g_cmd.pitch_rad < PITCH_MIN_RAD || g_cmd.pitch_rad > PITCH_MAX_RAD) limited = true;
  if (g_cmd.yaw_rad < -YAW_MAX_RAD || g_cmd.yaw_rad > YAW_MAX_RAD) limited = true;

  g_cmd.height_m = clampf(g_cmd.height_m, HEIGHT_MIN_M, HEIGHT_MAX_M);
  g_cmd.roll_rad = clampf(g_cmd.roll_rad, -ROLL_MAX_RAD, ROLL_MAX_RAD);
  g_cmd.pitch_rad = clampf(g_cmd.pitch_rad, PITCH_MIN_RAD, PITCH_MAX_RAD);
  g_cmd.yaw_rad = clampf(g_cmd.yaw_rad, -YAW_MAX_RAD, YAW_MAX_RAD);

  if (limited) g_state.fault_code |= FAULT_LIMIT;
}

// ===== Parser =====
bool readExact(uint8_t* dst, uint16_t n) {
  uint16_t got = 0;
  uint32_t t0 = millis();
  Stream& link = JETSON_LINK_SERIAL1 ? (Stream&)Serial1 : (Stream&)Serial;
  while (got < n) {
    while (link.available() && got < n) {
      dst[got++] = (uint8_t)link.read();
    }
    if (millis() - t0 > 5) return false;
  }
  return true;
}

void handleFrame(uint8_t msg_type, const uint8_t* payload, uint16_t len) {
  if (msg_type == MSG_CMD_BODY && len == sizeof(float) * 4 + 1) {
    memcpy(&g_cmd.height_m, payload + 0, 4);
    memcpy(&g_cmd.roll_rad, payload + 4, 4);
    memcpy(&g_cmd.pitch_rad, payload + 8, 4);
    memcpy(&g_cmd.yaw_rad, payload + 12, 4);
    g_cmd.flags = payload[16];
    g_estop = (g_cmd.flags & 0x01) != 0;
    g_last_cmd_ms = millis();
    g_state.fault_code &= ~FAULT_TIMEOUT;
  } else if (msg_type == MSG_CMD_ENABLE && len == 1) {
    g_enabled = payload[0] != 0;
    g_last_cmd_ms = millis();
  }
}

void processSerial() {
  Stream& link = JETSON_LINK_SERIAL1 ? (Stream&)Serial1 : (Stream&)Serial;
  while (link.available() >= 2) {
    int b0 = link.peek();
    if (b0 != HDR0) {
      link.read();
      continue;
    }
    // Need full header first
    if (link.available() < 7) return;
    uint8_t hdr[7];
    if (!readExact(hdr, 7)) return;  // HDR0 HDR1 msg seq_l seq_h len_l len_h
    if (hdr[0] != HDR0 || hdr[1] != HDR1) continue;

    uint8_t msg_type = hdr[2];
    uint16_t seq = (uint16_t)hdr[3] | ((uint16_t)hdr[4] << 8);
    (void)seq;
    uint16_t len = (uint16_t)hdr[5] | ((uint16_t)hdr[6] << 8);
    if (len > 64) continue;  // basic guard

    uint8_t payload[64];
    if (!readExact(payload, len)) return;
    uint8_t crc_raw[2];
    if (!readExact(crc_raw, 2)) return;
    uint16_t crc_recv = (uint16_t)crc_raw[0] | ((uint16_t)crc_raw[1] << 8);

    uint8_t check[7 + 64];
    memcpy(check, hdr, 7);
    memcpy(check + 7, payload, len);
    uint16_t crc_calc = crc16_ccitt_false(check, 7 + len);
    if (crc_recv != crc_calc) continue;

    handleFrame(msg_type, payload, len);
  }
}

void publishStateBody() {
  uint8_t payload[sizeof(float) * 4 + 2];
  memcpy(payload + 0, &g_state.height_m, 4);
  memcpy(payload + 4, &g_state.roll_rad, 4);
  memcpy(payload + 8, &g_state.pitch_rad, 4);
  memcpy(payload + 12, &g_state.yaw_rad, 4);
  payload[16] = (uint8_t)(g_state.fault_code & 0xFF);
  payload[17] = (uint8_t)((g_state.fault_code >> 8) & 0xFF);

  uint16_t len = sizeof(payload);
  static uint16_t tx_seq = 0;
  uint8_t head[7];
  head[0] = HDR0;
  head[1] = HDR1;
  head[2] = MSG_STATE_BODY;
  head[3] = (uint8_t)(tx_seq & 0xFF);
  head[4] = (uint8_t)((tx_seq >> 8) & 0xFF);
  head[5] = (uint8_t)(len & 0xFF);
  head[6] = (uint8_t)((len >> 8) & 0xFF);
  tx_seq++;

  uint8_t crc_buf[7 + sizeof(payload)];
  memcpy(crc_buf, head, 7);
  memcpy(crc_buf + 7, payload, len);
  uint16_t crc = crc16_ccitt_false(crc_buf, 7 + len);

  Stream& link = JETSON_LINK_SERIAL1 ? (Stream&)Serial1 : (Stream&)Serial;
  link.write(head, 7);
  link.write(payload, len);
  uint8_t crc_out[2] = {(uint8_t)(crc & 0xFF), (uint8_t)((crc >> 8) & 0xFF)};
  link.write(crc_out, 2);
}

void setup() {
  Serial.begin(115200);   // USB debug / optional command link
  Serial1.begin(115200);  // TTL UART command link (RX1/TX1)
  if (USE_ROBOCLAW) {
    Serial2.begin(ROBOCLAW_BAUD);  // RoboClaw packet serial bus
  }

  pinMode(ESTOP_PIN, INPUT_PULLUP);

  for (int i = 0; i < 6; i++) {
    g_leg_pid[i] = {250.0f, 10.0f, 2.0f, 0, 0, -1.0f, 1.0f};
    g_leg_meas[i] = 0.180f;
    g_leg_target[i] = 0.180f;
  }
  g_last_cmd_ms = millis();
  Serial.println("Teensy 4.1 Stewart+Lift controller booted (RoboClaw mode).");
}

void loop() {
  static uint32_t last_ctrl = 0;
  static uint32_t last_tx = 0;

  processSerial();

  const uint32_t now = millis();
  if (readEstopPin()) g_estop = true;

  // control loop
  if (now - last_ctrl >= (1000 / CONTROL_HZ)) {
    float dt = (now - last_ctrl) * 0.001f;
    last_ctrl = now;
    g_state.fault_code = FAULT_NONE;

    if (!g_enabled) {
      g_state.fault_code |= FAULT_DISABLED;
      return;
    }
    if (g_estop) {
      g_state.fault_code |= FAULT_ESTOP;
      writeLiftCommand(0);
      for (int i = 0; i < 6; i++) writeLegCommand(i, 0);
      return;
    }
    if (now - g_last_cmd_ms > CMD_TIMEOUT_MS) {
      enterSafeHold();
    }

    applyLimits();

    // set targets
    g_lift_target = g_cmd.height_m;
    computeStewartTargets(g_cmd.roll_rad, g_cmd.pitch_rad, g_cmd.yaw_rad);

    // feedback
    g_lift_meas = readLiftPositionM();
    for (int i = 0; i < 6; i++) g_leg_meas[i] = readLegLengthM(i);

    // control
    float u_lift = pid_step(g_lift_pid, (g_lift_target - g_lift_meas), dt);
    writeLiftCommand(u_lift);
    for (int i = 0; i < 6; i++) {
      float u = pid_step(g_leg_pid[i], (g_leg_target[i] - g_leg_meas[i]), dt);
      writeLegCommand(i, u);
    }

    // state mirror
    g_state.height_m = g_lift_meas;
    g_state.roll_rad = g_cmd.roll_rad;
    g_state.pitch_rad = g_cmd.pitch_rad;
    g_state.yaw_rad = g_cmd.yaw_rad;
  }

  if (now - last_tx >= (1000 / TX_STATE_HZ)) {
    last_tx = now;
    publishStateBody();
  }
}
