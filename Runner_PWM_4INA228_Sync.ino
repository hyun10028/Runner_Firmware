/*
 * Runner V2 — BLE Controller + 4x INA228 Current Sense
 * Board  : Seeed XIAO nRF52840
 * Library: ArduinoBLE
 * Driver : DRV8837 x4
 * Sensor : INA228 x4
 *
 * Pin mapping:
 *   Left  Front : IN1=1,  IN2=0
 *   Left  Rear  : IN1=3,  IN2=2
 *   Right Front : IN1=9,  IN2=10
 *   Right Rear  : IN1=7,  IN2=8
 *   INA228      : SDA=4,  SCL=5
 *
 * INA228 I2C addresses (A1, A0 pins):
 *   Index 0 — Left  Front : 0x40 (A1=GND, A0=GND)  binary 1000000
 *   Index 1 — Left  Rear  : 0x41 (A1=GND, A0=VS )  binary 1000001
 *   Index 2 — Right Front : 0x44 (A1=VS,  A0=GND)  binary 1000100
 *   Index 3 — Right Rear  : 0x45 (A1=VS,  A0=VS )  binary 1000101
 *
 * BLE characteristics:
 *   abcdef01 (Write)  — command [8 bytes]
 *     byte 0: cmd           (0x00=STOP, 0x01=FWD, 0x13=JUMP)
 *     bytes 1–3: reserved   (0x00)
 *     byte 4: reserved      (0x00, upper byte of left speed word)
 *     byte 5: left_speed    (0–255 PWM duty)
 *     byte 6: reserved      (0x00, upper byte of right speed word)
 *     byte 7: right_speed   (0–255 PWM duty)
 *
 *   abcdef02 (Notify) — sensor [64 bytes]
 *     Per sensor (×4), each block is 16 bytes:
 *       bytes  0- 3: float current_mA
 *       bytes  4- 7: float vbus_V
 *       bytes  8-11: float shunt_mV
 *       bytes 12-15: float power_mW
 *     Sensor order: [LF][LR][RF][RR]
 *
 * Serial output every STATS_INTERVAL_MS:
 *   # [stats] sampleRate=XX.X Hz  bleRate=XX.X Hz  inaReads=NNNN  bleSent=NNNN
 *   tab-separated header: LF_mA LR_mA RF_mA RR_mA Vbus_V State
 *
 * PWM control (DRV8837 IN-IN interface, Table 7-1):
 *   Forward : IN1=PWM,  IN2=LOW
 *   Reverse : IN1=LOW,  IN2=PWM
 *   Stop    : IN1=LOW,  IN2=LOW  (coast)
 */

#include <ArduinoBLE.h>
#include <Wire.h>

// ── Timing ────────────────────────────────────────────────────────────────────
#define SAMPLE_INTERVAL_MS   2
#define NOTIFY_INTERVAL_MS   10
#define STATS_INTERVAL_MS    1000

// ── INA228 ────────────────────────────────────────────────────────────────────
#define NUM_INA228           4
#define RSHUNT_OHMS          0.015f
#define CURRENT_LSB          0.000305f

#define REG_CONFIG           0x00
#define REG_ADC_CFG          0x01
#define REG_SHUNT_CAL        0x02
#define REG_VSHUNT           0x04
#define REG_VBUS             0x05

// Addresses: binary 0X40:1000000(A0:GND A1:GND), 0X41:1000001(A0:VS A1:GND), 0X44:1000100(A0:GND A1:VS), 0X45:1000101(A0:VS A1:VS)
const uint8_t INA228_ADDR[NUM_INA228] = { 0x40, 0x41, 0x44, 0x45 };
const char*   INA228_NAME[NUM_INA228] = { "RF", "LF", "RR", "LR" };

// ── BLE ───────────────────────────────────────────────────────────────────────
#define SERVICE_UUID         "12345678-1234-1234-1234-123456789abc"
#define CMD_CHAR_UUID        "abcdef01-1234-1234-1234-123456789abc"
#define SENSOR_CHAR_UUID     "abcdef02-1234-1234-1234-123456789abc"

// ── Motor pins ────────────────────────────────────────────────────────────────
const int LF_IN1=1, LF_IN2=0;
const int LR_IN1=3, LR_IN2=2;
const int RF_IN1=9, RF_IN2=10;
const int RR_IN1=7, RR_IN2=8;

#define CMD_STOP    0x00
#define CMD_FORWARD 0x01
#define CMD_JUMP    0x13

// ── State ─────────────────────────────────────────────────────────────────────
int  left_speed  = 200;
int  right_speed = 200;
int  currentDir  = 0;
bool inaOK[NUM_INA228] = {};

enum JumpPhase { JUMP_IDLE, JUMP_WINDUP, JUMP_KICK };
JumpPhase     jumpPhase      = JUMP_IDLE;
unsigned long jumpTimer      = 0;
const int     JUMP_WINDUP_MS = 300;
const int     JUMP_KICK_MS   = 150;

// ── Latest sensor readings — one set per INA228 ───────────────────────────────
volatile float g_mA   [NUM_INA228] = {};
volatile float g_vbusV[NUM_INA228] = {};
volatile float g_shMV [NUM_INA228] = {};
volatile float g_pwrMW[NUM_INA228] = {};

// Round-robin index: sample one sensor per SAMPLE_INTERVAL tick to spread
// I2C load evenly. All four are refreshed at SAMPLE_INTERVAL_MS * 4.
static uint8_t sampleIdx = 0;

// ── Rate counters ─────────────────────────────────────────────────────────────
unsigned long sampleCount  = 0;
unsigned long bleCount     = 0;
unsigned long lastSample   = 0;
unsigned long lastNotify   = 0;
unsigned long lastStats    = 0;

// ── BLE objects ───────────────────────────────────────────────────────────────
BLEService        robotService(SERVICE_UUID);
BLECharacteristic commandChar(CMD_CHAR_UUID,    BLEWrite | BLEWriteWithoutResponse, 8);
BLECharacteristic sensorChar (SENSOR_CHAR_UUID, BLENotify, 64);  // 4 × 16 bytes

// ═════════════════════════════════════════════════════════════════════════════
// Utility
// ═════════════════════════════════════════════════════════════════════════════

void packFloat(uint8_t* buf, int offset, float val) {
  union { float f; uint8_t b[4]; } u;
  u.f = val;
  buf[offset+0]=u.b[0]; buf[offset+1]=u.b[1];
  buf[offset+2]=u.b[2]; buf[offset+3]=u.b[3];
}

// ═════════════════════════════════════════════════════════════════════════════
// INA228 I2C helpers — address-parameterised
// ═════════════════════════════════════════════════════════════════════════════

void ina228_write16(uint8_t addr, uint8_t reg, uint16_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write((val >> 8) & 0xFF);
  Wire.write(val & 0xFF);
  Wire.endTransmission();
}

uint16_t ina228_read16(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)2);
  uint16_t v = 0;
  if (Wire.available() >= 2) { v = (uint16_t)Wire.read() << 8; v |= Wire.read(); }
  return v;
}

int32_t ina228_read24s(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)3);
  int32_t v = 0;
  if (Wire.available() >= 3) {
    v  = (int32_t)(int8_t)Wire.read() << 16;
    v |= (int32_t)Wire.read() << 8;
    v |= Wire.read();
  }
  return v;
}

uint32_t ina228_read24u(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)3);
  uint32_t v = 0;
  if (Wire.available() >= 3) {
    v  = (uint32_t)Wire.read() << 16;
    v |= (uint32_t)Wire.read() << 8;
    v |= Wire.read();
  }
  return v;
}

// ── Print config for one sensor ───────────────────────────────────────────────
void printINA228Config(uint8_t idx) {
  uint8_t addr = INA228_ADDR[idx];
  uint16_t adcConfig = ina228_read16(addr, REG_ADC_CFG);

  uint8_t avg    = (adcConfig >> 0) & 0x07;
  uint8_t vshct  = (adcConfig >> 3) & 0x07;
  uint8_t vbusct = (adcConfig >> 6) & 0x07;
  uint8_t vtct   = (adcConfig >> 9) & 0x07;
  uint8_t mode   = (adcConfig >> 12) & 0x0F;

  const uint32_t convTimes[] = {50, 84, 150, 280, 540, 1052, 2074, 4120};
  const uint16_t avgCounts[] = {1, 4, 16, 64, 128, 256, 512, 1024};

  uint32_t totalCycle    = convTimes[vbusct] + convTimes[vshct] + convTimes[vtct];
  uint32_t totalWithAvg  = totalCycle * avgCounts[avg];

  Serial.print("# ═══ INA228["); Serial.print(INA228_NAME[idx]);
  Serial.print("] 0x"); Serial.print(addr, HEX); Serial.println(" ═══");
  Serial.print("#   ADC_CFG raw     : 0x"); Serial.println(adcConfig, HEX);
  Serial.print("#   Mode            : 0x"); Serial.println(mode, HEX);
  Serial.print("#   AVG samples     : "); Serial.println(avgCounts[avg]);
  Serial.print("#   Total 1 cycle   : "); Serial.print(totalCycle);       Serial.println(" µs");
  Serial.print("#   Total w/ avg    : "); Serial.print(totalWithAvg);     Serial.println(" µs");
  Serial.print("#   Max sample rate : "); Serial.print(1000000.0f / totalWithAvg, 2); Serial.println(" Hz");
}

// ── Initialise one INA228 ─────────────────────────────────────────────────────
void initOneINA228(uint8_t idx) {
  uint8_t addr = INA228_ADDR[idx];

  for (int attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(addr);
    inaOK[idx] = (Wire.endTransmission() == 0);
    if (inaOK[idx]) break;
    delay(20);
  }

  if (!inaOK[idx]) {
    Serial.print("# INA228["); Serial.print(INA228_NAME[idx]);
    Serial.print("] NOT FOUND @ 0x"); Serial.println(addr, HEX);
    return;
  }

  ina228_write16(addr, REG_CONFIG, 0x8000);  // soft reset
  delay(10);

  // continuous shunt only, 50µs bus, 150µs shunt, 50µs temp, 4-sample avg
  // → 150µs × 4 = 600µs per conversion (~1.6 kHz max hardware rate)
  ina228_write16(addr, REG_ADC_CFG, 0xA091);

  uint16_t cal = (uint16_t)(13107200.0f * CURRENT_LSB * RSHUNT_OHMS);
  ina228_write16(addr, REG_SHUNT_CAL, cal);

  uint16_t calCheck = ina228_read16(addr, REG_SHUNT_CAL);
  Serial.print("# INA228["); Serial.print(INA228_NAME[idx]);
  Serial.print("] ready @ 0x"); Serial.print(addr, HEX);
  Serial.print("  SHUNT_CAL="); Serial.print(cal);
  Serial.print(" readback="); Serial.println(calCheck);
  if (calCheck != cal) {
    Serial.print("# WARNING: SHUNT_CAL mismatch on INA228[");
    Serial.print(INA228_NAME[idx]); Serial.println("]");
  }

  printINA228Config(idx);
}

void initINA228() {
  Wire.begin();
  Wire.setClock(400000);  // 400 kHz — faster bus to keep 4-sensor round-robin tight
  delay(50);
  for (uint8_t i = 0; i < NUM_INA228; i++) initOneINA228(i);
}

// ── Sample one INA228 by index ────────────────────────────────────────────────
void sampleINA228(uint8_t idx) {
  uint8_t addr = INA228_ADDR[idx];
  int32_t  vshuntRaw = ina228_read24s(addr, REG_VSHUNT) >> 4;
  uint32_t vbusRaw   = ina228_read24u(addr, REG_VBUS)   >> 4;
  float shuntV   = vshuntRaw * 312.5e-9f;
  g_mA   [idx]   = fabsf(shuntV / RSHUNT_OHMS * 1000.0f);
  g_vbusV[idx]   = (float)vbusRaw * 0.0001953125f;
  g_shMV [idx]   = fabsf(shuntV * 1000.0f);
  g_pwrMW[idx]   = g_mA[idx] * g_vbusV[idx];
  sampleCount++;
}

// ── Send all sensor data over BLE + Serial ────────────────────────────────────
void sendNotify(bool connected) {
  // Serial: LF_mA, LR_mA, RF_mA, RR_mA, Vbus_V (avg), State
  for (uint8_t i = 0; i < NUM_INA228; i++) {
    Serial.print(g_mA[i], 2);
    Serial.print('\t');
  }
  // Use LF vbus as the representative bus voltage (all share same rail)
  Serial.print(g_vbusV[0], 4); Serial.print('\t');
  Serial.println(
    jumpPhase == JUMP_WINDUP ? "WINDUP" :
    jumpPhase == JUMP_KICK   ? "KICK"   :
    currentDir == 1          ? "FORWARD": "STOP"
  );

  if (!connected) return;

  // BLE: 4 × 16 bytes = 64 bytes total, order [LF][LR][RF][RR]
  uint8_t buf[64];
  for (uint8_t i = 0; i < NUM_INA228; i++) {
    int base = i * 16;
    packFloat(buf, base +  0, g_mA   [i]);
    packFloat(buf, base +  4, g_vbusV[i]);
    packFloat(buf, base +  8, g_shMV [i]);
    packFloat(buf, base + 12, g_pwrMW[i]);
  }
  sensorChar.writeValue(buf, 64);
  bleCount++;
}

// ── Print rate stats to Serial ────────────────────────────────────────────────
void printStats(unsigned long elapsed_ms) {
  float sampleHz = (elapsed_ms > 0) ? (sampleCount * 1000.0f / elapsed_ms) : 0;
  float bleHz    = (elapsed_ms > 0) ? (bleCount    * 1000.0f / elapsed_ms) : 0;
  Serial.print("# [stats] sampleRate="); Serial.print(sampleHz, 1);
  Serial.print(" Hz  bleRate=");         Serial.print(bleHz, 1);
  Serial.print(" Hz  inaReads=");        Serial.print(sampleCount);
  Serial.print("  bleSent=");            Serial.println(bleCount);
  sampleCount = 0;
  bleCount    = 0;
}

// ═════════════════════════════════════════════════════════════════════════════
// Motor helpers  (unchanged from V1)
// ═════════════════════════════════════════════════════════════════════════════

static bool pinIsPWM[40] = {};

void pinLow(int pin) {
  if (pinIsPWM[pin]) { analogWrite(pin, 0); pinIsPWM[pin] = false; }
  digitalWrite(pin, LOW);
}

void pinPWM(int pin, int duty) {
  analogWrite(pin, duty);
  pinIsPWM[pin] = true;
}

void setMotor(int in1, int in2, int dir, int duty) {
  if      (dir ==  1) { pinLow(in2); pinPWM(in1, duty); }
  else if (dir == -1) { pinLow(in1); pinPWM(in2, duty); }
  else                { pinLow(in1); pinLow(in2); }
}

void setLeft (int d) { setMotor(LF_IN1,LF_IN2,d,left_speed);  setMotor(LR_IN1,LR_IN2,d,left_speed);  }
void setRight(int d) { setMotor(RF_IN1,RF_IN2,d,right_speed); setMotor(RR_IN1,RR_IN2,d,right_speed); }
void stopMotors()    { setLeft(0); setRight(0); }

// ═════════════════════════════════════════════════════════════════════════════
// Setup
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  int pins[] = { LF_IN1,LF_IN2,LR_IN1,LR_IN2,RR_IN1,RR_IN2,RF_IN1,RF_IN2 };
  for (int i = 0; i < 8; i++) pinMode(pins[i], OUTPUT);
  stopMotors();

  initINA228();

  if (!BLE.begin()) { Serial.println("# BLE init failed"); while(1); }
  BLE.setLocalName("Runner");
  BLE.setAdvertisedService(robotService);
  robotService.addCharacteristic(commandChar);
  robotService.addCharacteristic(sensorChar);
  BLE.addService(robotService);
  BLE.advertise();

  // Serial header — one column per sensor + vbus + state
  Serial.print("LF_mA\tLR_mA\tRF_mA\tRR_mA\tVbus_V\tState");
  Serial.println();
  Serial.println("# Runner V2 (4×INA228) ready. Waiting for BLE...");
  Serial.print("# Round-robin sample period per sensor: ");
  Serial.print(SAMPLE_INTERVAL_MS * NUM_INA228);
  Serial.print(" ms  notify=");
  Serial.print(NOTIFY_INTERVAL_MS);
  Serial.println(" ms");

  lastSample = lastNotify = lastStats = millis();
}

// ═════════════════════════════════════════════════════════════════════════════
// Loop — non-blocking, three independent timers
// ═════════════════════════════════════════════════════════════════════════════

void loop() {
  BLE.poll();
  unsigned long now = millis();

  // ── BLE connection state ──────────────────────────────────────────────────
  BLEDevice central  = BLE.central();
  bool      connected = central && central.connected();

  // ── Timer 1: Sample one INA228 per tick (round-robin) ────────────────────
  // Cycling through all 4 sensors spreads I2C load across ticks and avoids
  // stalling BLE for the duration of 4 consecutive reads.
  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;
    // Find the next available (detected) sensor
    for (uint8_t tries = 0; tries < NUM_INA228; tries++) {
      if (inaOK[sampleIdx]) {
        sampleINA228(sampleIdx);
        BLE.poll();  // service BLE after I2C blocking read
        break;
      }
      sampleIdx = (sampleIdx + 1) % NUM_INA228;
    }
    sampleIdx = (sampleIdx + 1) % NUM_INA228;
  }

  // ── Timer 2: BLE notify + Serial data line ────────────────────────────────
  if (now - lastNotify >= NOTIFY_INTERVAL_MS) {
    lastNotify = now;
    sendNotify(connected);
    BLE.poll();
  }

  // ── Timer 3: Stats printout ───────────────────────────────────────────────
  if (now - lastStats >= STATS_INTERVAL_MS) {
    unsigned long elapsed = now - lastStats;
    lastStats = now;
    printStats(elapsed);
    BLE.poll();
  }

  // ── Jump state machine ────────────────────────────────────────────────────
  if (jumpPhase == JUMP_WINDUP && now - jumpTimer >= JUMP_WINDUP_MS) {
    stopMotors();
    jumpTimer = now; jumpPhase = JUMP_KICK;
    Serial.println("# JUMP: kick");
  } else if (jumpPhase == JUMP_KICK && now - jumpTimer >= JUMP_KICK_MS) {
    jumpPhase = JUMP_IDLE; currentDir = 0;
    Serial.println("# JUMP: done");
  }

  // ── Safety: stop motors if BLE drops ─────────────────────────────────────
  if (!connected) {
    if (currentDir != 0 || jumpPhase != JUMP_IDLE) {
      stopMotors();
      currentDir = 0;
      jumpPhase  = JUMP_IDLE;
    }
    return;
  }

  // ── BLE command receive ───────────────────────────────────────────────────
  if (!commandChar.written()) return;

  uint8_t d[8] = {};
  commandChar.readValue(d, 8);
  uint8_t  cmd = d[0];
  uint16_t ls  = (d[4] << 8) | d[5];
  uint16_t rs  = (d[6] << 8) | d[7];

  if (ls > 0) left_speed  = constrain((int)ls, 0, 255);
  if (rs > 0) right_speed = constrain((int)rs, 0, 255);

  Serial.print("# CMD:"); Serial.print(cmd);
  Serial.print(" L_SPD:"); Serial.print(left_speed);
  Serial.print(" R_SPD:"); Serial.println(right_speed);

  switch (cmd) {
    case CMD_FORWARD:
      currentDir = 1;
      if (jumpPhase == JUMP_IDLE) { setLeft(1); setRight(1); }
      break;
    case CMD_STOP:
      currentDir = 0; jumpPhase = JUMP_IDLE; stopMotors();
      break;
    case CMD_JUMP:
      if (jumpPhase == JUMP_IDLE) {
        currentDir = 0; setLeft(1); setRight(1);
        jumpTimer = now; jumpPhase = JUMP_WINDUP;
        Serial.println("# JUMP: windup");
      }
      break;
    default:
      currentDir = 0; stopMotors();
      break;
  }
}
