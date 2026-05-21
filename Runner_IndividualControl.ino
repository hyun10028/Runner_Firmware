/*
 * Runner — BLE Controller + 4× INA228 current sensors
 * Board  : Seeed XIAO nRF52840
 * Library: ArduinoBLE, Wire
 *
 * ── Motor pins ────────────────────────────────────────────────────────────────
 *   LF : IN1=1  IN2=0
 *   LR : IN1=3  IN2=2
 *   RF : IN1=9  IN2=10
 *   RR : IN1=7  IN2=8
 *
 * ── INA228 (Wire — external SDA/SCL, 400 kHz) ────────────────────────────────
 *   0x40 = LF  |  0x41 = LR  |  0x44 = RF  |  0x45 = RR
 *   Shunt: 0.1 Ω   CURRENT_LSB: 0.000305 A/LSB   SHUNT_CAL: 400
 *
 * ── BLE UUIDs ─────────────────────────────────────────────────────────────────
 *   Service : 12345678-1234-1234-1234-123456789abc
 *   CMD     : abcdef01  Write 8 B
 *               [0] command byte
 *               CMD_STOP         0x00
 *               CMD_MOTOR_SINGLE 0x30  [1]=motorIdx(0-3)  [2]=pwm(0-255)
 *               CMD_MOTOR_ALL    0x31  [1]=LF [2]=LR [3]=RF [4]=RR
 *   SENSOR  : abcdef02  Notify 64 B
 *               4 × 16 B, order LF LR RF RR
 *               each block: float32LE mA | vbusV | shuntmV | pwrMW
 *
 * ── Timing ────────────────────────────────────────────────────────────────────
 *   INA228 sampled round-robin every 2 ms (one sensor per tick)
 *   BLE notify + serial print every 10 ms
 *   Stats printed to serial every 1 s
 */

#include <ArduinoBLE.h>
#include <Wire.h>

// ── Timing ────────────────────────────────────────────────────────────────────
#define SAMPLE_INTERVAL_MS   2
#define NOTIFY_INTERVAL_MS   10
#define STATS_INTERVAL_MS    1000

// ── BLE ───────────────────────────────────────────────────────────────────────
#define SERVICE_UUID     "12345678-1234-1234-1234-123456789abc"
#define CMD_CHAR_UUID    "abcdef01-1234-1234-1234-123456789abc"
#define SENSOR_CHAR_UUID "abcdef02-1234-1234-1234-123456789abc"

BLEService        robotService(SERVICE_UUID);
BLECharacteristic commandChar (CMD_CHAR_UUID,    BLEWrite | BLEWriteWithoutResponse, 8);
BLECharacteristic sensorChar  (SENSOR_CHAR_UUID, BLENotify, 64);

// ── Command bytes ─────────────────────────────────────────────────────────────
#define CMD_STOP         0x00
#define CMD_MOTOR_SINGLE 0x30   // [1]=motorIdx 0-3, [2]=pwm 0-255
#define CMD_MOTOR_ALL    0x31   // [1]=LF, [2]=LR, [3]=RF, [4]=RR

// ── Motor pins ────────────────────────────────────────────────────────────────
//   Each motor is driven by two GPIO pins using software PWM (analogWrite).
//   IN1 = PWM forward, IN2 = LOW  → motor spins forward
//   IN1 = LOW,         IN2 = LOW  → motor coasts / stopped
const int MOTOR_IN1[4] = { 1,  3,  9, 7 };  // LF LR RF RR
const int MOTOR_IN2[4] = { 0,  2, 10, 8 };

bool pinIsPWM[40] = {};   // tracks which pins have an active analogWrite

// ── INA228 ────────────────────────────────────────────────────────────────────
#define NUM_INA      4
#define REG_CONFIG   0x00
#define REG_ADC_CFG  0x01
#define REG_SHNT_CAL 0x02
#define REG_VSHUNT   0x04
#define REG_VBUS     0x05

#define RSHUNT_OHMS  0.100f
#define CURRENT_LSB  0.000305f   // A/LSB → mA = raw * CURRENT_LSB * 1000

const uint8_t INA_ADDR[NUM_INA] = { 0x40, 0x41, 0x44, 0x45 };
const char*   INA_NAME[NUM_INA] = { "LF",  "LR",  "RF",  "RR" };

bool  inaOK  [NUM_INA] = {};
float g_mA   [NUM_INA] = {};
float g_vbusV[NUM_INA] = {};
float g_shMV [NUM_INA] = {};
float g_pwrMW[NUM_INA] = {};

uint8_t sampleIdx = 0;

// ── Rate counters ─────────────────────────────────────────────────────────────
unsigned long lastSample = 0, lastNotify = 0, lastStats = 0;
unsigned long sampleCount = 0, bleCount = 0;


// ═════════════════════════════════════════════════════════════════════════════
// Utility
// ═════════════════════════════════════════════════════════════════════════════

// Pack a float into a byte buffer at the given offset (little-endian)
void packFloat(uint8_t* buf, int offset, float val) {
  union { float f; uint8_t b[4]; } u;
  u.f = val;
  buf[offset + 0] = u.b[0];
  buf[offset + 1] = u.b[1];
  buf[offset + 2] = u.b[2];
  buf[offset + 3] = u.b[3];
}


// ═════════════════════════════════════════════════════════════════════════════
// INA228
// ═════════════════════════════════════════════════════════════════════════════

void ina228_write16(uint8_t addr, uint8_t reg, uint16_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write((val >> 8) & 0xFF);
  Wire.write(val & 0xFF);
  Wire.endTransmission();
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
    v |=          Wire.read();
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
    v |=           Wire.read();
  }
  return v;
}

void initINA228() {
  Wire.begin();
  Wire.setClock(400000);
  delay(50);

  // Scan bus and initialise each sensor
  for (uint8_t i = 0; i < NUM_INA; i++) {
    uint8_t addr = INA_ADDR[i];
    Wire.beginTransmission(addr);
    inaOK[i] = (Wire.endTransmission() == 0);

    if (!inaOK[i]) {
      Serial.print("# INA228 "); Serial.print(INA_NAME[i]);
      Serial.print(" not found @ 0x"); Serial.println(addr, HEX);
      continue;
    }

    ina228_write16(addr, REG_CONFIG,   0x8000); delay(10);  // soft reset
    ina228_write16(addr, REG_ADC_CFG,  0xA091);             // continuous, 16× avg
    uint16_t cal = (uint16_t)(13107200.0f * CURRENT_LSB * RSHUNT_OHMS); // 400
    ina228_write16(addr, REG_SHNT_CAL, cal);

    Serial.print("# INA228 "); Serial.print(INA_NAME[i]);
    Serial.print(" @ 0x"); Serial.print(addr, HEX);
    Serial.print("  CAL="); Serial.println(cal);
  }
}

void sampleINA228(uint8_t idx) {
  if (!inaOK[idx]) return;
  uint8_t addr = INA_ADDR[idx];

  int32_t  vshuntRaw = ina228_read24s(addr, REG_VSHUNT) >> 4;
  uint32_t vbusRaw   = ina228_read24u(addr, REG_VBUS)   >> 4;

  float shuntV    = vshuntRaw * 312.5e-9f;           // V
  g_mA   [idx]    = fabsf(shuntV / RSHUNT_OHMS * 1000.0f);
  g_vbusV[idx]    = (float)vbusRaw * 0.0001953125f;  // V
  g_shMV [idx]    = fabsf(shuntV * 1000.0f);         // mV
  g_pwrMW[idx]    = g_mA[idx] * g_vbusV[idx];        // mW
  sampleCount++;
}


// ═════════════════════════════════════════════════════════════════════════════
// Motor control
// ═════════════════════════════════════════════════════════════════════════════

void pinLow(int pin) {
  if (pinIsPWM[pin]) { analogWrite(pin, 0); pinIsPWM[pin] = false; }
  digitalWrite(pin, LOW);
}

void pinPWM(int pin, int duty) {
  analogWrite(pin, constrain(duty, 0, 255));
  pinIsPWM[pin] = true;
}

// Drive motor: forward at given PWM duty (0 = stop)
void setMotorPWM(uint8_t motorIdx, uint8_t pwm) {
  int in1 = MOTOR_IN1[motorIdx];
  int in2 = MOTOR_IN2[motorIdx];
  if (pwm == 0) {
    pinLow(in1);
    pinLow(in2);
  } else {
    pinLow(in2);
    pinPWM(in1, pwm);
  }
}

void setAllMotorPWM(uint8_t lf, uint8_t lr, uint8_t rf, uint8_t rr) {
  setMotorPWM(0, lf);
  setMotorPWM(1, lr);
  setMotorPWM(2, rf);
  setMotorPWM(3, rr);
}

void stopAllMotors() {
  setAllMotorPWM(0, 0, 0, 0);
}


// ═════════════════════════════════════════════════════════════════════════════
// BLE notify + serial
// ═════════════════════════════════════════════════════════════════════════════

void sendSensorNotify(bool connected) {
  // Serial: tab-separated current readings
  for (uint8_t i = 0; i < NUM_INA; i++) {
    Serial.print(g_mA[i], 2);
    Serial.print('\t');
  }
  Serial.println(g_vbusV[0], 3);

  if (!connected) return;

  // BLE: 4 × 16 B = 64 B  (mA, vbusV, shuntmV, pwrMW per motor)
  uint8_t buf[64];
  for (uint8_t i = 0; i < NUM_INA; i++) {
    int b = i * 16;
    packFloat(buf, b +  0, g_mA   [i]);
    packFloat(buf, b +  4, g_vbusV[i]);
    packFloat(buf, b +  8, g_shMV [i]);
    packFloat(buf, b + 12, g_pwrMW[i]);
  }
  sensorChar.writeValue(buf, 64);
  bleCount++;
}

void printStats(unsigned long elapsed_ms) {
  float sHz = elapsed_ms > 0 ? sampleCount * 1000.0f / elapsed_ms : 0;
  float bHz = elapsed_ms > 0 ? bleCount    * 1000.0f / elapsed_ms : 0;
  Serial.print("# inaHz="); Serial.print(sHz, 1);
  Serial.print("  bleHz="); Serial.println(bHz, 1);
  sampleCount = bleCount = 0;
}


// ═════════════════════════════════════════════════════════════════════════════
// Setup
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  // Motor pins
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(MOTOR_IN1[i], OUTPUT);
    pinMode(MOTOR_IN2[i], OUTPUT);
  }
  stopAllMotors();

  Serial.println("# Runner starting");

  initINA228();

  // BLE
  if (!BLE.begin()) {
    Serial.println("# BLE init failed — halting");
    while (1);
  }
  BLE.setLocalName("Runner");
  BLE.setDeviceName("Runner");
  BLE.setAdvertisedService(robotService);
  robotService.addCharacteristic(commandChar);
  robotService.addCharacteristic(sensorChar);
  BLE.addService(robotService);
  BLE.advertise();

  Serial.println("# BLE advertising as 'Runner'");
  Serial.println("LF_mA\tLR_mA\tRF_mA\tRR_mA\tVbus");

  lastSample = lastNotify = lastStats = millis();
}


// ═════════════════════════════════════════════════════════════════════════════
// Loop
// ═════════════════════════════════════════════════════════════════════════════

void loop() {
  BLE.poll();
  unsigned long now = millis();
  BLEDevice central  = BLE.central();
  bool      connected = central && central.connected();

  // ── Sample INA228 round-robin ──────────────────────────────────────────────
  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;
    // Advance to the next working sensor
    for (uint8_t tries = 0; tries < NUM_INA; tries++) {
      if (inaOK[sampleIdx]) { sampleINA228(sampleIdx); break; }
      sampleIdx = (sampleIdx + 1) % NUM_INA;
    }
    sampleIdx = (sampleIdx + 1) % NUM_INA;
  }

  // ── BLE notify + serial print ──────────────────────────────────────────────
  if (now - lastNotify >= NOTIFY_INTERVAL_MS) {
    lastNotify = now;
    sendSensorNotify(connected);
    BLE.poll();
  }

  // ── Stats ──────────────────────────────────────────────────────────────────
  if (now - lastStats >= STATS_INTERVAL_MS) {
    unsigned long elapsed = now - lastStats;
    lastStats = now;
    printStats(elapsed);
    BLE.poll();
  }

  // ── Safety: stop motors if central disconnects ─────────────────────────────
  if (!connected) {
    stopAllMotors();
    return;
  }

  // ── Command handler ────────────────────────────────────────────────────────
  if (!commandChar.written()) return;

  uint8_t d[8] = {};
  commandChar.readValue(d, 8);
  uint8_t cmd = d[0];

  switch (cmd) {

    case CMD_STOP:
      stopAllMotors();
      Serial.println("# CMD STOP");
      break;

    case CMD_MOTOR_SINGLE: {
      // Kept for manual debugging/backward compatibility.
      // The controller should normally use CMD_MOTOR_ALL so all motors update atomically.
      uint8_t motorIdx = d[1];
      uint8_t pwm      = d[2];
      if (motorIdx < 4) {
        setMotorPWM(motorIdx, pwm);
        Serial.print("# CMD SINGLE "); Serial.print(INA_NAME[motorIdx]);
        Serial.print(" PWM="); Serial.println(pwm);
      }
      break;
    }

    case CMD_MOTOR_ALL: {
      uint8_t lf = d[1];
      uint8_t lr = d[2];
      uint8_t rf = d[3];
      uint8_t rr = d[4];
      setAllMotorPWM(lf, lr, rf, rr);

      Serial.print("# CMD ALL LF="); Serial.print(lf);
      Serial.print(" LR="); Serial.print(lr);
      Serial.print(" RF="); Serial.print(rf);
      Serial.print(" RR="); Serial.println(rr);
      break;
    }

    default:
      stopAllMotors();
      Serial.print("# CMD unknown 0x"); Serial.println(cmd, HEX);
      break;
  }
}
