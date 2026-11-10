#include <Arduino.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <RH_ASK.h>
#include <esp8266_peri.h>
#include <math.h>
namespace Firmware {


/* ESP8266 NodeMCU: register-level BMP5xx + MPU6050/MPU6500 acquisition.
 * No Wire or sensor libraries. GPIO-register software I2C, external pull-ups.
 * Arduino ESP8266 runtime remains responsible for boot, time and Serial.
 * Single master only. GPIO4=D2 SDA; GPIO5=D1 SCL. 3.3V logic only.
 */


// RH_ASK(speed, RX pin, TX pin, PTT pin). All pin numbers here are GPIO.
// TX DATA -> D5/GPIO14. D6/GPIO12 shares the unused RX input with the button.
// This sketch is transmit-only. D7/GPIO13 remains unconnected.
RH_ASK radio(4000, 12, 14, 13, false);
bool radioOK = false;
// USB CSV at 50 Hz. RF sends these same snapshots only when its prior packet
// has finished; 4000 bps cannot carry all 50 CSV rows per second.
constexpr uint32_t OUTPUT_INTERVAL_MS = 20;

#if !defined(ESP8266)
#error Select an ESP8266 board, such as NodeMCU 1.0 (ESP-12E Module)
#endif

constexpr uint32_t SDA_MASK = 1UL << 4, SCL_MASK = 1UL << 5;
constexpr unsigned HALF_US = 2; // Shorter bit delays to service 100 Hz IMU; software timing is approximate
uint8_t bmpAddr = 0, mpuAddr = 0;
bool bmpOK = false, mpuOK = false;
uint8_t mpuID = 0;
const char *busError = "OK";

void low(uint32_t mask) { GPOC = mask; GPES = mask; }
void releaseLine(uint32_t mask) { GPEC = mask; }
bool high(uint32_t mask) { return (GPI & mask) != 0; }
void tick() { delayMicroseconds(HALF_US); }

bool clockHigh() {
  releaseLine(SCL_MASK);
  uint32_t t = micros();
  while (!high(SCL_MASK)) {
    if (uint32_t(micros() - t) > 2000) {
      busError = "SCL held LOW / clock timeout";
      return false;
    }
  }
  tick();
  return true;
}

bool stopBus() {
  low(SCL_MASK); low(SDA_MASK); tick();
  bool ok = clockHigh();
  releaseLine(SDA_MASK); tick();
  return ok && high(SDA_MASK);
}

bool startBus() {
  releaseLine(SDA_MASK); tick();
  if (!clockHigh()) return false;
  if (!high(SDA_MASK)) { busError = "SDA held LOW"; return false; }
  low(SDA_MASK); tick(); low(SCL_MASK); tick();
  return true;
}

bool putByte(uint8_t value) {
  for (uint8_t mask = 0x80; mask; mask >>= 1) {
    if (value & mask) releaseLine(SDA_MASK); else low(SDA_MASK);
    tick();
    if (!clockHigh()) return false;
    low(SCL_MASK); tick();
  }
  releaseLine(SDA_MASK); tick();
  if (!clockHigh()) return false;
  bool ack = !high(SDA_MASK);
  low(SCL_MASK); tick();
  if (!ack) busError = "NACK";
  return ack;
}

bool getByte(uint8_t &value, bool ack) {
  value = 0; releaseLine(SDA_MASK);
  for (uint8_t i = 0; i < 8; ++i) {
    tick(); if (!clockHigh()) return false;
    value = (value << 1) | (high(SDA_MASK) ? 1 : 0);
    low(SCL_MASK); tick();
  }
  if (ack) low(SDA_MASK); else releaseLine(SDA_MASK);
  tick(); if (!clockHigh()) return false;
  low(SCL_MASK); tick(); releaseLine(SDA_MASK);
  return true;
}

bool probe(uint8_t addr) {
  busError = "OK";
  bool ok = startBus() && putByte(addr << 1);
  bool stopped = stopBus();
  return ok && stopped;
}

bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  busError = "OK";
  bool ok = startBus() && putByte(addr << 1) && putByte(reg) && putByte(value);
  bool stopped = stopBus();
  if (!ok || !stopped)
    Serial.printf("WRITE addr=0x%02X reg=0x%02X failed: %s\n", addr, reg, busError);
  return ok && stopped;
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t *data, size_t count) {
  busError = "OK";
  bool ok = startBus() && putByte(addr << 1) && putByte(reg)
         && startBus() && putByte((addr << 1) | 1);
  for (size_t i = 0; ok && i < count; ++i)
    ok = getByte(data[i], i + 1 < count);
  bool stopped = stopBus();
  if (!ok || !stopped)
    Serial.printf("READ addr=0x%02X reg=0x%02X failed: %s\n", addr, reg, busError);
  return ok && stopped;
}

bool recoverBus() {
  releaseLine(SDA_MASK | SCL_MASK); tick();
  for (uint8_t i = 0; i < 9 && !high(SDA_MASK); ++i) {
    low(SCL_MASK); tick(); if (!clockHigh()) return false;
  }
  return stopBus();
}

bool initBMP() {
  uint8_t id, rev, odr, status = 0;
  if (!readRegs(bmpAddr, 0x01, &id, 1) || !readRegs(bmpAddr, 0x02, &rev, 1)) return false;
  Serial.printf("BMP family: CHIP_ID=0x%02X REV_ID=0x%02X\n", id, rev);
  // Bosch BMP5 API accepts both family IDs. This does not uniquely identify the model.
  if (id != 0x50 && id != 0x51) { Serial.println("Unexpected BMP chip ID"); return false; }
  if (!readRegs(bmpAddr, 0x37, &odr, 1)) return false;
  if (!writeReg(bmpAddr, 0x37, (odr & 0x7C) | 0x80)) return false;
  delay(5); // Explicit standby, deep standby disabled; handles MCU-only resets.
  for (uint8_t i = 0; i < 25; ++i) {
    if (!readRegs(bmpAddr, 0x28, &status, 1)) return false;
    if (status & 0x0C) { Serial.printf("BMP NVM error: 0x%02X\n", status); return false; }
    if (status & 0x02) break;
    delay(1);
  }
  if (!(status & 0x02)) { Serial.println("BMP NVM not ready"); return false; }
  // STM settings: pressure 64x, temperature 4x, 20 Hz; IIR P=7, T=3.
  if (!writeReg(bmpAddr, 0x30, 0xAB)) return false;
  if (!writeReg(bmpAddr, 0x18, 0x00) || !writeReg(bmpAddr, 0x31, 0x1A)
      || !writeReg(bmpAddr, 0x15, 0x01) || !writeReg(bmpAddr, 0x36, 0x72)
      || !writeReg(bmpAddr, 0x37, 0xD5)) return false;
  delay(60);
  if (!readRegs(bmpAddr, 0x38, &status, 1)) return false;
  if (!(status & 0x80)) { Serial.println("BMP invalid ODR/OSR configuration"); return false; }
  return true;
}

bool initMPU() {
  uint8_t id;
  if (!readRegs(mpuAddr, 0x75, &id, 1)) return false;
  Serial.printf("MPU WHO_AM_I=0x%02X\n", id);
  if (id != 0x68 && id != 0x70) {
    Serial.println("Expected MPU6050 (0x68) or MPU6500 (0x70).");
    return false;
  }
  mpuID = id;
  Serial.println(id == 0x70 ? "Detected MPU6500" : "Detected MPU6050");
  if (!writeReg(mpuAddr, 0x6B, 0x80)) return false;
  delay(100);
  if (!writeReg(mpuAddr, 0x6B, 0x01)) return false;
  delay(20);
  // MPU6500 accelerometer DLPF: enabled, approximately 21 Hz.
  if (mpuID == 0x70 && !writeReg(mpuAddr, 0x1D, 0x04)) return false;
  return writeReg(mpuAddr, 0x38, 0x01) && writeReg(mpuAddr, 0x37, 0x20)
      && writeReg(mpuAddr, 0x6C, 0x00) && writeReg(mpuAddr, 0x1A, 0x04)
      && writeReg(mpuAddr, 0x19, 9)   // 100 Hz with DLPF enabled
      && writeReg(mpuAddr, 0x1B, 0x08)   // +/-500 degrees/s
      && writeReg(mpuAddr, 0x1C, 0x08);  // +/-4 g
}

int16_t signedBE(const uint8_t *p) { return int16_t((uint16_t(p[0]) << 8) | p[1]); }

void initHardware() {
  Serial.begin(115200); delay(500);
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  radioOK = radio.init();
  if (!radioOK) Serial.println("RF initialization FAILED");
  else { radio.setModeIdle(); Serial.println("RF ready: 4000 bps, TX D5/GPIO14"); }
  Serial.println("\nNodeMCU direct-register I2C diagnostic");
  Serial.println(ESP.getResetReason());
  // Select GPIO function without enabling internal pull-ups.
  pinMode(4, INPUT); pinMode(5, INPUT);
  GPOC = SDA_MASK | SCL_MASK;
  releaseLine(SDA_MASK | SCL_MASK); delay(20);
  Serial.printf("Idle digital levels: SDA=%d SCL=%d (not a voltage measurement)\n",
                high(SDA_MASK), high(SCL_MASK));
  if (!recoverBus()) {
    Serial.println("Bus recovery failed. Check line levels and external pull-ups.");
    return;
  }
  unsigned count = 0;
  for (uint8_t addr = 8; addr < 0x78; ++addr) {
    if (probe(addr)) {
      Serial.printf("ACK at 0x%02X\n", addr); ++count;
      if (addr == 0x46 || addr == 0x47) bmpAddr = addr;
      if (addr == 0x68 || addr == 0x69) mpuAddr = addr;
    }
    yield();
  }
  Serial.printf("Scan found %u device(s)\n", count);
  if (bmpAddr) bmpOK = initBMP();
  if (mpuAddr) mpuOK = initMPU();
  Serial.printf("BMP ready=%d; MPU ready=%d\n", bmpOK, mpuOK);
  if (!bmpAddr) Serial.println("BMP: no address ACK at 0x46/0x47; initialization not attempted.");
  Serial.println("Press reset after changing connections.");
}




struct MPU9250_RawData_t { float accel_g[3], gyro_dps[3]; };
struct BMP585_Data_t { float pressure_pa, pressure_hpa, temperature_c; };
struct BMP585_IntStatus_t { bool drdy_data_reg; };
using MPU9250_Handle_t = int;
using BMP585_Handle_t = int;
using MPU9250_Status_t = int;
constexpr int MPU9250_OK=0, BMP585_OK=0;
#define UART3_Printf Serial.printf
#define MCU_DelayMs delay
#define MCU_LED_Set(...) ((void)0)
void UART3_PrintMPUError(const char *message, int status) { Serial.printf("%s: %d\n",message,status); }

bool imuReady() {
  uint8_t s;
  return readRegs(mpuAddr, 0x3A, &s, 1) && (s & 1);
}
bool readIMU(MPU9250_RawData_t &s) {
  uint8_t b[14];
  if (!readRegs(mpuAddr,0x3B,b,14)) return false;
  for (uint8_t i=0;i<3;++i) {
    s.accel_g[i]=signedBE(b+2*i)/8192.0f;
    s.gyro_dps[i]=signedBE(b+8+2*i)/65.5f;
  }
  return true;
}
// Startup calibration uses distinct hardware samples, not duplicate 5ms reads.
int MPU9250_ReadAll9Axis(MPU9250_Handle_t *, MPU9250_RawData_t *s) {
  uint32_t start=millis();
  while (!imuReady()) { if (millis()-start>40) return -1; delay(1); }
  return readIMU(*s) ? 0 : -1;
}
int BMP585_GetIntStatus(BMP585_Handle_t *, BMP585_IntStatus_t *s) {
  uint8_t v;
  if (!readRegs(bmpAddr,0x27,&v,1)) return -1;
  s->drdy_data_reg = (v&1)!=0; return 0;
}
int BMP585_ReadSensorData(BMP585_Handle_t *, BMP585_Data_t *s) {
  uint8_t b[6];
  if (!readRegs(bmpAddr,0x1D,b,6)) return -1;
  uint32_t u=uint32_t(b[0])|(uint32_t(b[1])<<8)|(uint32_t(b[2])<<16);
  int32_t t=int32_t(u); if(u&0x800000UL) t-=0x1000000L;
  uint32_t p=uint32_t(b[3])|(uint32_t(b[4])<<8)|(uint32_t(b[5])<<16);
  s->temperature_c=t/65536.0f; s->pressure_pa=p/64.0f;
  s->pressure_hpa=s->pressure_pa/100.0f; return 0;
}

// Ported from the supplied STM main.c; see README for timing and validity changes.
#define CAL_WARMUP_DISCARD_SAMPLES      (40U)       /* Discard first 40 IMU samples (200 ms) of warmup */
#define CAL_GYRO_ACCEL_SAMPLES          (400U)      /* 400 stationary IMU samples (~2.0 s) */
#define CAL_GYRO_MAX_FACTORY_ZRO_DPS    (25.0f)     /* Max factory uncalibrated Zero-Rate Offset (dps) */
#define CAL_GYRO_DELTA_STILL_DPS        (2.0f)      /* Max sample-to-sample change for stillness at boot */
#define CAL_ZUPT_DELTA_THRESHOLD_DPS    (0.60f)     /* Online ZUPT sample-to-sample gyro delta threshold */
#define CAL_ZUPT_MIN_HOLD_SAMPLES       (20U)       /* 20 consecutive still samples (200 ms) for ZUPT lock */
#define CAL_ZUPT_BIAS_ALPHA             (0.05f)     /* Online ZUPT bias adaptation rate */
#define CAL_GYRO_DEADBAND_DPS           (0.15f)     /* Stationary zero-rate deadband (deg/s) */

/* --- BMP585 Barometric Calibration & Stationary Stabilization Parameters --- */
#define CAL_BMP_WARMUP_DISCARD_SAMPLES  (12U)       /* Discard first 12 BMP samples (600 ms @ 20 Hz) for IIR/thermal settling */
#define CAL_BMP_BASELINE_SAMPLES        (30U)       /* Average 30 settled BMP samples (1.5 s @ 20 Hz) for exact P0 */
#define CAL_BMP_STD_SEA_LEVEL_PA        (101325.0f) /* Standard sea-level atmospheric pressure (Pa) */
#define CAL_BMP_PRESS_EMA_ALPHA_MOVING  (0.25f)     /* Pressure LPF alpha during vertical motion */
#define CAL_BMP_PRESS_EMA_ALPHA_STILL   (0.06f)     /* Heavy pressure LPF alpha when IMU detects stillness */
#define CAL_BMP_TEMP_EMA_ALPHA          (0.10f)     /* Temperature LPF alpha */
#define CAL_BMP_STILL_REF_TRACK_ALPHA   (0.03f)     /* Slow P0 reference tracking rate when stationary to cancel weather drift */
#define CAL_BMP_STILL_MAX_DRIFT_PA      (1.80f)     /* Max pressure deviation (Pa) considered stationary micro-turbulence (~15 cm) */
#define CAL_BMP_PRESS_DEADBAND_PA       (0.45f)     /* Stationary pressure deadband (Pa) around P0 to hold reading constant */
#define CAL_BMP_TEMP_DEADBAND_C         (0.03f)     /* Stationary temperature deadband (deg C) to prevent 0.01 C flicker */
#define CAL_BMP_ALT_DEADBAND_M          (0.05f)     /* Stationary relative altitude deadband (m) */
#define CAL_BMP_VZ_DEADBAND_MPS         (0.04f)     /* Vertical velocity deadband (m/s) */

/**
 * @brief Complete Combined MPU + BMP585 Calibration State
 *        (Defined and used exclusively inside Main.c)
 */
typedef struct {
    /* 1. MPU Gyroscope Zero-Rate Bias (deg/s) & Online ZUPT Stationary Tracker */
    float    gyro_bias_dps[3];
    float    prev_raw_gyro_dps[3];
    uint32_t stationary_count;
    float    latest_accel_norm_g;   /* Latest calibrated total acceleration magnitude |a| (g) */

    /* 2. MPU Accelerometer Zero-G Offset (g) & Scale Factor */
    float    accel_offset_g[3];
    float    accel_scale[3];
    bool     imu_calibrated;

    /* 3. BMP585 Reference Pressure (P0), Stationary Lock & Vertical Motion State */
    float    bmp_ref_pressure_pa;   /* Calibrated ground-level reference pressure P0 (Pa) */
    float    bmp_locked_press_pa;   /* Displayed/locked pressure after deadband (Pa) */
    float    bmp_filtered_press_pa; /* Low-pass filtered pressure state (Pa) */
    float    bmp_filtered_temp_c;   /* Low-pass filtered temperature state (deg C) */
    float    bmp_locked_temp_c;     /* Displayed/locked temperature after deadband (deg C) */
    float    bmp_abs_altitude_m;    /* Absolute barometric altitude above sea level (m) */
    float    bmp_rel_altitude_m;    /* Calibrated relative altitude change dAlt from P0 (m) */
    float    bmp_prev_rel_alt_m;    /* Previous relative altitude for vertical velocity (m) */
    float    bmp_vert_vel_mps;      /* Smoothed vertical climb/descent velocity Vz (m/s) */
    bool     bmp_calibrated;
} Main_Calibration_t;

/* ========================================================================== */
/* 2. CALIBRATION ROUTINES (IMPLEMENTED STRICTLY IN MAIN.C)                   */
/* ========================================================================== */

/**
 * @brief Initializes combined MPU + BMP585 calibration structure to neutral defaults.
 */
static void Main_InitCalibrationDefaults(Main_Calibration_t *cal)
{
    for (uint8_t i = 0U; i < 3U; i++) {
        cal->gyro_bias_dps[i]     = 0.0f;
        cal->prev_raw_gyro_dps[i] = 0.0f;
        cal->accel_offset_g[i]    = 0.0f;
        cal->accel_scale[i]       = 1.0f;
    }
    cal->stationary_count      = 0U;
    cal->latest_accel_norm_g   = 1.0f;
    cal->imu_calibrated        = false;

    cal->bmp_ref_pressure_pa   = CAL_BMP_STD_SEA_LEVEL_PA;
    cal->bmp_locked_press_pa   = CAL_BMP_STD_SEA_LEVEL_PA;
    cal->bmp_filtered_press_pa = CAL_BMP_STD_SEA_LEVEL_PA;
    cal->bmp_filtered_temp_c   = 25.0f;
    cal->bmp_locked_temp_c     = 25.0f;
    cal->bmp_abs_altitude_m    = 0.0f;
    cal->bmp_rel_altitude_m    = 0.0f;
    cal->bmp_prev_rel_alt_m    = 0.0f;
    cal->bmp_vert_vel_mps      = 0.0f;
    cal->bmp_calibrated        = false;
}

/**
 * @brief Step 1 Calibration (Stationary MPU Gyro & Accel):
 *        Computes Gyroscope Zero-Rate Offset (bias) and Accelerometer level
 *        offsets while the board is held still on a flat surface (Z-axis = +1g).
 */
static void Main_CalibrateStationaryGyroAndAccel(MPU9250_Handle_t *mpu, Main_Calibration_t *cal)
{
    MPU9250_RawData_t sample;
    float gyro_sum[3]  = {0.0f, 0.0f, 0.0f};
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    float prev_gyro[3] = {0.0f, 0.0f, 0.0f};
    bool  has_prev     = false;
    uint32_t valid_count  = 0U;
    uint32_t max_attempts = CAL_GYRO_ACCEL_SAMPLES * 3U;

    UART3_Printf("\r\n[CAL 1/2] Stationary MPU Gyro & Accel Calibration: Keep board flat & still...\r\n");
    MCU_LED_Set(NUCLEO_LED_BLUE_LD2, true);

    /* Discard initial warm-up samples so DLPF and MEMS charge pump settle */
    for (uint32_t w = 0U; w < CAL_WARMUP_DISCARD_SAMPLES; w++) {
        if (MPU9250_ReadAll9Axis(mpu, &sample) == MPU9250_OK) {
            prev_gyro[0] = sample.gyro_dps[0];
            prev_gyro[1] = sample.gyro_dps[1];
            prev_gyro[2] = sample.gyro_dps[2];
            has_prev = true;
        }
        MCU_DelayMs(5U);
    }

    for (uint32_t attempt = 0U; (attempt < max_attempts) && (valid_count < CAL_GYRO_ACCEL_SAMPLES); attempt++) {
        MPU9250_Status_t st = MPU9250_ReadAll9Axis(mpu, &sample);
        if (st != MPU9250_OK) {
            UART3_PrintMPUError("Calibration_ReadGyroAccel", st);
            MCU_DelayMs(5U);
            continue;
        }

        if (!has_prev) {
            prev_gyro[0] = sample.gyro_dps[0];
            prev_gyro[1] = sample.gyro_dps[1];
            prev_gyro[2] = sample.gyro_dps[2];
            has_prev = true;
        }

        float dgx = fabsf(sample.gyro_dps[0] - prev_gyro[0]);
        float dgy = fabsf(sample.gyro_dps[1] - prev_gyro[1]);
        float dgz = fabsf(sample.gyro_dps[2] - prev_gyro[2]);

        prev_gyro[0] = sample.gyro_dps[0];
        prev_gyro[1] = sample.gyro_dps[1];
        prev_gyro[2] = sample.gyro_dps[2];

        if ((dgx < CAL_GYRO_DELTA_STILL_DPS) &&
            (dgy < CAL_GYRO_DELTA_STILL_DPS) &&
            (dgz < CAL_GYRO_DELTA_STILL_DPS) &&
            (fabsf(sample.gyro_dps[0]) < CAL_GYRO_MAX_FACTORY_ZRO_DPS) &&
            (fabsf(sample.gyro_dps[1]) < CAL_GYRO_MAX_FACTORY_ZRO_DPS) &&
            (fabsf(sample.gyro_dps[2]) < CAL_GYRO_MAX_FACTORY_ZRO_DPS)) {
            for (uint8_t i = 0U; i < 3U; i++) {
                gyro_sum[i]  += sample.gyro_dps[i];
                accel_sum[i] += sample.accel_g[i];
            }
            valid_count++;
        }
        MCU_DelayMs(5U);
    }

    if (valid_count == CAL_GYRO_ACCEL_SAMPLES) {
        float inv_n = 1.0f / (float)valid_count;
        cal->gyro_bias_dps[0]     = gyro_sum[0] * inv_n;
        cal->gyro_bias_dps[1]     = gyro_sum[1] * inv_n;
        cal->gyro_bias_dps[2]     = gyro_sum[2] * inv_n;
        cal->prev_raw_gyro_dps[0] = cal->gyro_bias_dps[0];
        cal->prev_raw_gyro_dps[1] = cal->gyro_bias_dps[1];
        cal->prev_raw_gyro_dps[2] = cal->gyro_bias_dps[2];

        float mean_ax = accel_sum[0] * inv_n;
        float mean_ay = accel_sum[1] * inv_n;
        float mean_az = accel_sum[2] * inv_n;
        if (fabsf(mean_az) > 0.75f) {
            float expected_z = (mean_az >= 0.0f) ? 1.0f : -1.0f;
            cal->accel_offset_g[0] = mean_ax;
            cal->accel_offset_g[1] = mean_ay;
            cal->accel_offset_g[2] = mean_az - expected_z;
        }
        cal->stationary_count = CAL_ZUPT_MIN_HOLD_SAMPLES;
        cal->imu_calibrated   = true;
    }

    MCU_LED_Set(NUCLEO_LED_BLUE_LD2, false);
    UART3_Printf("[CAL 1/2] Done (%lu samples) | Gyro Bias: [%d, %d, %d] mdeg/s | Accel Off: [%d, %d, %d] mg\r\n",
                 (unsigned long)valid_count,
                 (int)(cal->gyro_bias_dps[0] * 1000.0f),
                 (int)(cal->gyro_bias_dps[1] * 1000.0f),
                 (int)(cal->gyro_bias_dps[2] * 1000.0f),
                 (int)(cal->accel_offset_g[0] * 1000.0f),
                 (int)(cal->accel_offset_g[1] * 1000.0f),
                 (int)(cal->accel_offset_g[2] * 1000.0f));
}

/**
 * @brief Step 2 Calibration (Stationary BMP585 Reference Pressure P0):
 *        1. Discards `CAL_BMP_WARMUP_DISCARD_SAMPLES` initial readings so the
 *           BMP585's internal IIR filter and thermal state settle completely.
 *        2. Averages `CAL_BMP_BASELINE_SAMPLES` settled readings so `P0` matches
 *           the true steady-state pressure and `dAlt` starts at 0.00 m.
 */
static void Main_CalibrateBMPReferencePressure(BMP585_Handle_t *bmp, Main_Calibration_t *cal, BMP585_Data_t *latest_bmp)
{
    BMP585_Data_t      sample;
    BMP585_IntStatus_t int_stat;
    float              press_sum_pa  = 0.0f;
    float              temp_sum_c    = 0.0f;
    uint32_t           warmup_count  = 0U;
    uint32_t           valid_count   = 0U;
    uint32_t           max_polls     = (CAL_BMP_WARMUP_DISCARD_SAMPLES + CAL_BMP_BASELINE_SAMPLES) * 20U;

    UART3_Printf("[CAL 2/2] Stationary BMP585 Baseline Pressure Calibration (%u warmup + %u avg samples @ 20 Hz)...\r\n",
                 (unsigned int)CAL_BMP_WARMUP_DISCARD_SAMPLES,
                 (unsigned int)CAL_BMP_BASELINE_SAMPLES);
    MCU_LED_Set(NUCLEO_LED_BLUE_LD2, true);

    for (uint32_t poll = 0U; (poll < max_polls) && (valid_count < CAL_BMP_BASELINE_SAMPLES); poll++) {
        if (BMP585_GetIntStatus(bmp, &int_stat) == BMP585_OK) {
            if (int_stat.drdy_data_reg) {
                if (BMP585_ReadSensorData(bmp, &sample) == BMP585_OK) {
                    if ((sample.pressure_pa > 30000.0f) && (sample.pressure_pa < 120000.0f)) {
                        if (warmup_count < CAL_BMP_WARMUP_DISCARD_SAMPLES) {
                            warmup_count++;
                        } else {
                            press_sum_pa += sample.pressure_pa;
                            temp_sum_c   += sample.temperature_c;
                            valid_count++;
                            if (latest_bmp != NULL) {
                                *latest_bmp = sample;
                            }
                        }
                    }
                }
            }
        }
        MCU_DelayMs(5U);
    }

    if (valid_count == CAL_BMP_BASELINE_SAMPLES) {
        float inv_n = 1.0f / (float)valid_count;
        cal->bmp_ref_pressure_pa   = press_sum_pa * inv_n;
        cal->bmp_locked_press_pa   = cal->bmp_ref_pressure_pa;
        cal->bmp_filtered_press_pa = cal->bmp_ref_pressure_pa;
        cal->bmp_filtered_temp_c   = temp_sum_c * inv_n;
        cal->bmp_locked_temp_c     = cal->bmp_filtered_temp_c;
        cal->bmp_abs_altitude_m    = 44330.0f * (1.0f - powf(cal->bmp_ref_pressure_pa / CAL_BMP_STD_SEA_LEVEL_PA, 0.190294957f));
        cal->bmp_rel_altitude_m    = 0.0f;
        cal->bmp_prev_rel_alt_m    = 0.0f;
        cal->bmp_vert_vel_mps      = 0.0f;
        cal->bmp_calibrated        = true;

        if (latest_bmp != NULL) {
            latest_bmp->pressure_pa   = cal->bmp_locked_press_pa;
            latest_bmp->pressure_hpa  = cal->bmp_locked_press_pa / 100.0f;
            latest_bmp->temperature_c = cal->bmp_locked_temp_c;
        }
    }

    MCU_LED_Set(NUCLEO_LED_BLUE_LD2, false);
    int32_t p0_int  = (int32_t)cal->bmp_ref_pressure_pa;
    int32_t p0_frac = (int32_t)((cal->bmp_ref_pressure_pa - (float)p0_int) * 100.0f + 0.5f);
    int32_t alt_int = (int32_t)cal->bmp_abs_altitude_m;
    UART3_Printf("[CAL 2/2] Done (%lu samples) | Ref Pressure P0: %ld.%02ld Pa | Base Altitude: ~%ld m ASL\r\n",
                 (unsigned long)valid_count,
                 (long)p0_int, (long)p0_frac,
                 (long)alt_int);
}

/**
 * @brief Applies MPU Gyro bias + Delta-Based Online ZUPT + Stationary Deadband
 *        and Accelerometer offset/scale in-place strictly inside Main.c.
 */
static void Main_ApplyIMUCalibration(Main_Calibration_t *cal, MPU9250_RawData_t *sample)
{
    /* 1. Apply Accelerometer offset & scale calibration */
    for (uint8_t i = 0U; i < 3U; i++) {
        sample->accel_g[i] = cal->accel_scale[i] * (sample->accel_g[i] - cal->accel_offset_g[i]);
    }

    cal->latest_accel_norm_g = sqrtf((sample->accel_g[0] * sample->accel_g[0]) +
                                     (sample->accel_g[1] * sample->accel_g[1]) +
                                     (sample->accel_g[2] * sample->accel_g[2]));

    /* 2. Delta-Based Online ZUPT (Zero Angular Rate Update) Stationary Detector */
    float dgx = fabsf(sample->gyro_dps[0] - cal->prev_raw_gyro_dps[0]);
    float dgy = fabsf(sample->gyro_dps[1] - cal->prev_raw_gyro_dps[1]);
    float dgz = fabsf(sample->gyro_dps[2] - cal->prev_raw_gyro_dps[2]);

    cal->prev_raw_gyro_dps[0] = sample->gyro_dps[0];
    cal->prev_raw_gyro_dps[1] = sample->gyro_dps[1];
    cal->prev_raw_gyro_dps[2] = sample->gyro_dps[2];

    if ((dgx < CAL_ZUPT_DELTA_THRESHOLD_DPS) &&
        (dgy < CAL_ZUPT_DELTA_THRESHOLD_DPS) &&
        (dgz < CAL_ZUPT_DELTA_THRESHOLD_DPS) &&
        (fabsf(sample->gyro_dps[0]) < CAL_GYRO_MAX_FACTORY_ZRO_DPS) &&
        (fabsf(sample->gyro_dps[1]) < CAL_GYRO_MAX_FACTORY_ZRO_DPS) &&
        (fabsf(sample->gyro_dps[2]) < CAL_GYRO_MAX_FACTORY_ZRO_DPS)) {
        if (cal->stationary_count < 10000U) {
            cal->stationary_count++;
        }
    } else {
        cal->stationary_count = 0U;
    }

    if (cal->stationary_count >= CAL_ZUPT_MIN_HOLD_SAMPLES) {
        for (uint8_t i = 0U; i < 3U; i++) {
            float residual = sample->gyro_dps[i] - cal->gyro_bias_dps[i];
            if (fabsf(residual) > 0.50f) {
                cal->gyro_bias_dps[i] = sample->gyro_dps[i];
            } else {
                cal->gyro_bias_dps[i] += CAL_ZUPT_BIAS_ALPHA * residual;
            }
            float omega = sample->gyro_dps[i] - cal->gyro_bias_dps[i];
            sample->gyro_dps[i] = (fabsf(omega) < CAL_GYRO_DEADBAND_DPS) ? 0.0f : omega;
        }
    } else {
        for (uint8_t i = 0U; i < 3U; i++) {
            sample->gyro_dps[i] = sample->gyro_dps[i] - cal->gyro_bias_dps[i];
        }
    }
}

/**
 * @brief Stabilizes BMP585 pressure, temperature, relative altitude (`dAlt`),
 *        and vertical velocity (`Vz`) strictly inside Main.c using:
 *        1. Motion-Adaptive Exponential Moving Average (EMA) on Pressure & Temp.
 *        2. IMU-Aided Stationary Barometric Lock (Baro ZUPT):
 *           When the 100 Hz IMU confirms the board is stationary (`stationary_count >= 20`
 *           and `|accel_norm - 1g| < 0.04g`) and pressure is within the ambient air
 *           turbulence window (`< 1.80 Pa`), slowly tracks ambient weather drift into
 *           `bmp_ref_pressure_pa`, locks `pressure_pa` & `temperature_c` inside a
 *           deadband, and clamps `dAlt = 0.00 m` and `Vz = 0.00 m/s`.
 *        3. Instantaneous Breakout on Real Vertical Motion:
 *           As soon as the board moves or pressure shifts beyond the stationary window,
 *           tracks real pressure, altitude change `dAlt`, and vertical climb rate `Vz`.
 */
static void Main_ApplyBMPCalibration(Main_Calibration_t *cal, BMP585_Data_t *bmp_sample, float bmp_dt_sec)
{
    if ((bmp_sample == NULL) ||
        (bmp_sample->pressure_pa < 30000.0f) ||
        (bmp_sample->pressure_pa > 120000.0f)) {
        return;
    }

    if (!cal->bmp_calibrated) {
        cal->bmp_ref_pressure_pa   = bmp_sample->pressure_pa;
        cal->bmp_locked_press_pa   = bmp_sample->pressure_pa;
        cal->bmp_filtered_press_pa = bmp_sample->pressure_pa;
        cal->bmp_filtered_temp_c   = bmp_sample->temperature_c;
        cal->bmp_locked_temp_c     = bmp_sample->temperature_c;
        cal->bmp_calibrated        = true;
    }

    /* Check whether the 100 Hz IMU confirms the sensor is physically stationary */
    bool imu_is_still = (cal->stationary_count >= CAL_ZUPT_MIN_HOLD_SAMPLES) &&
                        (fabsf(cal->latest_accel_norm_g - 1.0f) < 0.04f);

    float raw_diff_from_ref = fabsf(bmp_sample->pressure_pa - cal->bmp_ref_pressure_pa);
    bool  baro_is_still     = imu_is_still && (raw_diff_from_ref < CAL_BMP_STILL_MAX_DRIFT_PA);

    /* 1. Motion-adaptive low-pass filter on pressure & temperature */
    // Keep stronger pressure smoothing whenever the IMU is still, even when
    // pressure leaves the P0 window. Retain pressure breakout for real height changes.
    float press_alpha = imu_is_still ? CAL_BMP_PRESS_EMA_ALPHA_STILL : CAL_BMP_PRESS_EMA_ALPHA_MOVING;
    cal->bmp_filtered_press_pa += press_alpha * (bmp_sample->pressure_pa - cal->bmp_filtered_press_pa);
    cal->bmp_filtered_temp_c   += CAL_BMP_TEMP_EMA_ALPHA * (bmp_sample->temperature_c - cal->bmp_filtered_temp_c);

    /* 2. Temperature hysteresis deadband (0.03 C) to eliminate +/-0.01 C LSB flicker */
    if (fabsf(cal->bmp_filtered_temp_c - cal->bmp_locked_temp_c) >= CAL_BMP_TEMP_DEADBAND_C) {
        cal->bmp_locked_temp_c = cal->bmp_filtered_temp_c;
    }
    bmp_sample->temperature_c = cal->bmp_locked_temp_c;

    /* 3. IMU-Aided Stationary Barometric Lock (Baro ZUPT) vs. Active Vertical Motion */
    if (baro_is_still) {
        /* Slowly track ambient room/weather barometric drift into P0 so dAlt stays at 0.00 m */
        cal->bmp_ref_pressure_pa += CAL_BMP_STILL_REF_TRACK_ALPHA *
                                    (cal->bmp_filtered_press_pa - cal->bmp_ref_pressure_pa);

        /* Hold displayed pressure rock-steady unless it drifts past the deadband */
        if (fabsf(cal->bmp_filtered_press_pa - cal->bmp_locked_press_pa) >= CAL_BMP_PRESS_DEADBAND_PA) {
            cal->bmp_locked_press_pa = cal->bmp_filtered_press_pa;
        }
        bmp_sample->pressure_pa  = cal->bmp_locked_press_pa;
        bmp_sample->pressure_hpa = cal->bmp_locked_press_pa / 100.0f;

        /* Zero out stationary relative altitude & vertical velocity noise */
        cal->bmp_rel_altitude_m  = 0.0f;
        cal->bmp_prev_rel_alt_m  = 0.0f;
        cal->bmp_vert_vel_mps    = 0.0f;
    } else {
        /* Active motion: update displayed pressure and compute relative altitude + Vz */
        if (fabsf(cal->bmp_filtered_press_pa - cal->bmp_locked_press_pa) >= 0.08f) {
            cal->bmp_locked_press_pa = cal->bmp_filtered_press_pa;
        }
        bmp_sample->pressure_pa  = cal->bmp_locked_press_pa;
        bmp_sample->pressure_hpa = cal->bmp_locked_press_pa / 100.0f;

        float new_rel_alt = 44330.0f * (1.0f - powf(cal->bmp_filtered_press_pa / cal->bmp_ref_pressure_pa, 0.190294957f));
        if (fabsf(new_rel_alt) < CAL_BMP_ALT_DEADBAND_M) {
            new_rel_alt = 0.0f;
        }

        if (bmp_dt_sec > 0.010f) {
            float raw_vz = (new_rel_alt - cal->bmp_prev_rel_alt_m) / bmp_dt_sec;
            cal->bmp_vert_vel_mps += 0.20f * (raw_vz - cal->bmp_vert_vel_mps);
            if (fabsf(cal->bmp_vert_vel_mps) < CAL_BMP_VZ_DEADBAND_MPS) {
                cal->bmp_vert_vel_mps = 0.0f;
            }
        }

        cal->bmp_prev_rel_alt_m = new_rel_alt;
        cal->bmp_rel_altitude_m = new_rel_alt;
    }

    cal->bmp_abs_altitude_m = 44330.0f * (1.0f - powf(bmp_sample->pressure_pa / CAL_BMP_STD_SEA_LEVEL_PA, 0.190294957f));
}

/* ========================================================================== */
/* 3. MAIN APPLICATION ENTRY POINT                                            */
/* ========================================================================== */




struct MPU9250_Biquad3Axis_t { float b0,b1,b2,a1,a2,z1[3],z2[3]; bool initialized; };
static void Biquad_InitButterworthLPF(MPU9250_Biquad3Axis_t *bq, float cutoff_hz, float fs_hz)
{
    if ((bq == NULL) || (fs_hz <= 0.0f) || (cutoff_hz <= 0.0f) || (cutoff_hz >= (0.49f * fs_hz))) {
        return;
    }

    float omega = tanf((float)M_PI * cutoff_hz / fs_hz);
    float omega2 = omega * omega;
    float sqrt2_omega = 1.41421356237f * omega;
    float norm = 1.0f / (1.0f + sqrt2_omega + omega2);

    bq->b0 = omega2 * norm;
    bq->b1 = 2.0f * bq->b0;
    bq->b2 = bq->b0;
    bq->a1 = 2.0f * (omega2 - 1.0f) * norm;
    bq->a2 = (1.0f - sqrt2_omega + omega2) * norm;

    for (uint8_t i = 0U; i < 3U; i++) {
        bq->z1[i] = 0.0f;
        bq->z2[i] = 0.0f;
    }
    bq->initialized = false;
}

static void Biquad_Process3Axis(MPU9250_Biquad3Axis_t *bq, const float in[3], float out[3])
{
    if (!bq->initialized) {
        /* Pre-load filter state to steady-state input to eliminate startup transient step response */
        float z1_coeff = bq->b1 + bq->b2 - (bq->a1 + bq->a2);
        float z2_coeff = bq->b2 - bq->a2;
        for (uint8_t i = 0U; i < 3U; i++) {
            bq->z1[i] = in[i] * z1_coeff;
            bq->z2[i] = in[i] * z2_coeff;
            out[i]    = in[i];
        }
        bq->initialized = true;
        return;
    }

    for (uint8_t i = 0U; i < 3U; i++) {
        float x = in[i];
        float y = (bq->b0 * x) + bq->z1[i];
        bq->z1[i] = (bq->b1 * x) - (bq->a1 * y) + bq->z2[i];
        bq->z2[i] = (bq->b2 * x) - (bq->a2 * y);
        out[i] = y;
    }
}



Main_Calibration_t calibration;
MPU9250_Biquad3Axis_t accelFilter,gyroFilter;
float accel[3],gyro[3];
BMP585_Data_t latestBMP;
bool ready=false,haveIMU=false,haveBMP=false;
uint32_t lastIMU=0,lastBMP=0,lastSend=0,lastIMUPoll=0,lastBMPPoll=0;
uint32_t lateSamples=0,lastReport=0;

// D6 button to GND; D0 -> series resistor -> green LED -> GND.
constexpr uint8_t CAL_BUTTON_PIN = 12, CAL_LED_PIN = 16;
constexpr uint32_t DOUBLE_PRESS_MS = 400, BUTTON_RELEASE_MS = 40;
volatile bool buttonPressEvent = false;
volatile bool calibrationButtonArmed = false;
bool calibrationLedOn = false, releaseTiming = false, firstPressPending = false;
uint32_t calibrationLedStarted = 0, releasedAt = 0, firstPressAt = 0;
// 0 = none, 1 = barometer only, 2 = full table calibration.
uint8_t calibrationRequest = 0;

void IRAM_ATTR calibrationButtonISR() {
  if (calibrationButtonArmed) {
    calibrationButtonArmed = false;
    buttonPressEvent = true;
  }
}

void showCalibrationSuccess() {
  digitalWrite(CAL_LED_PIN, HIGH);
  calibrationLedStarted = millis();
  calibrationLedOn = true;
}

void serviceCalibrationControls() {
  uint32_t now = millis();
  if (calibrationLedOn && uint32_t(now-calibrationLedStarted) >= 5000) {
    digitalWrite(CAL_LED_PIN, LOW);
    calibrationLedOn = false;
  }
  if (buttonPressEvent) {
    // ISR is disarmed until a stable release, so clearing this cannot lose bounce.
    buttonPressEvent = false;
    releaseTiming = false;
    if (firstPressPending) {
      calibrationRequest = uint32_t(now-firstPressAt) <= DOUBLE_PRESS_MS ? 2 : 1;
      firstPressPending = false;
    } else {
      firstPressPending = true;
      firstPressAt = now;
    }
  }
  if (firstPressPending && uint32_t(now-firstPressAt) >= DOUBLE_PRESS_MS) {
    calibrationRequest = 1;
    firstPressPending = false;
  }
  if (calibrationRequest) {
    calibrationButtonArmed = false;
    return;
  }
  // Rearm only after a stable release: holding or bouncing cannot double-click.
  if (!calibrationButtonArmed) {
    if (digitalRead(CAL_BUTTON_PIN) == HIGH) {
      if (!releaseTiming) { releasedAt = now; releaseTiming = true; }
      else if (uint32_t(now-releasedAt) >= BUTTON_RELEASE_MS) {
        releaseTiming = false;
        calibrationButtonArmed = true;
      }
    } else releaseTiming = false;
  }
}

void runButtonCalibration() {
  const bool fullCalibration = calibrationRequest == 2;
  calibrationButtonArmed = false;
  calibrationRequest = 0;
  buttonPressEvent = false;
  firstPressPending = false;
  releaseTiming = false;
  digitalWrite(CAL_LED_PIN, LOW);
  calibrationLedOn = false;
  if (!mpuOK || !bmpOK || !radioOK) {
    Serial.println("Calibration unavailable: fix hardware initialization and reset.");
    return;
  }
  if (!fullCalibration && !calibration.imu_calibrated) {
    Serial.println("IMU not calibrated: place flat on table and double-press D6.");
    return;
  }

  // Work on a copy so a failed attempt cannot destroy an existing calibration.
  Main_Calibration_t nextCalibration = calibration;
  BMP585_Data_t nextBMP = latestBMP;
  if (fullCalibration) {
    Serial.println("FULL calibration: keep board FLAT on table, Z up, and STILL.");
    Main_InitCalibrationDefaults(&nextCalibration);
    Main_CalibrateStationaryGyroAndAccel(nullptr,&nextCalibration);
  } else {
    Serial.println("BAROMETER calibration: keep still in the worn position.");
    nextCalibration.bmp_calibrated = false;
  }
  Main_CalibrateBMPReferencePressure(nullptr,&nextCalibration,&nextBMP);
  if (!nextCalibration.imu_calibrated || !nextCalibration.bmp_calibrated) {
    Serial.println("Calibration failed; previous calibration retained. LED stays off.");
    return;
  }
  calibration = nextCalibration;
  latestBMP = nextBMP;
  if (fullCalibration) {
    // Original filter settings; single press preserves IMU biases and filters.
    Biquad_InitButterworthLPF(&accelFilter,15.0f,100.0f);
    Biquad_InitButterworthLPF(&gyroFilter,15.0f,100.0f);
  }
  // Require fresh samples after the pause; exclude calibration time from dt.
  haveIMU = haveBMP = false;
  lastIMU = lastBMP = lastIMUPoll = lastBMPPoll = micros();
  lastSend = lastReport = millis();
  lateSamples = 0;
  ready = true;
  showCalibrationSuccess();
  Serial.println("Calibration successful. Green LED on for 5 seconds.");
}

void firmwareSetup() {
  pinMode(CAL_LED_PIN, OUTPUT);
  digitalWrite(CAL_LED_PIN, LOW);
  initHardware();
  // Set pull-up after radio.init() configures its unused RX input.
  pinMode(CAL_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CAL_BUTTON_PIN), calibrationButtonISR, FALLING);
  if (!mpuOK || !bmpOK || !radioOK) {
    Serial.println("Both sensors and RF must initialize. Fix error and reset."); return;
  }
  Serial.println("Keep board FLAT and STILL throughout calibration (about 7 seconds).");
  Main_InitCalibrationDefaults(&calibration);
  Main_CalibrateStationaryGyroAndAccel(nullptr,&calibration);
  Main_CalibrateBMPReferencePressure(nullptr,&calibration,&latestBMP);
  if (!calibration.imu_calibrated || !calibration.bmp_calibrated) {
    Serial.println("Calibration incomplete. Keep flat/still and reset; no CSV sent."); return;
  }
  Biquad_InitButterworthLPF(&accelFilter,15.0f,100.0f);
  Biquad_InitButterworthLPF(&gyroFilter,15.0f,100.0f);
  ready=true;
  showCalibrationSuccess();
  Serial.println("Ready: accX,accY,accZ,gyrX,gyrY,gyrZ,pressure_kPa");
}
// Decimal formatting: two places for IMU, three for pressure in kPa.
// Round away from zero on ties and suppress negative zero.
bool appendValue(char *line,size_t capacity,size_t &used,float value,bool comma,uint8_t decimals) {
  if (!isfinite(value) || fabsf(value)>1000000.0f || (decimals!=2 && decimals!=3)) return false;
  uint32_t scale = decimals==3 ? 1000UL : 100UL;
  uint32_t units=uint32_t(fabsf(value)*float(scale)+0.5f);
  int n=snprintf(line+used,capacity-used,"%s%lu.%0*lu%s",
    value<0 && units ? "-" : "",(unsigned long)(units/scale),
    int(decimals),(unsigned long)(units%scale),comma ? "," : "");
  if(n<0 || size_t(n)>=capacity-used) return false;
  used+=size_t(n); return true;
}
void firmwareLoop() {
  serviceCalibrationControls();
  if (calibrationRequest) {
    runButtonCalibration();
    return;
  }
  if(!ready) {delay(10);return;}
  uint32_t now=micros();
  // Data-ready polling ensures each filter update consumes a new sample.
  if(uint32_t(now-lastIMUPoll)>=1000) {
    lastIMUPoll=now;
    if(imuReady()) {
      MPU9250_RawData_t sample;
      if(readIMU(sample)) {
        uint32_t taken=micros();
        if(haveIMU && uint32_t(taken-lastIMU)>15000) ++lateSamples;
        lastIMU=taken;
        Main_ApplyIMUCalibration(&calibration,&sample);
        Biquad_Process3Axis(&accelFilter,sample.accel_g,accel);
        Biquad_Process3Axis(&gyroFilter,sample.gyro_dps,gyro);
        haveIMU=true;
      }
    }
  }
  now=micros();
  if(uint32_t(now-lastBMPPoll)>=5000) {
    lastBMPPoll=now;
    BMP585_IntStatus_t status;
    if(BMP585_GetIntStatus(nullptr,&status)==0 && status.drdy_data_reg) {
      BMP585_Data_t sample;
      if(BMP585_ReadSensorData(nullptr,&sample)==0 && sample.pressure_pa>30000 && sample.pressure_pa<120000) {
        float dt=haveBMP ? uint32_t(micros()-lastBMP)*1.0e-6f : 0.05f;
        // Don't let a stale IMU stationary flag lock new barometer readings.
        if(!haveIMU || uint32_t(micros()-lastIMU)>50000) calibration.stationary_count=0;
        Main_ApplyBMPCalibration(&calibration,&sample,dt);
        latestBMP=sample; lastBMP=micros(); haveBMP=true;
      }
    }
  }
  // USB output runs independently of RF packet airtime.
  // No waiting for RF: busy radio skips this snapshot, while USB still receives it.
  uint32_t ms=millis();
  if(uint32_t(ms-lastSend)>=OUTPUT_INTERVAL_MS) {
    lastSend=ms;
    if(haveIMU && haveBMP && uint32_t(micros()-lastIMU)<50000 && uint32_t(micros()-lastBMP)<250000) {
      float values[7]={accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2],latestBMP.pressure_pa / 1000.0f};
      char csv[61]; size_t used=0; bool ok=true;
      for(uint8_t i=0;i<7 && ok;++i) ok=appendValue(csv,sizeof(csv),used,values[i],i<6,i==6 ? 3 : 2);
      if(ok && used<=radio.maxMessageLength()) {
        Serial.println(csv); // Nominal 50 Hz USB data for training/plotting.
        if(radio.mode()!=RHGenericDriver::RHModeTx) {
          if(!radio.send((const uint8_t *)csv,uint8_t(used))) Serial.println("RF send failed");
        }
      } else Serial.println("CSV exceeds packet limit or invalid data; skipped");
    } else Serial.println("Stale/missing sensor reading; CSV skipped");
  }
  if(uint32_t(ms-lastReport)>=10000) {
    lastReport=ms;
    if(lateSamples) {Serial.printf("Timing: %lu IMU intervals >15ms\n",(unsigned long)lateSamples);lateSamples=0;}
  }
  delay(0);
}

} // namespace Firmware
void setup() { Firmware::firmwareSetup(); }
void loop() { Firmware::firmwareLoop(); }


