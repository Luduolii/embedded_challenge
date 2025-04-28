#include "arduinoFFT.h"
#include <Adafruit_CircuitPlayground.h>

/* ---------- tunables ---------- */
const uint16_t   FFT_SAMPLES   = 512;      
const float      SAMPLE_RATE_HZ = 100.0;   // 100 Hz → 3 s ≈ 300 samples → zero-pad to 512
const float      MOVE_THRESH_G = 0.05;     // below this (≈ ±0.05 g) we think it's “not held”
const uint16_t   HOLD_OFF_MS   = 3000;     // capture window ≈ 3 s
const float      TREMOR_MIN_HZ = 3.0, TREMOR_MAX_HZ = 5.0;
const float      DYSK_MIN_HZ   = 5.0, DYSK_MAX_HZ   = 7.0;

/* ---------- globals ---------- */
double vReal[512];
double vImag[512];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_SAMPLES, SAMPLE_RATE_HZ);

uint16_t  sampleIndex   = 0;
uint32_t  lastSampleMic = 0;
bool      collecting    = false;
bool      windowReady   = false;


/* ---------- helper: g-force magnitude ---------- */
float readAccelG()
{
  float ax = CircuitPlayground.motionX();
  float ay = CircuitPlayground.motionY();
  float az = CircuitPlayground.motionZ();

  // magnitude, then convert to g
  return sqrtf(ax * ax + ay * ay + az * az) / 9.80665f;
}

/* ---------- Arduino life-cycle ---------- */
void setup()
{
  CircuitPlayground.begin();
  CircuitPlayground.setAccelRange(LIS3DH_RANGE_4_G);
  CircuitPlayground.clearPixels();
  
  Serial.begin(115200); // Just for testing -- print out whether the motion is T or D
  while(!Serial);
}

void loop()
{
  /* ------ 1. decide whether the board is being held  ------ */
  float g = readAccelG();
  static uint32_t lastMovement = 0;
  if (fabs(g - 1.0) > MOVE_THRESH_G)               // ≠ gravity-only
      lastMovement = millis();

  bool held = (millis() - lastMovement < HOLD_OFF_MS);

  /* ------ 2. start or stop capture ------ */
  if (held && !collecting) {                 // new capture window
      sampleIndex = 0;
      collecting  = true;
  }
  if (!held && collecting) {                 // aborted because idle
      collecting  = false;
  }

  /* ------ 3. timed sampling @ SAMPLE_RATE_HZ ------ */
  if (collecting) {
      uint32_t nowMic = micros();
      if (nowMic - lastSampleMic >= (1e6 / SAMPLE_RATE_HZ)) {
          lastSampleMic = nowMic;
          vReal[sampleIndex] = (double)((g - 1.0) * 1000.0);  // convert to milli-g centred on 0
          vImag[sampleIndex] = 0.0;
          sampleIndex++;
          if (sampleIndex >= FFT_SAMPLES) {      // buffer full
              collecting  = false;
              windowReady = true;
          }
      }
  }

  /* ------ 4. run FFT when a window is ready ------ */
  if (windowReady) {
      /* 4a. windowing & FFT */
      FFT.windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);
      FFT.compute(FFT_FORWARD);
      FFT.complexToMagnitude();

      /* 4b. energy in each band */
      float tremorEnergy = 0.0, dyskEnergy = 0.0;
      for (uint16_t i = 1; i < FFT_SAMPLES / 2; ++i) {     // Nyquist/2
          float freq = (i * SAMPLE_RATE_HZ) / FFT_SAMPLES;
          if (freq >= TREMOR_MIN_HZ && freq <= TREMOR_MAX_HZ)
              tremorEnergy += vReal[i];
          else if (freq >= DYSK_MIN_HZ && freq <= DYSK_MAX_HZ)
              dyskEnergy += vReal[i];
      }

      /* 4c. classify (simple winner-takes-all) */
      const float ENERGY_THRESH = 50.0;           // empirical, adjust!
      if (tremorEnergy > ENERGY_THRESH || dyskEnergy > ENERGY_THRESH) {
          if (tremorEnergy > dyskEnergy) {
              /* ---- TODO: indicate Tremor detected ---- */
              Serial.println("This is TREMOR");
          } else {
              /* ---- TODO: indicate Dyskinesia detected ---- */
              Serial.println("This is DYSKINESIA");
          }
      } else {
          /* below threshold → no disorder detected */
      }

      windowReady = false;                        // wait for next capture
  }
}
