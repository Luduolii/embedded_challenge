#include <Adafruit_CircuitPlayground.h>
#include "arduinoFFT.h"

#define FFT_SAMPLES 128
#define SAMPLE_RATE_HZ 42.67

double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_SAMPLES, SAMPLE_RATE_HZ);

// === CONFIG OPTIONS ===
const bool USE_Z_AXIS_ONLY = true;
const double ENERGY_THRESHOLD = 20.0;      // reject noise
const double VARIANCE_THRESHOLD = 0.01;    // reject idle
const double TREMOR_ENERGY_UPPERLIMIT = 800; // empirical value, can be changed
const double DYSK_ENERGY_UPPERLIMIT = 10000; // empirical value, can be changed
const double DOMINANCE_RATIO = 1.5;        // must dominate to classify

void setup() {
  Serial.begin(115200);  // Teleplot recommends >=115200
  CircuitPlayground.begin();
}

void loop() {
  // 1. Collect sensor data
  for (int i = 0; i < FFT_SAMPLES; i++) {
    double mag;
    if (USE_Z_AXIS_ONLY) {
      mag = CircuitPlayground.motionZ();
    } else {
      float ax = CircuitPlayground.motionX();
      float ay = CircuitPlayground.motionY();
      float az = CircuitPlayground.motionZ();
      mag = sqrt(ax * ax + ay * ay + az * az);
    }
    vReal[i] = mag;
    vImag[i] = 0;
    delay(1000 * 3 / FFT_SAMPLES);  // ≈23ms
  }

  // 2. Remove DC offset
  FFT.dcRemoval(vReal, FFT_SAMPLES);

  // 3. Variance check
  double variance = 0;
  for (int i = 0; i < FFT_SAMPLES; i++) {
    variance += vReal[i] * vReal[i];
  }
  variance /= FFT_SAMPLES;

  if (variance < VARIANCE_THRESHOLD) {
    Serial.println("output:0");  // No movement
    Serial.println("tremorEnergy:0");
    Serial.println("dyskinesiaEnergy:0");
    Serial.println("totalEnergy:0");
    delay(1000);
    return;
  }

  // 4. FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // 5. Energy computation
  double band3_5Hz = 0;
  double band5_7Hz = 0;
  double totalEnergy = 0;

  for (int i = 1; i < FFT_SAMPLES / 2; i++) {
    double freq = (i * SAMPLE_RATE_HZ) / FFT_SAMPLES;
    double power = vReal[i] * vReal[i];
    totalEnergy += power;

    if (freq >= 3.0 && freq < 5.0) {
      band3_5Hz += power;
    } else if (freq >= 5.0 && freq < 7.0) {
      band5_7Hz += power;
    }
  }

  // 6. Classification
  int outputCode = 0;
  bool tremor_within_limit = band5_7Hz < TREMOR_ENERGY_UPPERLIMIT;
  bool dysk_within_limit = band3_5Hz < DYSK_ENERGY_UPPERLIMIT;

  if (totalEnergy < ENERGY_THRESHOLD) {
    outputCode = 0;  // no strong signal
  } else if ((band5_7Hz > band3_5Hz * DOMINANCE_RATIO) && tremor_within_limit) {
    outputCode = 1;  // Tremor (5–7 Hz dominates)
  } else if ((band3_5Hz > band5_7Hz * DOMINANCE_RATIO) && dysk_within_limit) {
    outputCode = 2;  // Dyskinesia (3–5 Hz dominates)
  } else {
    outputCode = 3; // Not defined movement
  }

  // 7. Teleplot-compatible serial output
  Serial.print("tremorEnergy:");
  Serial.println(band5_7Hz);
  Serial.print("dyskinesiaEnergy:");
  Serial.println(band3_5Hz);
  Serial.print("totalEnergy:");
  Serial.println(totalEnergy);
  Serial.print("output:");
  Serial.println(outputCode);

  delay(1000);
}
