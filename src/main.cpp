#include <Adafruit_CircuitPlayground.h>
#include "arduinoFFT.h"


#define FFT_SAMPLES 128
#define SAMPLE_RATE_HZ 42.67


double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];


ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_SAMPLES, SAMPLE_RATE_HZ);


// === CONFIG OPTIONS ===
const bool USE_Z_AXIS_ONLY = true;
const double ENERGY_THRESHOLD = 20.0;
const double VARIANCE_THRESHOLD = 0.01;
const double TREMOR_ENERGY_UPPERLIMIT = 800;
const double DYSK_ENERGY_UPPERLIMIT = 10000;
const double DOMINANCE_RATIO = 1.5;


// === FUNCTION PROTOTYPES ===
void indicateWithLED(int code, float totalEnergy);
float mapEnergy(float value, float minE, float maxE);


void setup() {
  Serial.begin(115200);
  CircuitPlayground.begin();
  CircuitPlayground.strip.setBrightness(50);
  CircuitPlayground.strip.clear();
  CircuitPlayground.strip.show();
}


void loop() {
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


  // DC Offset removal
  FFT.dcRemoval(vReal, FFT_SAMPLES);


  // Variance check
  double variance = 0;
  for (int i = 0; i < FFT_SAMPLES; i++) {
    variance += vReal[i] * vReal[i];
  }
  variance /= FFT_SAMPLES;


  if (variance < VARIANCE_THRESHOLD) {
    Serial.println("output:0");
    Serial.println("tremorEnergy:0");
    Serial.println("dyskinesiaEnergy:0");
    Serial.println("totalEnergy:0");
    indicateWithLED(0, 0);  // Green, all on
    delay(1000);
    return;
  }


  // FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();


  // Energy computation
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


  // Classification
  int outputCode = 0;
  bool tremor_within_limit = band5_7Hz < TREMOR_ENERGY_UPPERLIMIT;
  bool dysk_within_limit = band3_5Hz < DYSK_ENERGY_UPPERLIMIT;


  if (totalEnergy < ENERGY_THRESHOLD) {
    outputCode = 0;
  } else if ((band5_7Hz > band3_5Hz * DOMINANCE_RATIO) && tremor_within_limit) {
    outputCode = 1;
  } else if ((band3_5Hz > band5_7Hz * DOMINANCE_RATIO) && dysk_within_limit) {
    outputCode = 2;
  } else {
    outputCode = 3;
  }


  // Serial output
  Serial.print("tremorEnergy:");
  Serial.println(band5_7Hz);
  Serial.print("dyskinesiaEnergy:");
  Serial.println(band3_5Hz);
  Serial.print("totalEnergy:");
  Serial.println(totalEnergy);
  Serial.print("output:");
  Serial.println(outputCode);


  // LED output
  indicateWithLED(outputCode, totalEnergy);
  delay(1000);
}


// === LED CONTROL ===
void indicateWithLED(int code, float totalEnergy) {
  uint32_t color;
  float intensity = mapEnergy(totalEnergy, 100, 10000);  // Normalize to [0, 1]
  int numPixels;


  // Set color and pixel count logic
  switch (code) {
    case 0: // No movement
    case 3: // Ambiguous
      color = CircuitPlayground.strip.Color(0, 255, 0);  // Green
      numPixels = 10;  // All on
      break;


    case 1: // Tremor
      color = CircuitPlayground.strip.Color(255, 0, 0);  // Red
      if (intensity < 0.5) {
        numPixels = round(intensity * 10 / 2);  // 0–5 pixels
      } else {
        numPixels = round(intensity * 10);      // Up to 10
      }
      break;


    case 2: // Dyskinesia
      color = CircuitPlayground.strip.Color(255, 255, 0);  // Yellow
      numPixels = round(intensity * 10);  // 0–10 pixels
      break;


    default:
      color = CircuitPlayground.strip.Color(0, 0, 255);  // Blue fallback
      numPixels = 0;
      break;
  }


  // Light up NeoPixels
  CircuitPlayground.strip.clear();
  for (int i = 0; i < numPixels; i++) {
    CircuitPlayground.strip.setPixelColor(i, color);
  }
  CircuitPlayground.strip.show();
}


// === ENERGY SCALING HELPER ===
float mapEnergy(float value, float minE, float maxE) {
  value = constrain(value, minE, maxE);
  return (value - minE) / (maxE - minE);
}
