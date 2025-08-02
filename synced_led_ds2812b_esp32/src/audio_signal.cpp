#include <driver/i2s.h>
#include <arduinoFFT.h>
#include "audio_signal.hpp"
#include <Arduino.h>

// --- Configuration FFT ---
static double vReal[SAMPLE_COUNT];
static double vImag[SAMPLE_COUNT];
static ArduinoFFT<double> FFT(vReal, vImag, SAMPLE_COUNT, SAMPLING_FREQ);

// --- Configuration bandes fréquence ---
typedef struct {
  const char* name;
  double freqLow;
  double freqHigh;
  float energy;
} Band;

Band bands[NUM_BANDS] = {
  {"Kick", 20, 100},
  {"Snare", 150, 250},
  {"HiHat", 5000, 10000},
  {"Clap", 1000, 3000}
};

// --- Configuration I2S ---
const i2s_config_t i2s_config = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = 44100,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
  .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
  .communication_format = I2S_COMM_FORMAT_STAND_I2S,
  .intr_alloc_flags = 0,
  .dma_buf_count = 8,
  .dma_buf_len = 64,
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0
};

const i2s_pin_config_t pin_config = {
  .bck_io_num = I2S_SCK,
  .ws_io_num = I2S_WS,
  .data_out_num = I2S_PIN_NO_CHANGE,
  .data_in_num = I2S_SD
};

int initI2SMicrophone() {
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);
  Serial.printf("I2S PINS — SCK: %d, WS: %d, SD: %d\n", I2S_SCK, I2S_WS, I2S_SD);
  return 0;
}

// --- Volume global

double readVolume_dBFS() {
  const int buffer_len = 64;
  int32_t buffer[buffer_len];
  size_t bytes_read;
  i2s_read(I2S_NUM_0, (void*)buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);
  if (bytes_read == 0) Serial.println("I2S read returned 0 bytes!");
  int samples_read = bytes_read / sizeof(int32_t);
  double sum = 0;
  for (int i = 0; i < samples_read; i++) {
    int32_t sample = buffer[i] >> 8;
    float normalized = sample / 8388608.0f;
    sum += normalized * normalized;
  }
  double rms = sqrt(sum / samples_read);
  double dB = 20.0 * log10(rms + 1e-9);
  return dB;
}

// --- Lecture + FFT
bool readAndComputeFFT() {
  const size_t bytes_needed = SAMPLE_COUNT * sizeof(int32_t);
  int32_t* raw_buffer = (int32_t*)malloc(bytes_needed);
  size_t bytes_read;
  if (!raw_buffer) {
    Serial.println("Erreur : mémoire insuffisante pour la FFT");
    return false;
  }
  esp_err_t result = i2s_read(I2S_NUM_0, raw_buffer, bytes_needed, &bytes_read, portMAX_DELAY);
  if (result != ESP_OK || bytes_read != bytes_needed) {
    Serial.println("Erreur de lecture I2S");
    free(raw_buffer);
    return false;
  }
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int32_t sample = raw_buffer[i] >> 8;
    vReal[i] = sample / 8388608.0f;
    vImag[i] = 0.0;
  }
  free(raw_buffer);
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();
  return true;
}

// --- Analyse de bandes de fréquences
void computeFrequencyBands() {
  for (int b = 0; b < NUM_BANDS; b++) bands[b].energy = 0;
  double binWidth = SAMPLING_FREQ / (double)SAMPLE_COUNT;
  for (int i = 1; i < SAMPLE_COUNT / 2; i++) {
    double freq = i * binWidth;
    double magnitude = vReal[i];
    for (int b = 0; b < NUM_BANDS; b++) {
      if (freq >= bands[b].freqLow && freq < bands[b].freqHigh) {
        bands[b].energy += magnitude;
        break;
      }
    }
  }
  for (int b = 0; b < NUM_BANDS; b++) {
    int binCount = (bands[b].freqHigh - bands[b].freqLow) / binWidth;
    if (binCount > 0) bands[b].energy /= binCount;
  }
}

// --- Calibrage bruit de fond ---
void calibrateNoiseFloor(float* noiseFloor, int numBands, int numIterations) {
  float accum[numBands];
  memset(accum, 0, sizeof(float) * numBands);
  Serial.println("Calibration du bruit de fond...");
  for (int n = 0; n < numIterations; n++) {
    if (!readAndComputeFFT()) continue;
    computeFrequencyBands();
    for (int b = 0; b < numBands; b++) accum[b] += bands[b].energy;
    delay(100);
  }
  for (int b = 0; b < numBands; b++) {
    noiseFloor[b] = accum[b] / numIterations;
    Serial.printf("Bruit [%s] : %.5f\n", bands[b].name, noiseFloor[b]);
  }
  Serial.println("Calibration terminée.\n");
}

// --- Affichage du spectre
void printFrequencySpectrum() {
  if (!readAndComputeFFT()) return;
  Serial.println("Spectre (Hz | Amplitude) :");
  for (int i = 1; i < SAMPLE_COUNT / 2; i++) {
    double freq = (i * 1.0 * SAMPLING_FREQ) / SAMPLE_COUNT;
    if (vReal[i] > 0.01) {
      Serial.printf("%5.0f Hz : %f\n", freq, vReal[i]);
    }
  }
}

// --- Détection des drums
void detectDrums() {
  if (!readAndComputeFFT()) return;
  struct DrumBand {
    const char* name;
    double freqLow;
    double freqHigh;
    double threshold;
    double energy;
  };

  DrumBand drumBands[] = {
    {"Kick", 50, 150, 0.02, 0.0},
    {"Snare", 150, 800, 0.015, 0.0},
    {"Clap", 2000, 4000, 0.01, 0.0},
    {"HiHat", 4000, 10000, 0.008, 0.0}
  };

  double bin_resolution = (double)SAMPLING_FREQ / (double)SAMPLE_COUNT;

  for (auto& band : drumBands) {
    int binStart = band.freqLow / bin_resolution;
    int binEnd = band.freqHigh / bin_resolution;
    for (int i = binStart; i <= binEnd && i < SAMPLE_COUNT / 2; i++) {
      band.energy += vReal[i];
    }
    band.energy /= (binEnd - binStart + 1);
  }

  Serial.print("[DetectDrums] ");
  bool anyDetected = false;
  for (auto& band : drumBands) {
    if (band.energy > band.threshold) {
      Serial.printf("%s (%.3f) ", band.name, band.energy);
      anyDetected = true;
    }
  }
  if (!anyDetected) Serial.print("Rien detecté");
  Serial.println();
}