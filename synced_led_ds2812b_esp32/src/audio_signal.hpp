#ifndef AUDIO_SIGNAL_HPP
#define AUDIO_SIGNAL_HPP

#define I2S_WS 18      // Word Select (LRCL)
#define I2S_SCK 19     // Serial Clock (SCK)
#define I2S_SD 5      // Serial Data (SD)

#define SAMPLE_COUNT 1024
#define SAMPLING_FREQ 44100  // Doit correspondre à ton i2s_config
#define NUM_BANDS 4

int initI2SMicrophone();
double readVolume_dBFS();
// void printFrequencySpectrum();
void detectDrums();
void calibrateNoiseFloor(float* noiseFloor, int numBands, int numIterations);  // attends le silence ici

#endif