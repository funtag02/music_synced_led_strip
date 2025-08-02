#include <Arduino.h>
#include "main.h"
#include "led_strip.hpp"
#include "audio_signal.hpp"

void setup() {
  Serial.begin(115200); // base : 9600
  delay(1000);

  Serial.println("Setting up the LED Strip System...");

  /*
  if (initLedStrip() == 0) {
    Serial.println("initialized led strip !");
  } else {
    Serial.println("ERROR !!");
  }
  */

  if (initI2SMicrophone() == 0) {
    Serial.println("initialized inmp441 i2s mic !");
  } else {
    Serial.println("ERROR !!");
  }

  float noiseFloor[NUM_BANDS];
  calibrateNoiseFloor(noiseFloor, NUM_BANDS, 20);  // attends le silence ici

  Serial.println("system setup done.");
  Serial.flush();
}

void loop() {
  /*
  Serial.println(updateColorsOnLedStrip() == 0 
                    ? "Led Strip colors are successfully updated !" 
                    : "STRIP ERROR !");

  */

  /*
  if (updateColorsOnLedStrip() != 0) {
    Serial.println("STRIP ERROR !");
  }
  */

  // double dB = readVolume_dBFS();

  // if (dB) {
  //   Serial.printf("Sound level measured : %.2f dBFS\n", dB);
  // } else {
  //   Serial.println("MICROPHONE ERROR !!");
  // }

  // printFrequencySpectrum();

  detectDrums();

  // delay(2000);

}