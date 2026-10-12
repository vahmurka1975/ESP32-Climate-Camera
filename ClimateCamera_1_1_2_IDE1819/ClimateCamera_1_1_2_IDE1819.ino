/*
 * Климатическая камера ESP32 1.1.2 — Arduino IDE 1.8.19 / 2.x
 * Реализация логики находится в файле ClimateCamera.cpp.
 * Плата: ESP32 Dev Module (Espressif ESP32 core 2.0.17).
 */
#include "ClimateCamera.h"

void setup() {
  cameraSetup();
}

void loop() {
  cameraLoop();
}
