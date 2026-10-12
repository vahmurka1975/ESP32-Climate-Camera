/*
 * Климатическая камера ESP32 1.0.5 — Arduino IDE 1.8.19 / 2.x
 * Открывайте этот файл из распакованной папки с тем же именем: ClimateCamera_1_0_5_IDE1819
 * Вся реализация находится на вкладках ClimateCamera.cpp и Sht30Reliable.cpp.
 * Пины и алгоритм ШИМ вентилятора сохранены без изменений.
 * Плата: ESP32 Dev Module (пакет ESP32 by Espressif Systems 2.0.17).
 */
#include "ClimateCamera.h"

void setup() {
  cameraSetup();
}

void loop() {
  cameraLoop();
}
