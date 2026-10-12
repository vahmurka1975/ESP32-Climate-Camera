/*
 * Климатическая камера ESP32 1.0.2 — Arduino IDE 1.8.19.
 * Открывайте этот файл из распакованной папки с тем же именем.
 * Вся реализация находится на вкладке ClimateCamera.cpp.
 * Пины и алгоритм ШИМ вентилятора сохранены без изменений.
 * ESP32 by Espressif Systems 2.0.17, плата ESP32 Dev Module.
 */
#include "ClimateCamera.h"

void setup() {
  cameraSetup();
}

void loop() {
  cameraLoop();
}
