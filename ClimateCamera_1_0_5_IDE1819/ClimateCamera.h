#ifndef CLIMATE_CAMERA_H
#define CLIMATE_CAMERA_H

/*
 * Климатическая камера ESP32 1.0.5 — Arduino IDE 1.8.19 / 2.x
 * Изменения v1.0.5: Task WDT, лимит компрессора 40 мин, Fail-Safe SHT30.
 */
void cameraSetup();
void cameraLoop();

#endif // CLIMATE_CAMERA_H
