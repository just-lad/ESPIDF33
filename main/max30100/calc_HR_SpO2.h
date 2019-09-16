#ifndef _CALC_HR_SPO2_H_
#define _CALC_HR_SPO2_H_

#include "esp_system.h"


#define MARK_VALUE		30000

// основная функция расчёта HR и SpO2
float calculate(uint8_t mode);

float getHeartRate();
float getR();
float getSpO2();
uint8_t getFingerPresent();


int16_t allocMemory(uint16_t buffSize);
void freeMemory();

uint16_t *readInputIrSamples(uint16_t samples, uint8_t syncStart);
uint16_t *readStage1IrSamples(uint16_t samples, uint8_t syncStart);
uint16_t *readStage2IrSamples(uint16_t samples, uint8_t syncStart);
uint16_t *readStage3IrSamples(uint16_t samples, uint8_t syncStart);
uint16_t *readInputRedSamples(uint16_t samples, uint8_t syncStart);
uint16_t *readStage1RedSamples(uint16_t samples, uint8_t syncStart);
uint16_t *readStage2RedSamples(uint16_t samples, uint8_t syncStart);

#endif
