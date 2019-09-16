#ifndef _MAX30100_H_
#define _MAX30100_H_

#include "esp_system.h"


typedef struct
{
	uint8_t wr_pointer;
	uint8_t ovf_counter;
	uint8_t rd_pointer;
} tFIFO_PTRs;



typedef enum SampleRate
{
	rate50hz = 50,
	rate100hz = 100,
	rate167hz = 167,
	rate200hz = 200,
	rate400hz = 400,
	rate600hz = 600,
	rate800hz = 800,
	rate1000hz = 1000
} eSampleRate;


typedef enum LedCurrent
{
	ledCurrent_0_0_mA = 0,
	ledCurrent_4_4_mA =	1,
	ledCurrent_7_6_mA =	2,
	ledCurrent_11_0_mA = 3,
	ledCurrent_14_2_mA = 4,
	ledCurrent_17_4_mA = 5,
	ledCurrent_20_8_mA = 6,
	ledCurrent_24_0_mA = 7,
	ledCurrent_27_1_mA = 8,
	ledCurrent_30_6_mA = 9,
	ledCurrent_33_8_mA = 10,
	ledCurrent_37_0_mA = 11,
	ledCurrent_40_2_mA = 12,
	ledCurrent_43_6_mA = 13,
	ledCurrent_46_8_mA = 14,
	ledCurrent_50_0_mA = 15
} eLedCurrent;

extern eSampleRate SampleRate;
//extern uint16_t BuffSize;
extern uint16_t ir_max, ir_min;



// Инициализация MAX30100
int max30100_init(uint8_t spO2_enable, eSampleRate sampleRate, uint8_t bufferedReading);
// Деинициализация MAX30100, освобождение ОЗУ
void max30100_deinit(void);
// Ревизия MAX30100
uint8_t max30100_getRevisionID();
// Номер партии MAX30100
uint8_t max30100_getPartID();
// Температура MAX30100
float max30100_getTemp();

// Старт измерения температуры MAX30100
void max30100_startTempMeasure();
// Выбор режима работы MAX30100
void max30100_setMode(uint8_t spO2_enable, uint8_t bufferedReading);
void max30100_setLedcurrent(eLedCurrent redLed, eLedCurrent irLed);
int max30100_setSampleRate(uint8_t spO2_enable, eSampleRate sampleRate);

void readLastSamples(uint16_t *ir_buff, uint16_t *red_buff, uint16_t samplesNum);




#endif
