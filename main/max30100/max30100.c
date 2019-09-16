#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "calc_HR_SpO2.h"
#include "max30100.h"
#include "i2cm.h"
#include "esp_heap_caps.h"
#include <esp_err.h>
#include <string.h>

#define SlaveID		0x57

// Registers
#define REG_INT_STATUS  0x00  		// Which interrupts are tripped
#define REG_INT_ENABLE  0x01  		// Which interrupts are active
#define REG_FIFO_WR_PTR	0x02  		// Where data is being written
#define REG_OVRFLOW_CTR 0x03  		// Number of lost samples
#define REG_FIFO_RD_PTR 0x04  		// Where to read from
#define REG_FIFO_DATA   0x05  		// Ouput data buffer
#define REG_MODE_CONFIG 0x06  		// Control register
#define REG_SPO2_CONFIG 0x07  		// Oximetry settings
#define REG_LED_CONFIG  0x09  		// Pulse width and power of LEDs
#define REG_TEMP_INT    0x16  		// Temperature integer
#define REG_TEMP_FRAC   0x17  		// Temperature fraction
#define REG_REV_ID      0xFE  		// Part revision
#define REG_PART_ID     0xFF  		// Part ID, normally 0x11

// Флаги статусного регистра прерываний сенсора и разрешения прерываний
#define INT_A_FULL		(1<<7)		// Буфер FIFO почти заполнен (осталось место на 1 семпл)
#define INT_TEMP_RDY	(1<<6)		// Измерение температуры завершено
#define INT_HR_RDY		(1<<5)		//
#define INT_SPO2_RDY	(1<<4)		//
#define INT_PWR_RDY		(1<<0)		// Сенсор готов к сбору данных после сброса по питанию

// Флаги режимов сенсора
#define MODE_SHDN		(1<<7)		// Режим Shutdown
#define MODE_RESET		(1<<6)		// Программный сброс сенсора
#define MODE_TEMP_EN	(1<<3)		// Старт измерения температуры сенсора
#define MODE_MASK		0x7			// Маска режима работы
#define MODE_HR_ONLY	2			// Режим измерения только частоты пульса
#define MODE_HR_SPO2	3			// Старт измерения частоты пульса и оксометрии


#define SPO2CONFIG_HI_RES_EN	(1<<6)
#define SPO2CONFIG_50_SAMPLES	(0<<2)
#define SPO2CONFIG_100_SAMPLES	(1<<2)
#define SPO2CONFIG_167_SAMPLES	(2<<2)
#define SPO2CONFIG_200_SAMPLES	(3<<2)
#define SPO2CONFIG_400_SAMPLES	(4<<2)
#define SPO2CONFIG_600_SAMPLES	(5<<2)
#define SPO2CONFIG_800_SAMPLES	(6<<2)
#define SPO2CONFIG_1000_SAMPLES	(7<<2)
#define SPO2CONFIG_LED_200US	0
#define SPO2CONFIG_LED_400US	1
#define SPO2CONFIG_LED_800US	2
#define SPO2CONFIG_LED_1600US	3



static uint8_t BufferedReading = 0;
static uint8_t SkipData = 0;

static uint8_t RevisionBuff[] = {0, 0};
static float LastTemp = 0;
static uint8_t ModeConfig = 0;
eSampleRate SampleRate = rate200hz;

#define CYCLE_BUFF_RAM_TYPE			(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL)

// Циклический буфер приёма данных
typedef struct
{
	uint16_t ir;
	uint16_t red;
} tSample;

static tSample *cycle_buff;
static uint16_t BuffSize;
static uint16_t WrIdx;
static TaskHandle_t *pTaskH = NULL;

void max30100read_task(void* arg);



// Настройка прерываний сенсора MAX30100
static int max30100_enableInterrupts(uint8_t value)
{
	int result = i2cm_write(SlaveID, REG_INT_ENABLE, &value, 1);
	return result;
}

// Сброс указателей FIFO
static void max30100_resetFIFO()
{
	uint8_t Buff[] = {0, 0, 0};
	i2cm_write(SlaveID, REG_FIFO_WR_PTR, Buff, 3);
}

// Деинициализация MAX30100, освобождение ОЗУ
void max30100_deinit(void)
{
	SkipData = 1;
    vTaskDelay(10 / portTICK_RATE_MS);

	if (pTaskH)
		vTaskDelete(pTaskH);

	freeMemory();
	i2cm_deinit();
}

// Инициализация MAX30100
int max30100_init(uint8_t spO2_enable, eSampleRate sampleRate, uint8_t bufferedReading)
{
	i2cm_init();

	int result = i2cm_read(SlaveID, REG_REV_ID, RevisionBuff, 2);
	if (result != ESP_OK)
		return result;

	// Настройка сенсора
	max30100_setMode(spO2_enable, bufferedReading);
	max30100_setSampleRate(spO2_enable, sampleRate);

	if (spO2_enable)
		max30100_setLedcurrent(ledCurrent_24_0_mA, ledCurrent_24_0_mA);
	else
		max30100_setLedcurrent(ledCurrent_0_0_mA, ledCurrent_24_0_mA);

	max30100_startTempMeasure();

	BaseType_t error = xTaskCreatePinnedToCore(max30100read_task, "ProcessData", 2048, NULL, 1, pTaskH, 1);	// 0);
	if (error != pdPASS)
	{
		pTaskH = NULL;
		return -10001;
	}

	SkipData = 0;
	return ESP_OK;
}

// Ревизия MAX30100
uint8_t max30100_getRevisionID()
{
	return RevisionBuff[0];
}

// Номер партии MAX30100
uint8_t max30100_getPartID()
{
	return RevisionBuff[1];
}

// Старт измерения температуры MAX30100
void max30100_startTempMeasure()
{
	SkipData = 1;
    vTaskDelay(10 / portTICK_RATE_MS);

	ModeConfig |= MODE_TEMP_EN;
	i2cm_write(SlaveID, REG_MODE_CONFIG, &ModeConfig, 1);

	SkipData = 0;
}

//
void max30100_setLedcurrent(eLedCurrent redLed, eLedCurrent irLed)
{
	SkipData = 1;
    vTaskDelay(10 / portTICK_RATE_MS);

	uint8_t LedPa = (redLed << 4) | irLed;
	i2cm_write(SlaveID, REG_LED_CONFIG, &LedPa, 1);

	SkipData = 0;
}

//
int max30100_setSampleRate(uint8_t spO2_enable, eSampleRate sampleRate)
{
	SampleRate = sampleRate;

	SkipData = 1;
    vTaskDelay(10 / portTICK_RATE_MS);

	// Освобождение памяти для буферов
	freeMemory();
	if (cycle_buff)
		heap_caps_free(cycle_buff);

	// Выделение памяти для буферов (каждый буфер на 2 секунды и не меньше, чем ширина экрана)
	//BuffSize = sampleRate < 165 ? 165 : sampleRate;
	BuffSize = sampleRate <= 200 ? 260 : sampleRate;
	BuffSize <<= 1;

	// Выделение памяти для буферов
	cycle_buff = heap_caps_malloc(BuffSize * sizeof(tSample), CYCLE_BUFF_RAM_TYPE);
	int16_t result = allocMemory(BuffSize);

	if ((!cycle_buff) || (result < 0))
	{
		freeMemory();
		return -10000;
	}

	memset(cycle_buff, 0, BuffSize * sizeof(tSample));
	WrIdx = 0;

	uint8_t SpO2_config = 0;
	switch (sampleRate)
	{
	case rate50hz:
		SpO2_config = SPO2CONFIG_50_SAMPLES | SPO2CONFIG_LED_1600US; // | SPO2CONFIG_HI_RES_EN;
		break;
	case rate100hz:
		SpO2_config = SPO2CONFIG_100_SAMPLES | SPO2CONFIG_LED_1600US; // | SPO2CONFIG_HI_RES_EN;
		break;
	case rate167hz:
		SpO2_config = SPO2CONFIG_167_SAMPLES | SPO2CONFIG_LED_800US;
		break;
	case rate200hz:
		SpO2_config = SPO2CONFIG_200_SAMPLES | SPO2CONFIG_LED_800US;
		break;
	case rate400hz:
		SpO2_config = SPO2CONFIG_400_SAMPLES | SPO2CONFIG_LED_400US;
		break;
	case rate600hz:
		if (spO2_enable)
			SpO2_config = SPO2CONFIG_600_SAMPLES | SPO2CONFIG_LED_200US;
		else
			SpO2_config = SPO2CONFIG_600_SAMPLES | SPO2CONFIG_LED_400US;
		break;
	case rate800hz:
		if (spO2_enable)
			SpO2_config = SPO2CONFIG_800_SAMPLES | SPO2CONFIG_LED_200US;
		else
			SpO2_config = SPO2CONFIG_800_SAMPLES | SPO2CONFIG_LED_400US;
		break;
	case rate1000hz:
		if (spO2_enable)
			SpO2_config = SPO2CONFIG_1000_SAMPLES | SPO2CONFIG_LED_200US;
		else
			SpO2_config = SPO2CONFIG_1000_SAMPLES | SPO2CONFIG_LED_400US;
		break;
	default:
		SpO2_config = SPO2CONFIG_50_SAMPLES | SPO2CONFIG_LED_1600US | SPO2CONFIG_HI_RES_EN;
		break;
	}
	i2cm_write(SlaveID, REG_SPO2_CONFIG, &SpO2_config, 1);

	SkipData = 0;
	return 0;
}

// Выбор режима работы MAX30100
void max30100_setMode(uint8_t spO2_enable, uint8_t bufferedReading)
{
	BufferedReading = bufferedReading;

	ModeConfig &= ~MODE_MASK;
	if (spO2_enable)
		ModeConfig |= MODE_HR_SPO2;
	else
		ModeConfig |= MODE_HR_ONLY;
	i2cm_write(SlaveID, REG_MODE_CONFIG, &ModeConfig, 1);

	uint8_t IntMode = INT_TEMP_RDY;
	if (BufferedReading)
		IntMode |= INT_A_FULL;
	else
		IntMode |= INT_HR_RDY | INT_SPO2_RDY;

	max30100_enableInterrupts(IntMode);
}

// Температура MAX30100
float max30100_getTemp()
{
	return LastTemp;
}

//
void readLastSamples(uint16_t *ir_buff, uint16_t *red_buff, uint16_t samplesNum)
{
	uint16_t LastSample = WrIdx ? WrIdx - 1: BuffSize - 1;

	while (samplesNum--)
	{
		uint16_t Idx = LastSample + BuffSize - samplesNum;
		while (Idx >= BuffSize)
			Idx -= BuffSize;

		*(ir_buff++) = cycle_buff[Idx].ir;
		*(red_buff++) = cycle_buff[Idx].red;
	}
}

//
static void writeSample(tSample *sample)
{
	cycle_buff[WrIdx].ir = sample->ir;
	cycle_buff[WrIdx].red = sample->red;

	WrIdx++;
	if (WrIdx == BuffSize)
		WrIdx = 0;
}

// Расчёт температуры MAX30100
static float max30100_readTemp()
{
	uint8_t Buff[2];
	i2cm_read(SlaveID, REG_TEMP_INT, Buff, 2);
	max30100_startTempMeasure();

	float temp = *((int8_t *) &(Buff[0]));
	temp += (((float)(Buff[1] & 0x0F)) / 16);

	LastTemp = temp;
	return temp;
}

// Чтение семплов из MAX30100
static void max30100_readSamples()
{
	tFIFO_PTRs FIFO_PTRs;
	uint8_t samples = 15;

	if (!BufferedReading)
	{
		// Чтение статусов прерываний
		int result = i2cm_read(SlaveID, REG_FIFO_WR_PTR, (uint8_t *)&FIFO_PTRs, 3);
		if (result != ESP_OK)
			return;

		if (FIFO_PTRs.ovf_counter == 15)
		{
			max30100_resetFIFO();
			return;
		}

		samples = (FIFO_PTRs.wr_pointer - FIFO_PTRs.rd_pointer) & (16 - 1);
	}

    if (samples)
    {
    	uint8_t Buff[64];
    	i2cm_read(SlaveID, REG_FIFO_DATA, Buff, samples << 2);

    	for (uint8_t i = 0; i < samples; i++)
    	{
    		tSample newSample;
    		newSample.ir = (Buff[i << 2] << 8) | Buff[(i << 2) + 1];
    		newSample.red = (Buff[(i << 2) + 2] << 8) | Buff[(i << 2) + 3];
    		writeSample(&newSample);
    	}
    }
}

// Обработать данные MAX30100
static void max30100_processData()
{
	// Чтение статусов прерываний
	uint8_t Status;
	int result = i2cm_read(SlaveID, REG_INT_STATUS, &Status, 1);
	if (result != ESP_OK)
		return;

	if (Status & INT_TEMP_RDY)	// Температура оцифрована
		max30100_readTemp();
	if (Status & (INT_A_FULL | INT_HR_RDY | INT_SPO2_RDY))
		max30100_readSamples();
}

//
void max30100read_task(void* arg)
{
	max30100_resetFIFO();

	while (1)
    {
		if (!SkipData)
			max30100_processData();
    }
}
