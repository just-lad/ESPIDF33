#include "calc_HR_SpO2.h"
#include "max30100.h"
#include <esp_err.h>
#include "esp_heap_caps.h"
#include <string.h>
#include <math.h>


uint16_t ir_max, ir_min;
static uint16_t red_max, red_min;
static uint16_t SyncIdx = 0;

static uint16_t SampleCount = 0;
static uint16_t PrevSample = 0;
static uint16_t BuffSize;

static float R;
static float SpO2;
static float HeartRate;


// Указатели на промежуточные буферы для обработки принятых данных
//#define DATA_PROCESS_RAM_TYPE		(MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM)
#define DATA_PROCESS_RAM_TYPE		(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL)
static uint16_t *ir_input_buff;
static uint16_t *ir_stage1_buff;
static uint16_t *ir_stage2_buff;
static uint16_t *ir_stage3_buff;
static uint16_t *red_input_buff;
static uint16_t *red_stage1_buff;
static uint16_t *red_stage2_buff;


int16_t allocMemory(uint16_t buffSize)
{
	BuffSize = buffSize;
	ir_input_buff = heap_caps_malloc(BuffSize << 1, DATA_PROCESS_RAM_TYPE);
	ir_stage1_buff = heap_caps_malloc(BuffSize << 1, DATA_PROCESS_RAM_TYPE);
	ir_stage2_buff = heap_caps_malloc(BuffSize << 1, DATA_PROCESS_RAM_TYPE);
	ir_stage3_buff = heap_caps_malloc(BuffSize << 1, DATA_PROCESS_RAM_TYPE);
	red_input_buff = heap_caps_malloc(BuffSize << 1, DATA_PROCESS_RAM_TYPE);
	red_stage1_buff = heap_caps_malloc(BuffSize << 1, DATA_PROCESS_RAM_TYPE);
	red_stage2_buff = heap_caps_malloc(BuffSize << 1, DATA_PROCESS_RAM_TYPE);

	if ((!ir_input_buff) || (!ir_stage1_buff) || (!ir_stage2_buff) || (!ir_stage3_buff) || (!red_input_buff) || (!red_stage1_buff) || (!red_stage2_buff))
		return -1;
	return 0;
}

void freeMemory()
{
	if (ir_input_buff)
		heap_caps_free(ir_input_buff);
	if (ir_stage1_buff)
		heap_caps_free(ir_stage1_buff);
	if (ir_stage2_buff)
		heap_caps_free(ir_stage2_buff);
	if (ir_stage3_buff)
		heap_caps_free(ir_stage3_buff);
	if (red_input_buff)
		heap_caps_free(red_input_buff);
	if (red_stage1_buff)
		heap_caps_free(red_stage1_buff);
	if (red_stage2_buff)
		heap_caps_free(red_stage2_buff);
}

//
uint16_t max30100_calcMaxMinIdx(uint16_t *buff, uint16_t idx1, uint16_t idx2, uint16_t *pmax, uint16_t *pmin)
{
	*pmax = *pmin = *buff;
	uint16_t idx = idx1;
	while (idx <= idx2)
	{
		if (buff[idx] < *pmin)
			*pmin = buff[idx];
		if (buff[idx] > *pmax)
			*pmax = buff[idx];
		idx++;
	}

	return *pmax - *pmin;
}

//
uint16_t max30100_calcMaxMin(uint16_t *buff, uint16_t samples, uint16_t *pmax, uint16_t *pmin)
{
	max30100_calcMaxMinIdx(buff, 0, samples - 1, pmax, pmin);
	if (*pmax - *pmin < 500)
		*pmax = *pmin + 500;

	return *pmax - *pmin;
}

//
static void normaliseBuff(uint16_t *buffIn, uint16_t *buffOut, uint16_t samples, uint16_t max, uint16_t min)
{
	uint16_t delta = max - min;
	while (samples--)
	{
		uint32_t temp = *buffIn - min;
		temp = temp * 65535 / delta;
		*buffOut = (uint16_t) temp;
		buffIn++;
		buffOut++;
	}
}

//
static void filterBuff(uint16_t *buff_in, uint16_t *buff_out, uint16_t samples, uint8_t window)
{
	if ((window % 2) == 0)
		window++;

	uint8_t hw = (window - 1) / 2;
	buff_out[0] = buff_in[0];

	for (uint16_t i = 1; i < samples; i++)
	{
		uint32_t sum = 0;
		uint16_t k1, k2;
		uint8_t samples_cnt;

		if (i < hw)
	    {
	    	k1 = 0;
	        k2 = 2 * i;
	        samples_cnt = k2 + 1;
	    }
	    else if ((i + hw) > (samples - 1))
	    {
	    	k1 = i - samples + i + 1;
	        k2 = samples - 1;
	        samples_cnt = k2 - k1 + 1;
	    }
	    else
	    {
	    	k1 = i - hw;
	        k2 = i + hw;
	        samples_cnt = window;
	    }

	    for (uint16_t j = k1; j <= k2; j++)
	    	sum += buff_in[j];

	    buff_out[i] = sum / samples_cnt;
	}
}

//
static void calcDiffBuff(uint16_t *buff_in, uint16_t *buff_out, uint16_t samples, uint8_t step, uint16_t offset)
{
	for (uint16_t i = 1; i < samples; i++)
	{
		if (i >= step)
			buff_out[i] = buff_in[i] + offset - buff_in[i - step];
		else
			buff_out[i] = buff_in[i] + offset - buff_in[0];
	}
}

void markExtremums(uint16_t *buff, uint16_t samples)
{
	uint16_t Idx1 = 0, Idx2 = 0;
	uint16_t startIdx = SampleRate / 15;
	PrevSample = buff[startIdx++];
	for (uint16_t i = startIdx; i < samples; i++)
	{
		uint16_t newValue = buff[i];
		if ((newValue > PrevSample) && (newValue - PrevSample > 50000))	// Передний фронт
		{
			Idx1 = i;
		}
		if ((newValue < PrevSample) && (PrevSample - newValue > 50000))	// Задний фронт
		{
			Idx2 = i;

			for (uint16_t j = Idx1; j < (Idx2 - 1); j++)
			{
				if (buff[j + 1] > buff[j])
				{
					buff[j] = MARK_VALUE;
					break;
				}
			}
		}
		PrevSample = newValue;
	}
}

//
static float getBeat(eSampleRate sampleRate, uint16_t *buff, uint16_t samples, uint16_t *pBeatIdx1, uint16_t *pBeatIdx2)
{
	uint8_t firstBeat = 1;
	uint16_t startIdx = SampleRate / 15;
	PrevSample = buff[startIdx++];
	*pBeatIdx1 = *pBeatIdx2 = 0;

	for (uint16_t i = startIdx; i < samples; i++)
	{
		uint16_t newValue = buff[i];
		if ((newValue < PrevSample) && (PrevSample - newValue > 50000))	// Задний фронт
		{

			if (firstBeat)
			{
				firstBeat = 0;
				SampleCount = 0;
				SyncIdx = i;
				*pBeatIdx1 = i;
			}
			else
			{
				if (SampleCount)
				{
					*pBeatIdx2 = i;
					return (60 * sampleRate) / SampleCount;
				}
			}
		}

		SampleCount++;
		PrevSample = newValue;
	}

	SyncIdx = 0;
	return 0;
}

//
static float getBeat2(eSampleRate sampleRate, uint16_t *buff, uint16_t samples, uint16_t *pBeatIdx1, uint16_t *pBeatIdx2)
{
	uint8_t firstBeat = 1;
	uint16_t startIdx = SampleRate / 15;
	*pBeatIdx1 = *pBeatIdx2 = 0;

	for (uint16_t i = startIdx; i < samples; i++)
	{
		if (buff[i] == MARK_VALUE)
		{
			if (firstBeat)
			{
				firstBeat = 0;
				SampleCount = 0;
				SyncIdx = i;
				*pBeatIdx1 = i;
			}
			else
			{
				if (SampleCount)
				{
					*pBeatIdx2 = i;
					return (60 * sampleRate) / SampleCount;
				}
			}
		}

		SampleCount++;
	}

	SyncIdx = 0;
	return 0;
}

// основная функция расчёта HR и SpO2
float calculate(uint8_t mode)
{
	if ((!ir_input_buff) || (!ir_stage1_buff) || (!ir_stage2_buff) || (!ir_stage3_buff) || (!red_input_buff) || (!red_stage1_buff) || (!red_stage2_buff))
		return 0;

	uint16_t samples = BuffSize;
	readLastSamples(ir_input_buff, red_input_buff, samples);

	uint16_t filterWidth = SampleRate / 15;		// 15;
	uint16_t diffStep = SampleRate / 30;//40;		// 5;

	uint16_t ir_delta = max30100_calcMaxMin(ir_input_buff, samples, &ir_max, &ir_min);
	normaliseBuff(ir_input_buff, ir_stage1_buff, samples, ir_max, ir_min);
	filterBuff(ir_stage1_buff, ir_stage2_buff, samples, filterWidth);
	calcDiffBuff(ir_stage2_buff, ir_stage3_buff, samples, diffStep, ir_delta * 3);

    uint16_t red_delta = max30100_calcMaxMin(red_input_buff, samples, &red_max, &red_min);
	normaliseBuff(red_input_buff, red_stage1_buff, samples, red_max, red_min);
    filterBuff(red_stage1_buff, red_stage2_buff, samples, filterWidth);

    // Расчёт частоты сердцебиения
    uint16_t Idx1, Idx2;
	if (mode >= 5)
	{
		markExtremums(ir_stage3_buff, samples);
	    HeartRate = getBeat2(SampleRate, ir_stage3_buff, samples, &Idx1, &Idx2);
	}
	else
	    HeartRate = getBeat(SampleRate, ir_stage3_buff, samples, &Idx1, &Idx2);

    uint16_t ir_min2, ir_max2, red_min2, red_max2;
    ir_delta = max30100_calcMaxMinIdx(ir_input_buff, Idx1, Idx2, &ir_max2, &ir_min2);
    red_delta = max30100_calcMaxMinIdx(red_input_buff, Idx1, Idx2, &red_max2, &red_min2);

    // Расчёт SpO2 по AN6409
    if ((red_min2 != 0.0) && (ir_delta != 0.0))
    {
    	R = red_delta * ir_min2;
    	R /= red_min2;
    	R /= ir_delta;
    	SpO2 = 104 - (17 * R);
    }

    return getHeartRate();
}

float getHeartRate()
{
    if ((HeartRate < 30) || (HeartRate > 250))
    	return (float)0;
    else
    	return HeartRate;
}

float getR()
{
   	return R;
}

float getSpO2()
{
   	return SpO2;
}

uint8_t getFingerPresent()
{
	return ir_min > 30000 ? 1 : 0;
}

uint16_t *readInputIrSamples(uint16_t samples, uint8_t syncStart)
{
	if (samples + SampleRate / 15 > BuffSize)
		return ir_input_buff;

	uint16_t idx = syncStart ? SyncIdx : BuffSize - samples - SampleRate / 15 - 1;
	return &ir_input_buff[idx];
}

uint16_t *readStage1IrSamples(uint16_t samples, uint8_t syncStart)
{
	if (samples + SampleRate / 15 > BuffSize)
		return ir_stage1_buff;

	uint16_t idx = syncStart ? SyncIdx : BuffSize - samples - SampleRate / 15 - 1;
	return &ir_stage1_buff[idx];
}

uint16_t *readStage2IrSamples(uint16_t samples, uint8_t syncStart)
{
	if (samples + SampleRate / 15 > BuffSize)
		return ir_stage2_buff;

	uint16_t idx = syncStart ? SyncIdx : BuffSize - samples - SampleRate / 15 - 1;
	return &ir_stage2_buff[idx];
}

uint16_t *readStage3IrSamples(uint16_t samples, uint8_t syncStart)
{
	if (samples + SampleRate / 15 > BuffSize)
		return ir_stage3_buff;

	uint16_t idx = syncStart ? SyncIdx : BuffSize - samples - SampleRate / 15 - 1;
	return &ir_stage3_buff[idx];
}

uint16_t *readInputRedSamples(uint16_t samples, uint8_t syncStart)
{
	if (samples + SampleRate / 15 > BuffSize)
		return red_input_buff;

	uint16_t idx = syncStart ? SyncIdx : BuffSize - samples - SampleRate / 15 - 1;
	return &red_input_buff[idx];
}

uint16_t *readStage1RedSamples(uint16_t samples, uint8_t syncStart)
{
	if (samples + SampleRate / 15 > BuffSize)
		return red_stage1_buff;

	uint16_t idx = syncStart ? SyncIdx : BuffSize - samples - SampleRate / 15 - 1;
	return &red_stage1_buff[idx];
}

uint16_t *readStage2RedSamples(uint16_t samples, uint8_t syncStart)
{
	if (samples + SampleRate / 15 > BuffSize)
		return red_stage2_buff;

	uint16_t idx = syncStart ? SyncIdx : BuffSize - samples - SampleRate / 15 - 1;
	return &red_stage2_buff[idx];
}
