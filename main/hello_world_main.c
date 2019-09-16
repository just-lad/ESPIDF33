
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "console/console.h"
#include "console/console.c"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_spiram.h"
#include "display/dispcolor.h"
#include "display/dispcolor.c"
#include "display/rgbcolor.h"
#include "display/fonts/font.h"
#include "display/fonts/font.c"
#include "display/fonts/f6x8m.h"
#include "display/fonts/f16f.h"
#include "display/fonts/f24f.h"
#include "display/fonts/f32f.h"
#include "display/fonts/f6x8m.c"
#include "display/fonts/f16f.c"
#include "display/fonts/f24f.c"
#include "display/fonts/f32f.c"
#include "max30100/max30100.h"
#include "max30100/max30100.c"
#include "max30100/i2cm.h"
#include "max30100/i2cm.c"
#include "max30100/calc_HR_SpO2.h"
#include "max30100/calc_HR_SpO2.c"
#include "ili9341/ili9341.h"
#include "ili9341/ili9341.c"
#include "esp_spiram.h"
#include "spiram_psram.h"
#include "esp_himem.h"





#define SW_VERSION_MAJOR	1
#define SW_VERSION_MINOR	0


// Разрешение экрана
#define dispWidth 				320
#define dispHeight				240


#define PIN_BUTTON1 	39
#define PIN_BUTTON2 	37
#define PIN_BUTTON3		38

#define BUFF_SIZE		dispWidth

struct sButtonStates
{
	uint8_t	button1 	:1;
	uint8_t	button2 	:1;
	uint8_t	button3 	:1;
	uint8_t	button1_old :1;
	uint8_t	button2_old :1;
	uint8_t	button3_old :1;
};

typedef enum Mode
{
	SelectMode = 0,
	SelectRate = 1,
	SelectLedCurrent = 2
} eMode;
#define ButtonsModeNum		3
eMode ButtonsMode = SelectMode;

// Переключение тока светодиодов
#define LedCurrentNum		16
const float LedCurrents[LedCurrentNum] = {0.0, 4.4, 7.6, 11.0, 14.2, 17.4, 20.8, 24.0, 27.1, 30.6, 33.8, 37.0, 40.2, 43.6, 46.8, 50.0};
uint8_t CurLedCurrent = 7;	//24

// Переключение частоты семплирования
#define SampleRatesNum		8
uint8_t CurSampleRateIdx = 3;	// rate200hz
const eSampleRate SampleRates[SampleRatesNum] = {rate50hz, rate100hz, rate167hz, rate200hz, rate400hz, rate600hz, rate800hz, rate1000hz};

// Переключение режимов отображения
#define MODE_MAX		7
uint8_t CurrentMode = 0;


void buttons_init()
{
	// Инициализация ножки, подключенной к кнопке 1
	gpio_set_direction(PIN_BUTTON1, GPIO_MODE_INPUT);
	gpio_pullup_en(PIN_BUTTON1);
	// Инициализация ножки, подключенной к кнопке 2
	gpio_set_direction(PIN_BUTTON2, GPIO_MODE_INPUT);
	gpio_pullup_en(PIN_BUTTON2);
	// Инициализация ножки, подключенной к кнопке 3
	gpio_set_direction(PIN_BUTTON3, GPIO_MODE_INPUT);
	gpio_pullup_en(PIN_BUTTON3);
}

void RenderPlot(uint8_t mode)
{
	uint16_t samples = BUFF_SIZE;
	uint8_t fingerPresent = ir_min > 30000 ? 1 : 0;
	uint8_t syncStart = 0;
	uint16_t *buff1;
	uint16_t *buff2;
	uint16_t *buff3;
	uint16_t *buff4;
	uint16_t *buff5;

	switch (mode)
	{
	case 0:		// Исходные кривые
		buff1 = readInputIrSamples(samples, syncStart);
		buff2 = readInputRedSamples(samples, syncStart);

        for (int i = 0; i < samples - 1; i++)
    	{
    		dispcolor_DrawLine(i, 114 - buff1[i]/574, i + 1, 114 - buff1[i + 1]/574, RGB565(255, 190, 190));
    		dispcolor_DrawLine(i, 114 * 2 - buff2[i]/574, i + 1, 114 * 2 - buff2[i + 1]/574, RED);
    	}
		break;
	case 1:		// Кривые после нормализации
		buff1 = readStage1IrSamples(samples, syncStart);
		buff2 = readStage1RedSamples(samples, syncStart);

        for (int i = 0; i < samples - 1; i++)
    	{
    		dispcolor_DrawLine(i, 114 - buff1[i]/574, i + 1, 114 - buff1[i + 1]/574, RGB565(255, 190, 190));
    		dispcolor_DrawLine(i, 114 * 2 - buff2[i]/574, i + 1, 114 * 2 - buff2[i + 1]/574, RED);
    	}
		break;
	case 2:		// Кривые после фильтрации
    	buff1 = readStage1IrSamples(samples, syncStart);
    	buff2 = readStage2IrSamples(samples, syncStart);
    	buff3 = readStage1RedSamples(samples, syncStart);
    	buff4 = readStage2RedSamples(samples, syncStart);

        for (int i = 0; i < samples - 1; i++)
    	{
    		dispcolor_DrawLine(i, 114 - buff1[i]/574, i + 1, 114 - buff1[i + 1]/574, RGB565(110, 60, 60));
    		dispcolor_DrawLine(i, 114 - buff2[i]/574, i + 1, 114 - buff2[i + 1]/574, RGB565(255, 190, 190));

    		dispcolor_DrawLine(i, 114 * 2 - buff3[i]/574, i + 1, 114 * 2 - buff3[i + 1]/574, RGB565(100, 0, 0));
    		dispcolor_DrawLine(i, 114 * 2 - buff4[i]/574, i + 1, 114 * 2 - buff4[i + 1]/574, RED);
    	}
		break;
	case 3:		// Вывод кривой приращений
    	buff1 = readStage1IrSamples(samples, syncStart);
    	buff2 = readStage2IrSamples(samples, syncStart);
    	buff3 = readStage3IrSamples(samples, syncStart);
    	buff4 = readStage1RedSamples(samples, syncStart);
    	buff5 = readStage2RedSamples(samples, syncStart);

        for (int i = 0; i < samples - 1; i++)
    	{
    		dispcolor_DrawLine(i, 114 - buff1[i]/574, i + 1, 114 - buff1[i + 1]/574, RGB565(130, 90, 90));
    		dispcolor_DrawLine(i, 114 - buff2[i]/574, i + 1, 114 - buff2[i + 1]/574, RGB565(255, 190, 190));

    		dispcolor_DrawLine(i, 114 * 2 - buff4[i]/574, i + 1, 114 * 2 - buff4[i + 1]/574, RGB565(130, 0, 0));
    		dispcolor_DrawLine(i, 114 * 2 - buff5[i]/574, i + 1, 114 * 2 - buff5[i + 1]/574, RED);

   	   		// Исходная кривая приращений
   	   		dispcolor_DrawLine(i, 114 - buff3[i]/574, i + 1, 114 - buff3[i + 1]/574, BLUE);
    	}
		break;
	case 4:
    	syncStart = 1;
    	buff1 = readStage1IrSamples(samples, syncStart);
    	buff2 = readStage2IrSamples(samples, syncStart);
    	buff3 = readStage3IrSamples(samples, syncStart);
    	buff4 = readStage1RedSamples(samples, syncStart);
    	buff5 = readStage2RedSamples(samples, syncStart);

        for (int i = 0; i < samples - 1; i++)
    	{
        	if (fingerPresent)
        	{
        		// Исхдная кривая дифференциала
        		dispcolor_DrawLine(i, 114 - buff3[i]/574, i + 1, 114 - buff3[i + 1]/574, BLUE);
        	}

    		dispcolor_DrawLine(i, 114 - buff1[i]/574, i + 1, 114 - buff1[i + 1]/574, RGB565(130, 90, 90));
    		dispcolor_DrawLine(i, 114 - buff2[i]/574, i + 1, 114 - buff2[i + 1]/574, RGB565(255, 190, 190));

    		dispcolor_DrawLine(i, 114 * 2 - buff4[i]/574, i + 1, 114 * 2 - buff4[i + 1]/574, RGB565(130, 0, 0));
    		dispcolor_DrawLine(i, 114 * 2 - buff5[i]/574, i + 1, 114 * 2 - buff5[i + 1]/574, RED);
    	}
		break;
	case 5:
    	syncStart = 1;
    	buff1 = readStage1IrSamples(samples, syncStart);
    	buff2 = readStage2IrSamples(samples, syncStart);
    	buff3 = readStage3IrSamples(samples, syncStart);
    	buff4 = readStage1RedSamples(samples, syncStart);
    	buff5 = readStage2RedSamples(samples, syncStart);

        for (int i = 0; i < samples - 1; i++)
    	{
        	if (fingerPresent)
        	{
        		// Исхдная кривая дифференциала
        		dispcolor_DrawLine(i, 114 - buff3[i]/574, i + 1, 114 - buff3[i + 1]/574, BLUE);
        	}

    		dispcolor_DrawLine(i, 114 - buff1[i]/574, i + 1, 114 - buff1[i + 1]/574, RGB565(130, 90, 90));
    		dispcolor_DrawLine(i, 114 - buff2[i]/574, i + 1, 114 - buff2[i + 1]/574, RGB565(255, 190, 190));

    		dispcolor_DrawLine(i, 114 * 2 - buff4[i]/574, i + 1, 114 * 2 - buff4[i + 1]/574, RGB565(130, 0, 0));
    		dispcolor_DrawLine(i, 114 * 2 - buff5[i]/574, i + 1, 114 * 2 - buff5[i + 1]/574, RED);
    	}
		break;
	case 6:
    	syncStart = 1;
    	buff1 = readStage1IrSamples(samples, syncStart);
    	buff2 = readStage2IrSamples(samples, syncStart);
    	buff3 = readStage3IrSamples(samples, syncStart);
    	buff4 = readStage1RedSamples(samples, syncStart);
    	buff5 = readStage2RedSamples(samples, syncStart);

        for (int i = 0; i < samples - 1; i++)
    	{
        	if (fingerPresent)
        	{
        		if (buff3[i] == MARK_VALUE)
        			dispcolor_DrawLine(i, 0, i, 232, BLUE);
        	}

    		dispcolor_DrawLine(i, 114 - buff1[i]/574, i + 1, 114 - buff1[i + 1]/574, RGB565(130, 90, 90));
    		dispcolor_DrawLine(i, 114 - buff2[i]/574, i + 1, 114 - buff2[i + 1]/574, RGB565(255, 190, 190));

    		dispcolor_DrawLine(i, 114 * 2 - buff4[i]/574, i + 1, 114 * 2 - buff4[i + 1]/574, RGB565(130, 0, 0));
    		dispcolor_DrawLine(i, 114 * 2 - buff5[i]/574, i + 1, 114 * 2 - buff5[i + 1]/574, RED);
    	}
		break;
	case 7:
    	syncStart = 1;
    	buff2 = readStage2IrSamples(samples, syncStart);
    	buff3 = readStage3IrSamples(samples, syncStart);
    	buff5 = readStage2RedSamples(samples, syncStart);

        for (int i = 0; i < samples - 1; i++)
    	{
        	if (fingerPresent)
        	{
           		if (buff3[i] == MARK_VALUE)
        			dispcolor_DrawLine(i, 0, i, 232, BLUE);
        	}

    		dispcolor_DrawLine(i, 114 - buff2[i]/574, i + 1, 114 - buff2[i + 1]/574, RGB565(255, 190, 190));
    		dispcolor_DrawLine(i, 114 * 2 - buff5[i]/574, i + 1, 114 * 2 - buff5[i + 1]/574, RED);
    	}
		break;
	}
}

void RenderResults(uint8_t mode)
{
	if (mode < 4)
		return;

	float heartRate = getHeartRate();

    if ((heartRate == (float)0) || (!getFingerPresent()))
    {
    	uint16_t x = dispcolor_printf(0, 0, FONTID_32F, WHITE, "--");
    	dispcolor_printf(0, 32, FONTID_32F, WHITE, "--");
        dispcolor_printf(x + 3, 18, FONTID_6X8M, WHITE, "BPM");
    	dispcolor_printf(x + 3, 50, FONTID_6X8M, WHITE, "%%");
    }
    else
    {
    	uint16_t x = dispcolor_printf(0, 0, FONTID_32F, WHITE, "%d", (int)heartRate);
        dispcolor_printf(x + 3, 18, FONTID_6X8M, WHITE, "BPM");
    	x = dispcolor_printf(0, 32, FONTID_32F, WHITE, "%d", (int)getSpO2());
    	dispcolor_printf(x + 3, 50, FONTID_6X8M, WHITE, "%%");

    	dispcolor_printf(200, 232, FONTID_6X8M, WHITE, "R:%.2f\n", getR());
    }
}

//
void app_main()
{
	int result;
	struct sButtonStates ButtonStates;

	// Инициализация кнопок
	buttons_init();

	// Инициализация дисплея
    printf("Display init\n");
    dispcolor_Init(dispWidth, dispHeight);
    dispcolor_SetBrightness(100);

    // Вывод общей информации о CPU
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    console_printf(MsgInfo, "ESP32 rev. %d (%d CPU cores, WiFi%s%s), ", chip_info.revision, chip_info.cores, (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    console_printf(MsgInfo, " %d MB %s SPI FLASH\n", spi_flash_get_chip_size() / (1024 * 1024), (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    console_printf(MsgInfo, " %d MB SPI PSRAM\n", 8); //esp_spiram_get_size() / (1024 * 1024) this was instead of '8' but it workes only like this
    console_pause(300);

    // Инициализация MAX30100
	console_printf(MsgInfo, "Инициализация MAX30100\n");
    result = max30100_init(1, SampleRates[CurSampleRateIdx], 1);
    if (result == -10000)
    	FatalErrorMsg("Ошибка выделения ОЗУ для буферов MAX30100\n");
    else if (result == -10001)
    	FatalErrorMsg("Ошибка запуска задачи чтения данных из MAX30100\n");
    else if (result != ESP_OK)
    	FatalErrorMsg("Ошибка инициализации MAX30100\n");

    while (1)
    {
    	ButtonStates.button1 = gpio_get_level(PIN_BUTTON1);
    	ButtonStates.button2 = gpio_get_level(PIN_BUTTON2);
    	ButtonStates.button3 = gpio_get_level(PIN_BUTTON3);

    	// Нажате кнопки 1
    	if ((!ButtonStates.button1) && (ButtonStates.button1_old))
    	{
    		switch (ButtonsMode)
    		{
    		case SelectMode:
    			if (CurrentMode < MODE_MAX)
    				CurrentMode++;
    			break;
    		case SelectRate:
    			if (CurSampleRateIdx < (SampleRatesNum - 1))
    			{
    				CurSampleRateIdx++;
    				max30100_setSampleRate(1, SampleRates[CurSampleRateIdx]);
    			}
    			break;
    		case SelectLedCurrent:
    			if (CurLedCurrent < (LedCurrentNum - 1))
    			{
    				CurLedCurrent++;
    				max30100_setLedcurrent(CurLedCurrent, CurLedCurrent);
    			}
    			break;
    		}
    	}

    	// Нажате кнопки 2
    	if ((!ButtonStates.button2) && (ButtonStates.button2_old))
    	{
    		if (++ButtonsMode == ButtonsModeNum)
    			ButtonsMode = 0;
    	}

    	// Нажате кнопки 3
    	if ((!ButtonStates.button3) && (ButtonStates.button3_old))
    	{
    		switch (ButtonsMode)
    		{
    		case SelectMode:
    			if (CurrentMode)
    				CurrentMode--;
    			break;
    		case SelectRate:
    			if (CurSampleRateIdx)
    			{
    				CurSampleRateIdx--;
    				max30100_setSampleRate(1, SampleRates[CurSampleRateIdx]);
    			}
    			break;
    		case SelectLedCurrent:
    			if (CurLedCurrent)
    			{
    				CurLedCurrent--;
    				max30100_setLedcurrent(CurLedCurrent, CurLedCurrent);
    			}
    			break;
    		}
    	}

		ButtonStates.button1_old = ButtonStates.button1;
		ButtonStates.button2_old = ButtonStates.button2;
		ButtonStates.button3_old = ButtonStates.button3;


		// Очистка буфера экрана
        dispcolor_FillRect(0, 0, 320, 240, BLACK);

        // Обсчёт кривых
    	calculate(CurrentMode);
    	// Отрисовка графиков
    	RenderPlot(CurrentMode);
        // Вывод измеренных значений в углу
    	RenderResults(CurrentMode);

    	// Вывод параметров в статусную строку
		switch (ButtonsMode)
		{
		case SelectMode:
        	dispcolor_printf(0, 232, FONTID_6X8M, WHITE, "MODE %d", CurrentMode);
			break;
		case SelectRate:
        	dispcolor_printf(0, 232, FONTID_6X8M, WHITE, "RATE %d", SampleRates[CurSampleRateIdx]);
			break;
		case SelectLedCurrent:
        	dispcolor_printf(0, 232, FONTID_6X8M, WHITE, "LEDP %.1f", LedCurrents[CurLedCurrent]);
			break;
		}

    	dispcolor_printf(60, 232, FONTID_6X8M, WHITE, "MIN:%d\n", ir_min);
    	dispcolor_printf(130, 232, FONTID_6X8M, WHITE, "MAX:%d\n", ir_max);
    	dispcolor_printf(260, 232, FONTID_6X8M, WHITE, "TEMP=%.1f", max30100_getTemp());

    	// Обновляем дисплей из буфера кадра
    	dispcolor_Update();

        vTaskDelay(20 / portTICK_RATE_MS);
    }
}
