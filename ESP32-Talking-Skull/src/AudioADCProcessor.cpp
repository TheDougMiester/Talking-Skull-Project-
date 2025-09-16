
/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <Arduino.h>
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "mesh/access.h"
#include "portmacro.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"


#include "freertos/semphr.h"
#include "esp_random.h"
#include <sys/types.h>
#include <math.h>

#include "esp_adc/adc_continuous.h"
#include "esp_dsp.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "AudioADCProcessor.h"

int32_t abs_int32_t(int32_t arg){      
	return arg>=0 ? arg: -arg;                      
}  
//#define _EXAMPLE_ADC_UNIT_STR(unit) #unit
//#define EXAMPLE_ADC_UNIT_STR(unit) _EXAMPLE_ADC_UNIT_STR(unit)
#define ADC_ATTEN ADC_ATTEN_DB_12 //ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data) ((p_data)->type1.data)
#else
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define ADC_GET_CHANNEL(p_data) ((p_data)->type2.channel)
#define ADC_GET_DATA(p_data) ((uint32_t)((p_data)->type2.data))
#endif
 
#define TAG "ADC_AUDIO_READ"
 
 
 #define MAKE_EVEN_ROUND_DOWN(x) ((x) & ~1)  //example 0111 is 7, becomes 0110, that is, 6
 #define MAKE_EVEN_ROUND_UP(x) (((x)+1) & ~1)  //example 0111 is 7, becomes 1110, that is, 8
									   //2nd example: 8, or 1110 rounds to 1111 (9), which 
										//becomes 1110 (8 again)

 // The range of human voice is about 70-1100 Hz. However:
 // Frequencies below 400 Hz are not typically associated with jaw movement. Most are “plosives” 
 // associated with unvoiced sounds such as “t,” “k,” and “p,” and when recorded, are actually 
 //amplified as an artifact of the recording process (i.e., air expelled during speech that 
 //impacts the microphone element).

 // Also, there are frequencies produced by the human mouth that go well above the range of vocal
 // chords. Frequencies above 2.5 kHz are typically sibilant sounds such as “ssss”/“shh". 
 // The human mouth normally produces these by bringing the teeth and jaw together. 
 // Therefore, frequencies we should sample for jaw motion are between 400 Hz and and 2.5 kHz.										

 

#define ADC_READ_CHECK(x)                                   \
	 do {                                                    \
		 esp_err_t err_rc_ = (x);                            \
		 if (err_rc_ == ESP_ERR_INVALID_STATE) {             \
			 ESP_LOGW(TAG, "ADC internal buf full.");        \
		 } else if (err_rc_ == ESP_ERR_TIMEOUT) {            \
			 ESP_LOGW(TAG, "ADC reports NO data available"); \
		 }                                                   \
	 } while (0)


#define _ADC_TO_GPIO(x) x ## _GPIO_NUM
#define ADC_TO_GPIO(x) _ADC_TO_GPIO(x)
#define _GPIO_TO_ADC1(x) ADC1_GPIO ## x ##_CHANNEL
#define GPIO_TO_ADC1(x) _GPIO_TO_ADC1(x)
#define _GPIO_TO_ADC2(x) ADC2_GPIO ## x ##_CHANNEL
#define GPIO_TO_ADC2(x) _GPIO_TO_ADC2(x)

//this isn't quite right yet
	 // Use some convoluted macro-magic to find the corresponding ADC for a specified GPIO
	 // Because ADCx_CHANNELx is n Enum, we can't use macro checks to verify which mapping is correct
	 //  Instead:
	 //    (1)  map GPIO->ADCx_GPIOy_CHANNEL : This will either result in an enum or an undefined macro
	 //    (2)  map (1) -> ADCx_CHANNEL_y_GPIO_NUM :
	 //         If (1) mapped to an enum, this will map back to a GPIO #
	 //         If (1) did NOT map to an enum, this will map to an empty value
	 //    (3)  compare (2) to the original GPIO which will only succeed if (1) mapped properly
	 #if (ADC_TO_GPIO(GPIO_TO_ADC1(CONFIG_BUTTON_INTR_PIN)) == CONFIG_BUTTON_INTR_PIN)
	 #endif
	 #if (ADC_TO_GPIO(GPIO_TO_ADC1(CONFIG_GPIO_PIN)) == CONFIG_GPIO_PIN)
		 #define CONFIG_ADC_CHAN GPIO_TO_ADC1(CONFIG_GPIO_PIN)
	 #elif (ADC_TO_GPIO(GPIO_TO_ADC2(CONFIG_GPIO_PIN)) == CONFIG_GPIO_PIN)
		 #define CONFIG_ADC_CHAN GPIO_TO_ADC2(CONFIG_GPIO_PIN)
	 #else
		 #error "No ADC found for CONFIG_GPIO_PIN" 
		 #define CONFIG_BUTTON_ADC_CHAN 0
	 #endif



void AudioADCProcessor::continuous_adc_init() {	

	AudioADCProcessor::adc_handle = NULL;

     // Initialize ADC in continuous mode (singe shot is wayyyy to slow)
	adc_continuous_handle_cfg_t adc_config = {
		.max_store_buf_size = ADC_MAX_STORE_BUF_SZ,
		.conv_frame_size = ADC_CONV_FRAME_SZ,
	};

	ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &AudioADCProcessor::adc_handle));

	adc_digi_pattern_config_t adc_pattern = {
		.atten = ADC_ATTEN,
		.channel = AudioADCProcessor::adc_channel,
		.unit = AudioADCProcessor::adc_unit,
		.bit_width = ADC_BIT_WIDTH,
	};

	ESP_LOGI(TAG, "adc_pattern{.atten= %u , .channel= %u, .unit= %u .bit_width=%u}", 
		adc_pattern.atten, adc_pattern.channel, adc_pattern.unit, adc_pattern.bit_width);

	adc_continuous_config_t dig_cfg = {
		.pattern_num = 1,
		.adc_pattern = &adc_pattern,
		.sample_freq_hz = ADC_SAMPLE_RATE,
		.conv_mode = ADC_CONV_SINGLE_UNIT_1,
		.format = ADC_OUTPUT_TYPE,
	};

	ESP_ERROR_CHECK(adc_continuous_config(AudioADCProcessor::adc_handle, &dig_cfg));

}

//exponential moving avereage EMA_BW -
// the lower the number, the better the response, but more noise
#define EMA_BW 4
#define DEBUG_AUDIO_FILTER
void AudioADCProcessor::readAndProcessAdc (void *parameters) {
	uint8_t adc_buf[ADC_MAX_STORE_BUF_SZ];
	uint32_t ema_filtered_data[ADC_MAX_STORE_BUF_SZ];

	uint32_t rxLen = 0;
	uint32_t ret_num = 0, tot_read_num = 0;
	uint32_t filtered_data_abs; //absolute value of filtered data
	int32_t ema_filter = 0; //envelope detector - removes jitter from ADC read
	adc_digi_output_data_t *p;
	uint16_t pad = 7;  // 2^7 = 128 //this will amplify the input to decrease loss of LSBs
	int32_t data; // must be a signed int
	ESP_ERROR_CHECK(adc_continuous_start(adc_handle));


	for (;;) {
	//int32_t max  = -10000;
	//int32_t min = 10000;
		//can remove once the adc_stop is removed
		//ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

		memset(ema_filtered_data, 0, sizeof(ema_filtered_data));

		// read ADC and fill the buffer
		tot_read_num = 0;
		//int64_t previous_us{esp_timer_get_time()};
	   //adc_continuous_read(adc_handle, adc_buf, ADC_CONV_FRAME_SZ, &ret_num, 0);
	
	   do {
			//ulTaskNotifyTake(pdTRUE, portMAX_DELAY);	
			ADC_READ_CHECK(adc_continuous_read(
				AudioADCProcessor::adc_handle, adc_buf + tot_read_num,
				ADC_CONV_FRAME_SZ * SOC_ADC_DIGI_RESULT_BYTES - tot_read_num, &ret_num,
				pdMS_TO_TICKS(100)));
			tot_read_num += ret_num;
		} while (tot_read_num < ADC_CONV_FRAME_SZ * SOC_ADC_DIGI_RESULT_BYTES);

		//previous_us = esp_timer_get_time();
		// run the data thru the filter, apply a moving average, then export
		// the data to the array the jaw will use.
		for (int i=0, j=0; i<ADC_MAX_STORE_BUF_SZ; j++, i+=SOC_ADC_DIGI_RESULT_BYTES)
		//for (int i=0, j= 0;  i<ret_num; j++, i+=SOC_ADC_DIGI_RESULT_BYTES)
		{
			p = (adc_digi_output_data_t *)&adc_buf[i]; //range = 0-4096 (3.3v)
			data = (int32_t)(ADC_GET_DATA(p)) - 2048; //normalize the data to -2048 > data < +2048 (from 0-4096)
			filtered_data_abs = applyfilter(data<<pad); // multiply by pad (amplify to decrease loss of LSBs)
			filtered_data_abs =  filtered_data_abs >>pad; //divide by pad (take the pad back out)
			ema_filter -= ema_filter >> EMA_BW; // moving average.
    		ema_filter += filtered_data_abs; // finish moving avg.
			ema_filtered_data[j] = ema_filter >> EMA_BW; //yes, bitshift by EMA_BW a second time. Has to be divided by (2^EMA_BW) to use
			//if (max < (int32_t)ema_filtered_data[j]){max = ema_filtered_data[j] ;} else if (ema_filtered_data[j] < min) {min = ema_filtered_data[j];}
			//Serial.printf("adc %d\t data: %d \tfiltered EMA'd data %d \t abs data %d\n", ADC_GET_DATA(p), data, ema_filtered_data[j], filtered_data_abs );
		}
		
		xSemaphoreTake(AudioADCProcessor::adcMutex, portMAX_DELAY);
		memcpy(AudioADCProcessor::processedAdcData, ema_filtered_data, sizeof(AudioADCProcessor::processedAdcData));

		xSemaphoreGive(AudioADCProcessor::adcMutex);
		//int64_t final_us{esp_timer_get_time()};

 		//Serial.printf("max %d \t min %d\n", max, min);
		
	// Stop ADC and read till empty (might print error message that ADC is
    // already stopped since print before takes long)
    //adc_continuous_stop(AudioADCProcessor::adc_handle);
    //while (adc_continuous_read(AudioADCProcessor::adc_handle, adc_buf, ADC_MAX_STORE_BUF_SZ, &ret_num,	0) == ESP_OK)
    //;

    // Restart ADC
    //previous_us = esp_timer_get_time();
    //ESP_ERROR_CHECK(adc_continuous_start(AudioADCProcessor::adc_handle));
	//	vTaskDelay(pdMS_TO_TICKS(1000));
	}
}


/* code below optimized, but originally built using http://jaggedplanet.com/iir/iir-explorer.asp
*  with the following parameters:
*  Type: Elliptical
*  Form: Bandpass
*  Order: 4
*  Samplerate: 22100
*  Cutoff: 500(hz), used to be 1200 (hz), but I had better results at 500
*  Ripple: .1  (if you change the filter type to elliptical)
*  SB-Ripple: -40 (if you change the filter type to elliptical)
*  Width 2000
*
*  Optimization strategy - Converted the output code into int32_t as follows:
*  biquada[] and biquadb[] are the biquads produced by the jaggedplanet website, then multiplied by 2^12 in setup_coeff()
*  (so we can use int32_t instead of double) the indicies[] array was implemented to use 
*  pointer arithmatic rather than adding integers (I wouldn't have thought of that on my own. Thanks Torai)
*  It also turns out the ESP32 compiler is kind of...dated; it optimizes for-loops inefficiently. 
*  So, I removed all of them from this routine. By optimising this way, I improved performance 
*  of the original code by more than an order of magnitude even before I put it into 
*  IRAM_ATTR, which got me another 4x speed improvement.
*/

#define REAL int32_t
#define NBQ 4
#define SCALE 12
#define FACT (1 << SCALE)
#define BIQUAD_SIZE NBQ*2
// the below are ranges were used for elliptical filter, with cuttoff at 1200hz. 
double biquada[]={0.9734775933565408,-1.9354466102042138,0.8720439296852008,-1.807585345497667,0.7793830752725933,-1.5687958438026732,0.9201456190502246,-1.5294908736295505};
double biquadb[]={0.9999999999999999,-1.98008833600583,0.9999999999999999,-1.9947985872439253,0.9999999999999999,-0.16661107833700373,0.9999999999999999,-1.2780586056483751};

REAL biquada_f[BIQUAD_SIZE];
REAL biquadb_f[BIQUAD_SIZE];

REAL adc_gain = 1;

int16_t counter = 0;

void AudioADCProcessor::setup_coeff(){
	int16_t i = 0;
	
	for(i=0; i<BIQUAD_SIZE; i++){
		biquada_f[i] = round(biquada[i]*FACT);
		biquadb_f[i] = round(biquadb[i]*FACT);
	}
	adc_gain = 64; // enter by hand, TODO change to bit shifts
}

int indices[] = {0, 2, 5, 1, 4, 3, 3, 5, 8, 4, 7, 6, 6, 8, 11, 7, 10, 9, 9, 11, 14, 10, 13, 12,0}; /* last one is a dummy */

// the number of array elements must be NBQ*3+3,
REAL z[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
void AudioADCProcessor::resetfilter() {
  memset( z, 0, sizeof(z));
  setup_coeff();
  return;
}

REAL IRAM_ATTR AudioADCProcessor::applyfilter(REAL v)
{

	int* idx = (int *) indices;
	REAL* a_idx = (REAL *) biquada_f;
	REAL* b_idx = (REAL *) biquadb_f;

	static REAL out;
	out=v/adc_gain;
	/* replaced for (int i=NBQ*3+2; i>0; i--) {z[i]=z[i-1];}  
    * the number of loops must be NBQ*3+2, (one less than the number of elements)
	* but the ESP32 compiler is very poor with for loops, so unroll for-loop.
	*/
	z[14] = z[13];
    z[13] = z[12];
	z[12] = z[11];
	z[11] = z[10];
	z[10] = z[9];
	z[9] = z[8];
	z[8] = z[7];
	z[7] = z[6];
	z[6] = z[5];
	z[5] = z[4];
	z[4] = z[3];
	z[3] = z[2];
	z[2] = z[1];
	z[1] = z[0];

	/* we have to unroll the for loop, but here's what it would look like:
	*for (int i=0; i<NBQ; i++)
	*{
	*	z[*idx++]=out;
	*	out+=*(z + *(idx++))* (*b_idx++) >> SCALE;
	*	out-=*(z + *(idx++))* (*a_idx++) >> SCALE;
	*	out+=*(z + *(idx++))* (*b_idx++) >> SCALE;
	*	out-=*(z + *(idx++))* (*a_idx++) >> SCALE;
	*	z[*idx++]=out;
	*}
	*/
	//I could have squeezed a few more usecs out
	// by putting integers in rather than the pointers to indicies 
	// Torai thought up, but at this point, it's fast enough.
	z[*idx++]=out;
	out+=*(z + *(idx++))* (*b_idx++) >> SCALE;
	out-=*(z + *(idx++))* (*a_idx++) >> SCALE;
	out+=*(z + *(idx++))* (*b_idx++) >> SCALE;
	out-=*(z + *(idx++))* (*a_idx++) >> SCALE;
	z[*idx++]=out;

	z[*idx++]=out;
	out+=*(z + *(idx++))* (*b_idx++) >> SCALE;
	out-=*(z + *(idx++))* (*a_idx++) >> SCALE;
	out+=*(z + *(idx++))* (*b_idx++) >> SCALE;
	out-=*(z + *(idx++))* (*a_idx++) >> SCALE;
	z[*idx++]=out;

	z[*idx++]=out;
	out+=*(z + *(idx++))* (*b_idx++) >> SCALE;
	out-=*(z + *(idx++))* (*a_idx++) >> SCALE;
	out+=*(z + *(idx++))* (*b_idx++) >> SCALE;
	out-=*(z + *(idx++))* (*a_idx++) >> SCALE;
	z[*idx++]=out;
	
    //negative numbers are not meaningful. Just return the magnitude
    return abs_int32_t(out);
}

void AudioADCProcessor::begin () {
	setup_coeff();
	continuous_adc_init();
	  // Create a max priority task on core 1
	//  xTaskCreatePinnedToCore(readAndProcessAdc, "readAndProcessAdc", 4096, NULL, 11, NULL,1);
	//xTaskCreate(readAndProcessAdc, "readAndProcessAdc", 8192, NULL, 4, NULL);
}

void AudioADCProcessor::initialize(adc_unit_t p_adcUnit, adc_channel_t p_adcChannel, xSemaphoreHandle *xMutexAdcSound) {
	AudioADCProcessor::adc_unit = p_adcUnit;
	AudioADCProcessor::adc_channel = p_adcChannel;
	AudioADCProcessor::adcMutex = *xMutexAdcSound;
}

adc_unit_t AudioADCProcessor::adc_unit;
adc_channel_t AudioADCProcessor::adc_channel;
adc_continuous_handle_t AudioADCProcessor::adc_handle;
uint32_t AudioADCProcessor::rawValue;
uint32_t AudioADCProcessor::filteredVal;
TaskHandle_t AudioADCProcessor::s_task_handle;
xSemaphoreHandle AudioADCProcessor::adcMutex = NULL;
uint32_t AudioADCProcessor::processedAdcData[ADC_CONV_FRAME_SZ];

