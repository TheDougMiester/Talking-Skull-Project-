/*
  esp32-rmt-pwm-decoder - Arduino libary for decoding an RF keyfob with an ESP32
  Copyright (c) 2025 Doug Brann.  All right reserved. 
  Please see Readme file for list of credits, howto, etc.
  
  Project home: https://github.com/?????

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __ESP32_AUDIO_SAMPLER__
#define __ESP32_AUDIO_SAMPLER__

#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"

#define ADC_CONV_FRAME_SZ 16 //number of elements in array - originally 128
#define ADC_SAMPLE_RATE 22050
#define ADC_MAX_STORE_BUF_SZ  ADC_CONV_FRAME_SZ*SOC_ADC_DIGI_DATA_BYTES_PER_CONV

#ifndef ESP32
#error esp32-audio-sampler can only be compiled on an ESP32
#endif
#if (ESP_IDF_VERSION_MAJOR < 5)
#error esp32-audio-sampler requires ESP-IDF version 5 or greater
#endif	

class AudioADCProcessor{
  private:
  static adc_unit_t adc_unit;
  static adc_channel_t adc_channel;
  static adc_continuous_handle_t adc_handle;
  static uint32_t filteredVal;
  static uint32_t rawValue;
  static TaskHandle_t s_task_handle;
  static void continuous_adc_init();
  static int32_t applyfilter(int32_t v);
  static void setup_coeff();
  static xSemaphoreHandle adcMutex;
  public:
  AudioADCProcessor(){};
	~AudioADCProcessor(){};
  void begin ();
  void initialize(adc_unit_t p_adcUnit, adc_channel_t p_adcChannel, SemaphoreHandle_t *xMutexAdcSound);
  static void readAndProcessAdc (void *parameters);
  static uint32_t processedAdcData[ADC_CONV_FRAME_SZ];
};
#endif //__ESP32_AUDIO_SAMPLER__
