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

#ifndef __ESP32_RMT_RX_DECODER__
#define __ESP32_RMT_RX_DECODER__
#include <Arduino.h>
#include "soc/reg_base.h"
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_idf_version.h"

/* definition to expand macro then apply to pragma message */
#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var "="  VALUE(var)

#ifndef ESP32
#error esp32-rmt-pwm-decoder can only be compiled on an ESP32
#endif
#if (ESP_IDF_VERSION_MAJOR < 5) 
#error esp32-rmt-pwm-decoder requires ESP-IDF version 5 or greater
#elif (ESP_IDF_VERSION_MINOR < 4)
#pragma message (VAR_NAME_VALUE(ESP_IDF_VERSION_MINOR))
#error esp32-rmt-pwm-decoder requires ESP-IDF version 5.4 or greater
#endif	

 class RxDecoder {
  private:
	static gpio_num_t rxPin;
	static uint32_t nReceivedValue;
	static uint8_t nReceivedBitlength;
	static int8_t checkHeaderWord(rmt_symbol_word_t *item, size_t &len, uint8_t starting_point);
    static uint32_t validateSignal(rmt_symbol_word_t *item, size_t &len);
    static int32_t validateRmtWord(rmt_symbol_word_t &item);
    static void rxDataDump(size_t len, rmt_symbol_word_t *item);
    IRAM_ATTR static bool rxDone(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *udata);

  public:
    RxDecoder(){};
	~RxDecoder(){};
	
	static void setRxPin(gpio_num_t l_rxPin);
	static void rxSignalHandler(void* param);
	static bool available();
	static void resetAvailable();
	static uint32_t getReceivedValue();
	static uint8_t getReceivedBitlength();

  private:
    // what follows are compile-time calculations to build a
    // template for the receiver protocol
    template<
	    /** A protocol specification is given by the following parameters: */
	    uint32_t protocolNumber,                 /* A unique integer identifier of this protocol. */
	    uint32_t usecClock,                      /* The clock rate in microseconds.  */
	    uint32_t percent_tolerance,               /* The tolerance for a pulse length to be recognized as a valid. */
	    uint32_t synch_high, uint32_t synch_low, /* Number of clocks for the synchronization pulse pair. */
	    uint32_t zero_high, uint32_t zero_low,   /* Number of clocks for a logical 0 bit data pulse pair. */
	    uint32_t one_high, uint32_t one_low,     /* Number of clocks for a logical 1 bit data pulse pair. */
	    bool inverseLevel>                       /* Flag whether pulse levels are normal or inverse. */

    // Calculate the timing specification from the protocol definition.
    struct makeTimingSpec {
	    static constexpr uint32_t PROTOCOL_NUMBER = protocolNumber;
	    static constexpr bool INVERSE_LEVEL = inverseLevel;

	    static constexpr uint32_t uSecSynchHigh = usecClock * synch_high;
	    static constexpr uint32_t uSecSynchLow = usecClock * synch_low;
	    static constexpr uint32_t usecSynchHigh_lowerBound = static_cast<uint32_t>(uSecSynchHigh) * (100-percent_tolerance) / 100;
	    static constexpr uint32_t usecSynchHigh_upperBound = static_cast<uint32_t>(uSecSynchHigh) * (100+percent_tolerance) / 100;
	    static constexpr uint32_t usecSynchLow_lowerBound = static_cast<uint32_t>(uSecSynchLow) * (100-percent_tolerance) / 100;
	    static constexpr uint32_t usecSynchLow_upperBound = static_cast<uint32_t>(uSecSynchLow) * (100+percent_tolerance) / 100;

	    static constexpr uint32_t uSecZeroHigh = usecClock * zero_high;
	    static constexpr uint32_t uSecZeroLow = usecClock * zero_low;
	    static constexpr uint32_t uSecZeroHigh_lowerBound = static_cast<uint32_t>(uSecZeroHigh) * (100-percent_tolerance) / 100;
	    static constexpr uint32_t uSecZeroHigh_upperBound = static_cast<uint32_t>(uSecZeroHigh) * (100+percent_tolerance) / 100;
	    static constexpr uint32_t uSecZeroLow_lowerBound = static_cast<uint32_t>(uSecZeroLow) * (100-percent_tolerance) / 100;
	    static constexpr uint32_t uSecZeroLow_upperBound = static_cast<uint32_t>(uSecZeroLow) * (100+percent_tolerance) / 100;

	    static constexpr uint32_t uSecOneHigh = usecClock * one_high;
	    static constexpr uint32_t uSecOneLow = usecClock * one_low;
	    static constexpr uint32_t uSecOneHigh_lowerBound = static_cast<uint32_t>(uSecOneHigh) * (100-percent_tolerance) / 100;
	    static constexpr uint32_t uSecOneHigh_upperBound = static_cast<uint32_t>(uSecOneHigh) * (100+percent_tolerance) / 100;
	    static constexpr uint32_t uSecOneLow_lowerBound = static_cast<uint32_t>(uSecOneLow) * (100-percent_tolerance) / 100;
	    static constexpr uint32_t uSecOneLow_upperBound = static_cast<uint32_t>(uSecOneLow) * (100+percent_tolerance) / 100;
    };

    // now use the template to build the stuct
	//			    	 <#, clk,  %, syHi,  syLo,  0hi,0lo,  1hi,1lo,invLvl> ->protocol structure
    //static makeTimingSpec< 1, 450, 20,   1,   31,    1,  3,    3,  1, false>   rxProtocol; //PT2262 4-button fob
	static makeTimingSpec< 1, 360, 60,   1,   31,    1,  3,    3,  1, false>   rxProtocol; //PT2262 8-button remote
    // end of compile-time calculations
	
};
#endif
