/*
  esp32-rmt-pwm-decoder - Arduino libary for decoding an RF keyfob with an ESP32
  Copyright (c) 2025 Doug Brann.  All right reserved. 
  Please see Readme file for list of credits, howto, etc.
  
  Project home: https://github.com/TheDougMiester

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.B00b00


  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "esp32-rmt-pwm-decoder.h"
#include "freertos/FreeRTOS.h"
#include "soc/gpio_struct.h"

#define NUMBER_OF_RMT_SYMBOLS 128
#define BIT_LENGTH 24 // number of ones and zeros in the signal we're trying to read.
#include "esp_timer.h"

void RxDecoder::rxSignalHandler(void* param){
	
	rmt_symbol_word_t symbols[NUMBER_OF_RMT_SYMBOLS];
	rmt_rx_done_event_data_t rx_data = {
		.received_symbols = symbols,
		.num_symbols = 0,
	};
	
	QueueHandle_t rx_queue = xQueueCreate(1, sizeof(rx_data));

	rmt_receive_config_t rx_config = {

		// The max value the register can hold is 255/(rx_ch_conf.clk_src), see
		// https://github.com/espressif/esp-idf/issues/14760 
		.signal_range_min_ns = 3000, //3.00µs

		//signal_range_max_ns isn't exactly a filter. Pulses above this value trigger a "done" event. 
		// uint32_t idle_reg_value = ((uint64_t)channel->resolution_hz * config->signal_range_max_ns) / 1000000000UL;
		.signal_range_max_ns = 32000000, // this is the biggest number I could use with .resolution_hz = 1000000. (32ms)
	
		.flags {
			.en_partial_rx = 0,
		}    
	};

	rmt_rx_channel_config_t rx_ch_conf = {
		.gpio_num = RxDecoder::rxPin, // GPIO number
		.clk_src = RMT_CLK_SRC_RC_FAST, //RMT_CLK_SRC_DEFAULT (80MHz),       RMT_CLK_SRC_RC_FAST //(8.5 MHz)// select source clock
		.resolution_hz =  1000000, // with RMT_CLK_SRC_DEFAULT, 1MHz tick resolution is 1 tick = 1us		
		.mem_block_symbols = NUMBER_OF_RMT_SYMBOLS, 
		.flags = {
			.invert_in = rxProtocol.INVERSE_LEVEL,      
			.with_dma = 1,  //if set to true on ESP-IDV v < 5.5, the data stops coming through. No idea why.
		}
	};
	//I'm not clear on whether I even need this.
	rmt_carrier_config_t rx_carrier_cfg = {
		.frequency_hz = 25000,    // 25 KHz carrier, should be smaller than the transmitter's carrier frequency
		.duty_cycle = 0.33,       // duty cycle 33%
		.flags =  {
			.polarity_active_low = false, // the carrier is modulated to high level
		}	
	};

	rmt_channel_handle_t rx_channel = NULL;
	rmt_new_rx_channel(&rx_ch_conf, &rx_channel);   
	rmt_rx_event_callbacks_t cbs = {
		.on_recv_done = rxDone,
	};
	ESP_ERROR_CHECK(rmt_apply_carrier(rx_channel, &rx_carrier_cfg));
	ESP_ERROR_CHECK(rmt_enable(rx_channel));
	ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, rx_queue));
	ESP_ERROR_CHECK(rmt_receive(rx_channel, symbols, sizeof(symbols), &rx_config));
	
	while(1) {	
		// If you set timeout to 0, the scheduler never gets control to assign other tasks (and the whole program gets stuck in
		// the below wile loop). So, set the timeout to at least one clock tick or insert a vTaskDelay(1) to delay 1 clock tick. 
		if (xQueueReceive(rx_queue, &rx_data, 1)) {//portMAX_DELAY)) { 

			uint32_t rcode = 0;
			if (rx_data.num_symbols > BIT_LENGTH) { //must be greater than BIT_LENGTH for the header
				rcode = validateSignal(rx_data.received_symbols, rx_data.num_symbols);
				if (rcode !=0) {
					RxDecoder::nReceivedValue = rcode;
					RxDecoder::nReceivedBitlength = rx_data.num_symbols;
					//Serial.printf("Code: 0x%X, number of symbols %d\n", rcode, rx_data.num_symbols);
				} /* else {  //debug
					rxDataDump(rx_data.num_symbols, rx_data.received_symbols);
					//Serial.printf("----------------------\n");
				} */
		
			} //if xQueueReceive(...)
			/*else {  //debug
				Serial.printf("not enough symbols to process %d\n", rx_data.num_symbols);

				//rxDataDump(rx_data.num_symbols, rx_data.received_symbols);
			}*/			
			ESP_ERROR_CHECK(rmt_receive(rx_channel, symbols, sizeof(symbols), &rx_config));
		} //if xQueue
		vTaskDelay(1); //yield the processor

	} //while(1)
	
	rmt_disable(rx_channel);
	rmt_del_channel(rx_channel);
	vQueueDelete(rx_queue);
	vTaskDelete(NULL);
}

void RxDecoder::setRxPin(gpio_num_t l_rxPin){
	RxDecoder::rxPin = l_rxPin;
	//pinMode((uint8_t)RxDecoder::rxPin, INPUT);
	gpio_set_pull_mode(l_rxPin, GPIO_FLOATING); //based on advice I found online..
}

void RxDecoder::rxDataDump(size_t len, rmt_symbol_word_t *item) { 
	Serial.printf("Rx len (# of bits) %d: \n", len);							
	for (uint8_t i=0; i < len ; i++ ) {
		Serial.printf("%d,%d,", item[i].duration0, item[i].duration1); 
	}
	Serial.println();								
}

uint32_t RxDecoder::validateSignal(rmt_symbol_word_t *item, size_t &len) {

    uint32_t code = 0x00000000;
	int32_t bit_value = -1;
	size_t num_bits_found = 0;
	size_t i;
	
	int8_t header_pos = 0;
	header_pos = checkHeaderWord(item, len, 0 );
	if (header_pos < 0 ) return 0;
	for(i = header_pos+1, num_bits_found = 0; (len - i > BIT_LENGTH), num_bits_found < BIT_LENGTH; i++){ 
	
		bit_value = validateRmtWord(item[i]);
		if(bit_value >= 0) {
			// Shift code left by 1 bit and OR with returned value. If m was 11100 and bit_value ==1, m would become 111001
            code = (code << 1) | bit_value;
			num_bits_found++;
		} else {
			//debug
			//Serial.printf("FAIL: i:%d, num_bits_found: %d, d0:%d, d1:%d, L0:%d L1:%d\n",
			//i, num_bits_found, item[i].duration0, item[i].duration1, item[i].level0, item[i].level1);

			//something went sideways. From looking at the data dumps it seems to usually mean
			//that some crappy data snuck in. We can keep trying though - most fobs send the signal 3 times
			// find the next header and keep going.
			header_pos = checkHeaderWord(item, len, i);
			if (header_pos < 0 ) return 0;
			// We found another header. Now do something evil in a "for" loop: Adjust the counter. I'm sorry.
			i = header_pos;
			code = 0x00000000;
			num_bits_found = 0;
		}
	}
    if (num_bits_found < BIT_LENGTH) {
		// there weren't enough bits left to get a 24-bit code.
		//Serial.printf("not enough bits. %d", num_bits_found);
		code = 0x00000000;
	}  		
	return code;
}

int32_t RxDecoder::validateRmtWord(rmt_symbol_word_t &item) {
	if	((item.duration0 < rxProtocol.uSecOneHigh_upperBound) && (item.duration0 > rxProtocol.uSecOneHigh_lowerBound) &&
		(item.duration1 < rxProtocol.uSecOneLow_upperBound) && (item.duration1 > rxProtocol.uSecOneLow_lowerBound)) {
		return(1);
	} else if ((item.duration0 < (rxProtocol.uSecZeroHigh_upperBound) && (item.duration0 > rxProtocol.uSecZeroHigh_lowerBound)) &&
		(item.duration1 < (rxProtocol.uSecZeroLow_upperBound)) && (item.duration1 > rxProtocol.uSecZeroLow_lowerBound)) {
		return(0);
	} else {
		return(-1);
	}
}
int8_t RxDecoder::checkHeaderWord(rmt_symbol_word_t *item, size_t &len, uint8_t starting_point) {
	//sometimes you get a synch bit in the 0th or first element, sometimes you have to go out a few
	//but if there isn't enough left in the array to have a header and the word, stop trying.
	uint8_t i;
	for (i=starting_point; i < (len-BIT_LENGTH-1) ; i++ ) {
		if ((item[i].duration1 > (rxProtocol.usecSynchLow_lowerBound-200)))
		//just look for the long 0. I haven't found any cases where the other bit is needed. IF you do, just uncomment below
		//&&
		//((item[i].duration0 < rxProtocol.usecSynchHigh_upperBound) && (item[i].duration0 > rxProtocol.usecSynchHigh_lowerBound)))
		{
			//Serial.printf("header found at %u, val %d len=%d\n", i, item[i].duration1,len );
			return(i);
		} 
	}

	//if we got here, well...we never found the header. Return a value greater than the length.
	//Serial.printf("header not found counter was %i, len-bitlength was:%d \n", i, len-BIT_LENGTH-1,len);
	//rxDataDump(len, item);
	return(-1);
}

bool RxDecoder::rxDone(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *udata){
	BaseType_t task_wakeup = pdFALSE;
	QueueHandle_t queueHandle = (QueueHandle_t)udata;
	xQueueSendFromISR(queueHandle, edata, &task_wakeup);
	return task_wakeup = pdTRUE; 
}

bool RxDecoder::available() {
  return RxDecoder::nReceivedValue != 0;
}

void RxDecoder::resetAvailable() {
  RxDecoder::nReceivedValue = 0;
}
uint32_t RxDecoder::getReceivedValue() {
  return RxDecoder::nReceivedValue;
}

uint8_t RxDecoder::getReceivedBitlength() {
  return RxDecoder::nReceivedBitlength;
}

//required stupidness because these variable need to be static. Otherwise, the program won't link.. :(
gpio_num_t RxDecoder::rxPin; 
uint32_t RxDecoder::nReceivedValue;
uint8_t RxDecoder::nReceivedBitlength;
