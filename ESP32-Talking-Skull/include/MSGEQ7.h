/*
  MSGEQ7 - small Arduino libary for setting up MSGEQ7 on an ESP32
  Program is highly limited in functionality, 

  Copyright (c) 2025 Doug Brann. All right reserved.
  Other contributors:
  Torai Majid
  
  Project home: https://github.com/TheDougMiester

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef __MSGEQ7__
#define __MSGEQ7__
//#include "driver/ledc.h"
//#include "DacTone.h"           // this is the DAC audio library
#include <Arduino.h>
#include "driver/ledc.h"
#include <esp32-hal-adc.h>


class MSGEQ7 {
  private:
    uint8_t dataPin; //multiplexed DC out to ESP32 (pin 3)
    uint8_t strobePin;  //strobed channel selection pin (pin 4)
    uint8_t resetPin;  // resets multiplexer (pin 7)
    uint8_t clkPin;  //sets clock input pin (pin 8)
    uint32_t clkFreq; //frequency of square wave on pin 6 of msgeq7
	  const static uint8_t ledChannel = 0;
	  const static uint8_t resolution = 8;
  
    // array of all input values
    const static uint8_t DATA_BAND_MAX = 7;		///< Number of bands output by the hardware IC
	  uint16_t _data[DATA_BAND_MAX];

  public:

    /* MSGEQ7 Pin Configuration
      1. VDD Positive Power Supply Typically 5 Volts (3.3V for ESP32)
      2. VSS Negative Power Supply Typically 0 Volts
      3. OUT Multiplexed DC Output
      4. STROBE Channel Selection Pin
      5. IN Audio Input (from audio player)
      6. GND Internally Generated Ground Reference.Typically 2.5V
      7. RESET Resets Multiplexor
      8. CKIN Clock Oscillator Pin
    */
    /**
     * @brief Initiate a clock to drive the MSGEQ7 - output goes to pin 6 on chip
     *
     * @note This function is non-blocking; it does its thing and returns. Run it in setup(), not loop()
     * @note It establishes pin setup and initiates a squarewave driver
     * @note This function can be called in ISR context.
     *
     * @param[in] data_pin - Multiplexed DC output to ESP32 (msgeq7 pin 3)
     * @param[in] strobe_pin - strobed channel selection pin (msgeq7 pin 4)
     * @param[in] reset_pin - resets multiplexer (msgeq7 pin 7)
     * @param[in] clk_pin - clock oscillator pin (msgeq7 pin 8)
     * @param[in] freq - the desired frequency of the clk_in_pin sqarewave (165000 -- 165kHz recommended)
     * @return - void
     */
    void initialize(uint8_t data_pin /*pin 3*/, uint8_t strobe_pin /*pin 4*/, 
                               uint8_t reset_pin /* pin 7*/ , uint8_t clk_pin /*pin 8 */, uint32_t freq /*in hz*/) {

      //establish pins
      dataPin = data_pin;
      pinMode(dataPin, INPUT);
      strobePin = strobe_pin;
      pinMode(strobePin, OUTPUT);
      resetPin = reset_pin;
      pinMode(resetPin, OUTPUT);
      clkPin = clk_pin;
      pinMode(clkPin, OUTPUT);

      // set up oscillating clock freq
	    clkFreq = freq; // MSGEQ7 clock pin should be 165000Hz (165kHz) (ESP32 max square wave freq = 300khz)
      // Set up analog read of MSGEQ7 digital pin
	    analogReadResolution(10); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
	    analogSetAttenuation(ADC_6db); // Default is 11db which is very noisy. Recommended to use 2.5 or 6.

    };

    void beginOscClock(){

      // Square wave configuration
      // Configure the LEDC channel for a square wave
      // Attach the LEDC channel to the clkInPin
      ledcAttach(dataPin,clkFreq, resolution);
	    ledcWrite(0, 128); // // 50% duty cycle (8 bit resolution: 128/255)
    };

void reset(void)
{
	// Reset the IC according to the chart in the data sheet
	// |<0.1us>|<--------- 72us --------->|
	//  _______                           
	// |       |                          |  RESET
	// |       |__________________________|
	// 
	//  _       __________________________                           
	// | |     |                          |  STROBE
	// | |_____|                          |
	// 

	digitalWrite(strobePin, HIGH);
	digitalWrite(strobePin, LOW);
	digitalWrite(resetPin, HIGH);
	
	delayMicroseconds(1); // tr = 100 nanoseconds here - likely the Arduino library will be slower than this anyway
	
	digitalWrite(resetPin, LOW);
	digitalWrite(strobePin, HIGH);
	delayMicroseconds(72);  // trs = 72 microseconds
}
void read(bool bReset) {
// Read all the values from the IC and store in local object array

	if (bReset) reset(); 	// reset the IC if required

	// read all MAX_BAND channels 
	for (int i = 0; i < DATA_BAND_MAX; i++) {
		// |<--------- 72us --------->| x MAX_BANDS
		//                      ______
		// |                   |      |
		// |                   |      |
		// |                   |      |
		// |<---36us--->|<18ms>|<18us>|
		// |____________|______|      |
		//              ^ADC measure

		// trigger next value
		digitalWrite(strobePin, LOW);

		// allow the output to settle
		delayMicroseconds(36);  // to = 36 microseconds

		// read pin
		_data[i] = analogRead(dataPin);
    
        delayMicroseconds(18);

		digitalWrite(strobePin, HIGH);
        delayMicroseconds(18);  // ts = 18 microseconds
  } //for
}

    MSGEQ7(){};
	~MSGEQ7(){};

};
#endif //#define __MSGEQ7__