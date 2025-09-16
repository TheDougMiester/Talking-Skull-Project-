/*
  EyeMovements - Goofy little Arduino class libary for
  blinking a couple of leds in the eye sockets of a 
  halloween skeleton. It's not spectacular - I just
  wanted to get this code out of my main program. In
  truth, it's just some C code in a C++ wrapper (and
  not very clever C code either).

  Copyright (c) 2025 Doug Brann. All right reserved.
  
  Project home: https://github.com/?????

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
#include <Arduino.h>
#ifndef _EYE_MOVEMENTS__
#define _EYE_MOVEMENTS__
const uint8_t EYES_OFF=6;
const uint8_t ALTERNATING_BLINK=5;
const uint8_t SYNCH_BLINK=4;
uint8_t alternatingBlink[] = {HIGH, LOW, LOW, HIGH};
uint8_t synchBlink[] = {HIGH, HIGH, LOW, LOW};

uint8_t *blinkPatternPtr;
uint16_t *blinkSpeedPtr;
const uint8_t FAST_BLINK=3;
const uint8_t MEDIUM_BLINK=2;
const uint8_t SLOW_BLINK=1;
const uint8_t NO_BLINK = 0;

uint16_t blinkSpeedSLOW[] = {1000, 1000};  // time to wait in each interval
uint16_t blinkSpeedMEDIUM[] =  {200, 200};
uint16_t blinkSpeedFAST[] = {100, 100};
uint8_t LEFT_EYE_PIN;
uint8_t RIGHT_EYE_PIN;
uint16_t blinkSpeed;
class EyeMovements {
  private:

    // sets pointers to arrays -- this sets blink speed and pattern of LED eyes
    void setBlinkSpeed(const uint16_t l_blinkSpeed) {
      blinkSpeed = l_blinkSpeed;
      //Serial.println(blinkPattern);  Serial.println(blinkSpeed);
      switch(blinkSpeed) {
        case FAST_BLINK:
          blinkSpeedPtr = &blinkSpeedFAST[0];
          break;
        case MEDIUM_BLINK: 
          blinkSpeedPtr = &blinkSpeedMEDIUM[0];
          break;
        case SLOW_BLINK:
          blinkSpeedPtr = &blinkSpeedSLOW[0];
          break;
        case NO_BLINK:  
        default:
          break;
      } 
    };
    void setBlinkPattern(const uint8_t blinkPattern) {
      switch (blinkPattern) {
        case ALTERNATING_BLINK:
          blinkPatternPtr = &alternatingBlink[0];
          break;
        case SYNCH_BLINK: 
          blinkPatternPtr = &synchBlink[0];
          break;
        case NO_BLINK: ///NO_BLINK is handled by blink_speed, not pattern
        default:
          break;  
      }
    };


  public:
    void setEyePins(uint8_t l_leftEye, uint8_t l_rightEye) {
        LEFT_EYE_PIN = l_leftEye;
        RIGHT_EYE_PIN = l_rightEye;
        pinMode(LEFT_EYE_PIN, OUTPUT);
        pinMode(RIGHT_EYE_PIN, OUTPUT);
    };

    void blinkFast() {
      setBlinkSpeed(FAST_BLINK);
    };
    void blinkMedium(){
      setBlinkSpeed(MEDIUM_BLINK);
    };
    void blinkSlow() {
      setBlinkSpeed(SLOW_BLINK);
    };
    void alternateEyesOn(){
      setBlinkPattern(ALTERNATING_BLINK);
    };
    void synchEyesOn(){
      setBlinkPattern(SYNCH_BLINK);
    };
    void bothEyesOn(){
      digitalWrite(RIGHT_EYE_PIN, HIGH);
      digitalWrite(LEFT_EYE_PIN, HIGH);
      setBlinkSpeed(NO_BLINK);
    };
    void eyesOff(){
      digitalWrite(RIGHT_EYE_PIN, LOW);
      digitalWrite(LEFT_EYE_PIN, LOW);
      setBlinkSpeed(NO_BLINK);
    };

    // executes the timing of LED eye blinks.
    void Blink() {
      if (blinkSpeed == NO_BLINK) {return;}

      static uint32_t previousMillisEyes = 0;
      static byte blinkState = 1;
      uint32_t timeHack = millis();
      uint32_t deltaEyes = timeHack - previousMillisEyes;
      // now check to see if we have to do anything
      if (deltaEyes >= blinkSpeedPtr[blinkState]) {
        // it's time for next state for the eyes
        blinkState++;
        blinkState = blinkState % 2;
        switch (blinkState) {
          case 0:
            digitalWrite(RIGHT_EYE_PIN, blinkPatternPtr[0]);
            digitalWrite(LEFT_EYE_PIN,  blinkPatternPtr[1]);
            //Serial.print("Eye delta "); Serial.print(deltaEyes); Serial.print(" case 0, right: "); Serial.print(blinkPatternPtr[0]); Serial.print(" left "); Serial.println(blinkPatternPtr[1]);
            break;
          case 1:
            digitalWrite(RIGHT_EYE_PIN, blinkPatternPtr[2]);
            digitalWrite(LEFT_EYE_PIN, blinkPatternPtr[3]);
            //Serial.print("Eye delta "); Serial.print(deltaEyes); Serial.print(" case 1, right: "); Serial.print(blinkPatternPtr[2]); Serial.print(" left "); Serial.println(blinkPatternPtr[3]);
            break;
          default: 
            //Serial.print("oops. State = "); Serial.println(blinkState);
            break;
        }
        previousMillisEyes = timeHack;
      }
    };
    EyeMovements(){};
	  ~EyeMovements(){};

};
#endif //#define _EYE_MOVEMENTS__
