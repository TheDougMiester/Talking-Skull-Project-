#include "esp32-rmt-pwm-decoder.h"
RxDecoder receiver;
//#include <Arduino.h>
void setup() {
  Serial.begin(115200);
  receiver.setRxPin(10);
  pinMode(10, INPUT);
  delay(1000);
  Serial.println("in setup"); 
  
  xTaskCreatePinnedToCore(receiver.rxSignalHandler, "rxSignalHandler", 4096, NULL, 10, NULL, 1);
}

void loop() {
//    Serial.println("in loop"); 
  if (receiver.available()) {
    Serial.printf("Received 0x%X, bit length %d\n", 
    receiver.getReceivedValue(), receiver.getReceivedBitlength() );

    receiver.resetAvailable();
  }
}
