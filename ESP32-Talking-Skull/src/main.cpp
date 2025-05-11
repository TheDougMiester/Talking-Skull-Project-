#include <Arduino.h>
#include <pthread.h>
#include "esp32-rmt-pwm-decoder.h"
#include "RidiculouslySmallDfPlayer.h"
#include "EyeMovements.h"
//#include "MSGEQ7.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "soc/gpio_struct.h"
#include "AudioADCProcessor.h"

RxDecoder rfReceiver;
static const gpio_num_t RF_RECEIVER_PIN = GPIO_NUM_11;

volatile uint8_t fifty_hz_timer_flag = 0;
#define FLAG_50HZ 0 
static void fifty_hz_timer_callback(void *arg) {
  /* This pushes handling into loop(). For this reason avoid using delay() */
  /* Waits can (and should) be implemented with countdowns in loop() using this 50Hz tick */
  /* or with some other non-blocking timer*/
   fifty_hz_timer_flag |= (1 << FLAG_50HZ);
}
static esp_timer_handle_t fifty_hz_timer_handle = NULL;
const esp_timer_create_args_t fifty_hz_timer_config = {
  .callback = &fifty_hz_timer_callback, // The callback function to be executed
  .arg = NULL, // Optional argument to pass to the callback
  .name = "fifty_hz_timer" // Optional name for debugging
};

//DF Player
bool playerIsPlaying = false;
static const uint8_t DFPLAYER_TX = GPIO_NUM_37;  
static const uint8_t DFPLAYER_RX = GPIO_NUM_36; 
static const uint8_t DFPLAYER_BUSY = GPIO_NUM_38;
static const uint8_t DFPLAYER_DAC_L = GPIO_NUM_41; // for FFT processing
static const uint32_t DFPLAYER_BAUD = 9600;
RidiculouslySmallDfPlayer dfPlayer;
HardwareSerial DfPlayerSerial(2);

//eyes
static const uint8_t LEFT_EYE = 21;
static const uint8_t RIGHT_EYE = 47;
EyeMovements ledEyes;

//Sound processing
//ADC_UNIT_1 with ADC_CHANNEL_4 maps to GPIO_5
static const adc_unit_t audio_adc_unit = ADC_UNIT_1;
static const adc_channel_t audio_adc_channel = ADC_CHANNEL_4; //maps to GPIO 4
AudioADCProcessor audioADC;
uint32_t jawInfo[ADC_CONV_FRAME_SZ];
SemaphoreHandle_t xMutexAdcSound =  xSemaphoreCreateMutex();

void stopPlaying() {
  playerIsPlaying = false;
  // make sure the DF_PLAYER is stopped.
  dfPlayer.stopPlaying();
  ledEyes.eyesOff();
  //closeMouth();
}
void setup() {
  Serial.begin(115200);
  delay(3000);

  rfReceiver.setRxPin(RF_RECEIVER_PIN);

  audioADC.initialize(audio_adc_unit, audio_adc_channel, &xMutexAdcSound); //maps to GPIO_5
  audioADC.begin();
  
  //df player
  DfPlayerSerial.begin(DFPLAYER_BAUD, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
  dfPlayer.begin(DfPlayerSerial, DFPLAYER_BUSY);
  //I don't know if I need these delays...
  delay(3000);
  dfPlayer.resetPlayer();
  delay(3000);
  dfPlayer.setVolume(31);
  
  ledEyes.setEyePins(LEFT_EYE,RIGHT_EYE);
  //let the user know the system is online.
  ledEyes.bothEyesOn();
  delay(2000);
  ledEyes.eyesOff();
  //put the receiver reader on core 1
  xTaskCreatePinnedToCore(rfReceiver.rxSignalHandler, "rxSignalHandler", 4096, NULL, 10, NULL, 1);
  //put the audio task on core 0
  xTaskCreatePinnedToCore(audioADC.readAndProcessAdc, "readAndProcessAdc", 4096, NULL, 11, NULL,0);

  //xTaskCreate(audioADC.adc_fft_task, "adc_fft_task", 8192, NULL, 4, NULL);
  delay(5000);

  esp_timer_create(&fifty_hz_timer_config, &fifty_hz_timer_handle);
  esp_timer_start_periodic(fifty_hz_timer_handle, 20000); // 50hz - 5 times per second
  Serial.printf("setup complete\n"); 
  #if 0
  Serial.printf("button 4\n");
  //Serial.printf("## code: 0x%X bit length: %d\n", 
  //rfReceiver.getReceivedValue(), rfReceiver.getReceivedBitlength() );
  dfPlayer.playTrack(4);
  playerIsPlaying = true;
  ledEyes.bothEyesOn();
  #endif
  vTaskDelay(pdMS_TO_TICKS(50u)); // give the player 50 ms to star
}

uint32_t decoded;
bool isBusy = false;

void loop() {

  if (fifty_hz_timer_flag & (1 << FLAG_50HZ)){// runs 50 times per sec
    fifty_hz_timer_flag &= ~(1 << FLAG_50HZ);
    isBusy = dfPlayer.isBusy();
    if((playerIsPlaying == true) && (!(isBusy))) {
      stopPlaying();
      Serial.printf("player is done\n");
    } else if (playerIsPlaying == true) {
      xSemaphoreTake(xMutexAdcSound, portMAX_DELAY);
        memcpy(jawInfo, audioADC.processedAdcData, sizeof(audioADC.processedAdcData));
        // Fill the array with zeros for ADC to reload
        memset( audioADC.processedAdcData, 0, sizeof(audioADC.processedAdcData));
      xSemaphoreGive(xMutexAdcSound);
      // put jaw stuff in here
      for (int i = 0; i < ADC_CONV_FRAME_SZ; i++) {
        Serial.printf("jaw = %d\n", jawInfo[i]);
      }
    }

    /* This is the only place (other than startup) where a blocking timer is OK*/
    /* Since this is the area where the voice stuff starts, an interrupt won't hurt anything*/
    if (rfReceiver.available()) {
      decoded = rfReceiver.getReceivedValue();
      switch (decoded) {   //translate fob codes (first 4)
        case 0x4C8568:
          Serial.printf("button 1\n");
          dfPlayer.playTrack(1);
          playerIsPlaying = true;
          ledEyes.bothEyesOn();
          vTaskDelay(pdMS_TO_TICKS(50u)); // give the player 50 ms to start        
          break;
        case 0x4C8564:
          Serial.printf("button 2\n"); 
          dfPlayer.playTrack(2);
          playerIsPlaying = true;
          ledEyes.bothEyesOn();
          vTaskDelay(pdMS_TO_TICKS(50u)); // give the player 50 ms to start        
          break;
        case 0x4C856C:
          Serial.printf("button 3\n");
          dfPlayer.playTrack(3);
          playerIsPlaying = true;
          ledEyes.bothEyesOn();
          vTaskDelay(pdMS_TO_TICKS(50u));// give the player 50 ms to start        
          break;
        case 0x4C8562:
          Serial.printf("button 4\n");
          //Serial.printf("## code: 0x%X bit length: %d\n", 
          //rfReceiver.getReceivedValue(), rfReceiver.getReceivedBitlength() );
          dfPlayer.playTrack(4);
          playerIsPlaying = true;
          ledEyes.bothEyesOn();
          vTaskDelay(pdMS_TO_TICKS(50u)); // give the player 50 ms to start        
          break;
        case 0x4C856A  :
          Serial.printf("button 5\n");
          //Serial.printf("## code: 0x%X bit length: %d\n", 
          //rfReceiver.getReceivedValue(), rfReceiver.getReceivedBitlength() );          
          break;
        case 0x4C8566:
          Serial.printf("button 6\n");          
          break;
        case 0x4C856E:
          Serial.printf("button 7\n");
          break;
        case 0x4C8561:
          Serial.printf("button 8\n");
          //Serial.printf("## code: 0x%X bit length: %d\n", 
          //  rfReceiver.getReceivedValue(), rfReceiver.getReceivedBitlength() );
          dfPlayer.stopPlaying();
          playerIsPlaying = true;
          ledEyes.eyesOff();

          vTaskDelay(pdMS_TO_TICKS(50u));; // give the player 50 ms to stop
          break;
        default:
          Serial.printf("Unknown signal: 0x%X bit length: %d\n", 
            rfReceiver.getReceivedValue(), rfReceiver.getReceivedBitlength() );
            break;
      } // end of switch
      rfReceiver.resetAvailable();    
    } //if receiver

  } //if (fifty_hz_timer_flag
  vTaskDelay(1); //delay for one clock tick to yield processing for other threads
}  //loop
