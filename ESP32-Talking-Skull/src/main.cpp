#include <Arduino.h>
#include <pthread.h>
#include "esp32-rmt-pwm-decoder.h"
#include "RidiculouslySmallDfPlayer.h"
#include "EyeMovements.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "soc/gpio_struct.h"
#include "AudioADCProcessor.h"
#include <esp32-hal-ledc.h>

RxDecoder rfReceiver;
static const gpio_num_t RF_RECEIVER_PIN = GPIO_NUM_15;

volatile uint8_t fifty_hz_timer_flag = 0;
#define FLAG_50HZ 0 
static void fifty_hz_timer_callback(void *arg) {
  /* This pushes handling into loop(). For this reason avoid using delay() */
  /* Waits can (and should) be implemented with countdowns in loop() using this 50Hz tick */
  /* or with some other non-blocking timer*/
   fifty_hz_timer_flag |= (1 << FLAG_50HZ);
}
uint8_t loop_10_Hz = 5; // 50Hz divide rate by 10

static esp_timer_handle_t fifty_hz_timer_handle = NULL;
const esp_timer_create_args_t fifty_hz_timer_config = {
  .callback = &fifty_hz_timer_callback, // The callback function to be executed
  .arg = NULL, // Optional argument to pass to the callback
  .name = "fifty_hz_timer" // Optional name for debugging
};

//DF Player
bool playerIsPlaying = false;
static const uint8_t DFPLAYER_TX = GPIO_NUM_46; //tx TO ESP32's RX
static const uint8_t DFPLAYER_RX = GPIO_NUM_11; //rx FROM ESP32's TX
static const uint8_t DFPLAYER_BUSY = GPIO_NUM_13;
//static const uint8_t DFPLAYER_DAC_L = GPIO_NUM_4; // for audio processing - not used (see ADC below)
static const uint32_t DFPLAYER_BAUD = 9600;
RidiculouslySmallDfPlayer dfPlayer;
HardwareSerial DfPlayerSerial(2);

//eyes
static const uint8_t LEFT_EYE = GPIO_NUM_5;
static const uint8_t RIGHT_EYE = GPIO_NUM_7;
EyeMovements ledEyes;

//Sound processing
//ADC_UNIT_1 with ADC_CHANNEL_3 maps to GPIO_NUM_4
static const adc_unit_t audio_adc_unit = ADC_UNIT_1;
static const adc_channel_t audio_adc_channel = ADC_CHANNEL_3;
AudioADCProcessor audioADC;
uint32_t jawInfo[ADC_CONV_FRAME_SZ];
static const uint8_t ADC_CONV_FRAME_MIDPOINT = (uint8_t (ADC_CONV_FRAME_SZ/2));
  
SemaphoreHandle_t xMutexAdcSound =  xSemaphoreCreateMutex();

/*
jaw motion:
Ok, my skull has a jaw motion where the motor was installed...upsidedown and at a funny angle.
So, its minimum angle is 85 degrees and the mak jaw angle is about 135 (I shaved off a couple
of degrees to make sure I didn't open the jaw too far).
*/
#define HS65MG
//#define MG995
#undef MG995
#ifdef MG995 
#define SERVOMIN_DEGREES 85
#define SERVOMAX_DEGREES 132
static const double SERVO_MIN_PULSE_WIDTH = 0.37; //0 degrees (experimentally determined)
static const double SERVO_MAX_PULSE_WIDTH = 2.4; //180 degrees (experimentally determined)
#elif defined(HS65MG)
#define SERVOMIN_DEGREES 93   //if motor installed upside down, this is "jaw fully open"
#define SERVOMAX_DEGREES 120  //if motor installed upside down, this is "jaw fully closed"
static const double SERVO_MIN_PULSE_WIDTH = 0.61; //0 degrees (experimentally determined, docs said .61ms)
static const double SERVO_MAX_PULSE_WIDTH = 2.4; //180 degrees (experimentally determined, but agreed w/docs)
#endif

#define JAW_PIN GPIO_NUM_18
#define JAW_FREQ_HZ 50 //hz - standard for small servos
#define JAW_FREQ_MS 20 // 1 sec/50hz(100000usec/sec)
#define JAW_FREQ_US 2000 // 1 sec/50hz(100000usec/sec)
#define JAW_RESOLUTION 13 //13 bit resolution: 2^13 = 1 <<13 = 8192


/*
Pulse with and servo angle based on for example:
1. pulse width (pw) at 0 degrees = SERVO_MIN_PULSE_WIDTH (in ms)
2. pw at 180 degrees = SERVO_MAX_PULSE_WIDTH (in ms)
3. use y=mx+b to get plot slope to calulate pulse width for 
   skull servo min and max.

I wanted to use the map() function, but I did all of this in double, and map() returns int.
*/
static const double SERVO_PLOT_SLOPE = (SERVO_MAX_PULSE_WIDTH-SERVO_MIN_PULSE_WIDTH)/180.0;

static const double SERVOMIN_PW =  SERVO_PLOT_SLOPE * SERVOMIN_DEGREES + SERVO_MIN_PULSE_WIDTH;
static const double SERVOMAX_PW =  SERVO_PLOT_SLOPE * SERVOMAX_DEGREES + SERVO_MIN_PULSE_WIDTH;
static const double SERVOZERO_PW =  SERVO_PLOT_SLOPE * 0.0 + SERVO_MIN_PULSE_WIDTH;
static const double SERVO180_PW =   SERVO_PLOT_SLOPE * 180.0 + SERVO_MIN_PULSE_WIDTH;

// DO NOT USE THESE when the servo is connected to the skull.
//static const uint16_t SERVOZERO_DC = (SERVOZERO_PW * ((1<<JAW_RESOLUTION) -1))/JAW_FREQ_MS; 
//static const uint16_t SERVO180_DC = (SERVO180_PW * ((1<<JAW_RESOLUTION) -1))/JAW_FREQ_MS; 

static const uint16_t SERVOMIN_DC = (SERVOMIN_PW * ((1<<JAW_RESOLUTION) -1))/JAW_FREQ_MS; 
static const uint16_t SERVOMAX_DC = (SERVOMAX_PW * ((1<<JAW_RESOLUTION) -1))/JAW_FREQ_MS; 
#define SUM_OF_SERVO_DUTY_CYCLES (SERVOMIN_DC+SERVOMAX_DC)


static const uint16_t JAW_CLOSED = SERVOMAX_DC; //I installed my servo upside down, so closed is max
static const uint16_t JAW_WIDE = SERVOMIN_DC; //I installed my servo upside down...
static const uint16_t SERVOINIT = JAW_CLOSED; //closed
static const uint16_t NOISE_FLOOR = 30;
static const uint8_t PULSE_WIDTHS_PER_DEGREE = (SERVOMAX_DC-SERVOMIN_DC)/(SERVOMAX_DEGREES - SERVOMIN_DEGREES);

int localAbs(int arg){      
	return arg>=0 ? arg: -arg;                      
}  
void closeMouth() {ledcWrite(JAW_PIN, JAW_CLOSED);}
void stopPlaying() {
  playerIsPlaying = false;
  // make sure the DF_PLAYER is stopped.
  dfPlayer.stopPlaying();
  ledEyes.eyesOff();
  closeMouth();
}
void setup() {
  Serial.begin(115200);
  delay(3000);
  
  Serial.printf("setup starting here\n"); 

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
  delay(1000);

  //put the receiver reader on core 1
  xTaskCreatePinnedToCore(rfReceiver.rxSignalHandler, "rxSignalHandler", 4096, NULL, 10, NULL, 1);
  //put the audio task on core 0
  xTaskCreatePinnedToCore(audioADC.readAndProcessAdc, "readAndProcessAdc", 4096, NULL, 11, NULL,0);

  //xTaskCreate(audioADC.adc_fft_task, "adc_fft_task", 8192, NULL, 4, NULL);
  delay(1000);

  //the main loop runs too fast to be useful. Execute it only 50 times per second.
  esp_timer_create(&fifty_hz_timer_config, &fifty_hz_timer_handle);
  esp_timer_start_periodic(fifty_hz_timer_handle, JAW_FREQ_US); // 50hz 

  // Configure jaw motion
  bool is_attached = ledcAttach(JAW_PIN, JAW_FREQ_HZ, JAW_RESOLUTION); 
  Serial.printf("is the pin attached? %d\n", is_attached);
  closeMouth();
  ledEyes.eyesOff();
  //ledcAttachChannel(pin, freq, bit_num, channel);

}

uint32_t decoded;
bool isBusy = false;
uint32_t filter_out;
uint32_t ema = 0; //exponential moving average - takes out the jitter in the jaw movement

uint32_t gain = 1; //default
uint16_t servo_pos;
uint16_t prev_servo_pos = 0;
//set these values based on evaluating jaw_max and jaw_min for each sound 

uint16_t map_min = 0;
uint16_t map_max = 0;
//#define SOUND_DEBUG
#undef SOUND_DEBUG
#ifdef SOUND_DEBUG
int32_t jaw_max = 0;
int32_t jaw_min = 10000;
int32_t jaw_count = 0;
int32_t jaw_total = 0;
#endif
static const int i = 0; //I have 16*4 elements to choose from from jawInfo[] array. But often,
                        // most of the array turns out to be 0s, so just grab the 0th element
                        //If there's ever a reason to look at the whole array, remove this.

void loop() {

  if (fifty_hz_timer_flag & (1 << FLAG_50HZ)) {// runs 50 times per sec
    fifty_hz_timer_flag &= ~(1 << FLAG_50HZ);

    isBusy = dfPlayer.isBusy();
    if((playerIsPlaying == true) && (!(isBusy))) {
      stopPlaying();
      Serial.printf("player is done\n");
      loop_10_Hz = 5;

    } else if (playerIsPlaying == true) {
      ledEyes.Blink(); //do whatever needs doing with the eyes.
      loop_10_Hz--;
      if (loop_10_Hz == 0 ) { 
        //A human jaw only moves at about 2hz.. max. So this if statement executes 10 times per sec (10 Hz)
        loop_10_Hz = 5;

        xSemaphoreTake(xMutexAdcSound, portMAX_DELAY);
        memcpy(jawInfo, audioADC.processedAdcData, sizeof(audioADC.processedAdcData));
        // Fill the array with zeros for ADC to reload
        memset(audioADC.processedAdcData, 0, sizeof(audioADC.processedAdcData));
        xSemaphoreGive(xMutexAdcSound);
        
        if(jawInfo[i] > NOISE_FLOOR) {
     
#ifdef SOUND_DEBUG
          if (jaw_max < jawInfo[i]) {jaw_max = jawInfo[i];}
          if (jaw_min > jawInfo[i]) {jaw_min = jawInfo[i];}
          jaw_count++;
          jaw_total+= jawInfo[i];
#endif        
        
          filter_out = jawInfo[i] * gain;
          ema = ( filter_out + ( (ema<< 2 ) - ema) ) >> 2;

          //servo_pos = map(jawInfo[i], map_min, map_max, SERVOMIN_DC,SERVOMAX_DC); // Forward servo rotation
          servo_pos = map(ema, map_min, map_max, SERVOMAX_DC, SERVOMIN_DC); // Reversed servo rotation

#ifdef SOUND_DEBUG
          Serial.printf("jaw = %d, ema %d, filter_out %d, servo_pos %d\t", jawInfo[i], ema,filter_out, servo_pos);
#endif
         if (servo_pos > SERVOMAX_DC) servo_pos = SERVOMAX_DC;
         if (servo_pos < SERVOMIN_DC) servo_pos = SERVOMIN_DC;
         // ledcWrite(JAW_PIN, servo_pos);
        
         // Serial.printf("after min/max %d\n", servo_pos);
          // move the servo - but only if there are enough pulse widths to be > 1 degree (else jitter happens)
          if (localAbs(servo_pos-prev_servo_pos) > PULSE_WIDTHS_PER_DEGREE ) { 
            ledcWrite(JAW_PIN, servo_pos);
#ifdef SOUND_DEBUG           
            Serial.printf("final servo pos %d\n", servo_pos);
            prev_servo_pos = servo_pos;
         }else {
           Serial.printf("didn't write. Prev servo pos %d servo pos %d\n",prev_servo_pos, servo_pos);
 
#endif  
         } //  if (localAbs(servo_pos-prev_servo_pos) > PULSE_WIDTHS_PER_DEGREE )           
       } //if > noise floor
       else {
#ifdef SOUND_DEBUG        
         Serial.printf("jaw = %d\n", jawInfo[i]);
#endif    
        ledcWrite(JAW_PIN, JAW_CLOSED);
       } //else
      } // if 10hz loop
    } // else if player is playing

    /* This is the only place (other than startup) where a blocking timer is OK*/
    /* Since this is the area where the voice stuff starts, an interrupt won't hurt anything*/
    if (rfReceiver.available()) {
      decoded = rfReceiver.getReceivedValue();
      switch (decoded) {   //translate fob codes 
        case 0x4C8568:
          // happy halloween
          audioADC.resetfilter();
          Serial.printf("button 1\n");
          //ADJUST GAIN HERE
          map_min = 100;
          map_max = 330;
          gain = 3;
          dfPlayer.playTrack(1);
          playerIsPlaying = true;
          ledEyes.synchEyesOn();
          ledEyes.blinkSlow();
          vTaskDelay(pdMS_TO_TICKS(100u)); // give the player 100 ms to start 
          loop_10_Hz = 5;      
          break;
        case 0x4C8564:
          // have a piece of candy...
          Serial.printf("button 2\n");
          audioADC.resetfilter();
          loop_10_Hz = 5;
          //ADJUST GAIN HERE
          map_min = 100;
          map_max = 330;
          gain = 3;
          dfPlayer.playTrack(2);
          playerIsPlaying = true;
          ledEyes.blinkMedium();
          vTaskDelay(pdMS_TO_TICKS(100u)); // give the player 100 ms to start        
          break;
        case 0x4C856C:
          // spooky scary skeletons
          Serial.printf("button 3\n");
          audioADC.resetfilter();
          loop_10_Hz = 5;
          //ADJUST GAIN HERE
          map_min = 100;
          map_max = 330; //450;
          gain = 3; //2;
          dfPlayer.playTrack(3);
          playerIsPlaying = true;
          ledEyes.alternateEyesOn();
          ledEyes.blinkFast();
          vTaskDelay(pdMS_TO_TICKS(100u)); // give the player 100 ms to start        
          break;
        case 0x4C8562:
          // have a holly jolly xmas
          Serial.printf("button 4\n");
          audioADC.resetfilter();
          loop_10_Hz = 5;
          //ADJUST GAIN HERE
          map_min = 100;
          map_max = 650;
          audioADC.resetfilter();
          gain = 3;
          //Serial.printf("## code: 0x%X bit length: %d\n", 
          //rfReceiver.getReceivedValue(), rfReceiver.getReceivedBitlength() );
          dfPlayer.playTrack(4);
          playerIsPlaying = true;
          ledEyes.bothEyesOn();
          vTaskDelay(pdMS_TO_TICKS(100u)); // give the player 100 ms to start        
          break;
        case 0x4C856A:
          //Blinky sings Happy Birfday
          Serial.printf("button 5\n");
          loop_10_Hz = 5;

          //ADJUST GAIN HERE
          audioADC.resetfilter();
          map_min = 80;
          map_max = 450;
          gain = 4;
          //Serial.printf("## code: 0x%X bit length: %d\n", 
          //rfReceiver.getReceivedValue(), rfReceiver.getReceivedBitlength() ); 
          dfPlayer.playTrack(5);
          playerIsPlaying = true;
          ledEyes.alternateEyesOn();
          ledEyes.blinkFast();
          vTaskDelay(pdMS_TO_TICKS(100u)); // give the player 100 ms to start        
          break;         
        case 0x4C8566:
          Serial.printf("button 6\n");
          audioADC.resetfilter();
          loop_10_Hz = 5;
          //No sound here. Just 20 seconds of silence
          map_min = NOISE_FLOOR;
          map_max = NOISE_FLOOR;
          gain = 1;
          dfPlayer.playTrack(6); //this is just 20 seconds of silence for debug purposes
          playerIsPlaying = true;
          ledEyes.bothEyesOn();
          vTaskDelay(pdMS_TO_TICKS(100u)); // give the player 100 ms to start        
          break;         
        case 0x4C856E:
          // Empty space
          Serial.printf("button 7\n");
          loop_10_Hz = 5;
          audioADC.resetfilter();
          //ADJUST GAIN HERE
          map_min = NOISE_FLOOR;
          map_max = NOISE_FLOOR;
          audioADC.resetfilter();
          gain = 1;
          //Serial.printf("## code: 0x%X bit length: %d\n", 
          //rfReceiver.getReceivedValue(), rfReceiver.getReceivedBitlength() );
          //dfPlayer.playTrack(7);
          playerIsPlaying = true;
          ledEyes.bothEyesOn();
          vTaskDelay(pdMS_TO_TICKS(100u)); // give the player 100 ms to start        
          break;
        case 0x4C8561:
          Serial.printf("button 8\n");
          loop_10_Hz = 5;
          audioADC.resetfilter();
          gain = 1;
          dfPlayer.stopPlaying();
          playerIsPlaying = true;
          ledEyes.eyesOff();
          vTaskDelay(pdMS_TO_TICKS(100u)); // give the player 100 ms to stop
#ifdef SOUND_DEBUG
          Serial.printf("JAW MAX = %d, JAW MIN = %d total %d count %d mean %f\n", jaw_max, jaw_min, jaw_total, jaw_count, ((double)jaw_total/(double)jaw_count)); 
          jaw_max = 0;
          jaw_min = 10000;
          jaw_count = 0;
          jaw_total = 0;  
#endif               
          break;
        default:
          Serial.printf("Unknown signal: 0x%X bit length: %d\n", 
          rfReceiver.getReceivedValue(), rfReceiver.getReceivedBitlength() );
          break;
      } // end of switch
      rfReceiver.resetAvailable();    
    } //if receiver

  } //if (fifty_hz_timer_flag)

  vTaskDelay(1); //delay for one clock tick to yield processing for other threads
}  //loop
