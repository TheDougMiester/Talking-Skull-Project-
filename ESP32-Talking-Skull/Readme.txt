This library, esp32-rmt-pwm-decoder, is fairly simple, but wouldn’t have been possible without the work done by Suat Özgür and his contributors on the RCSwitch project (ttps://github.com/sui77/rc-switch/). This library also would not be possible without the excellent work done by Wolfgang Schmieder in his development of RcSwitchReceiver (https://github.com/dac1e/RcSwitchReceiver/). I’m especially grateful to Herr Schmieder for his quick response to my concerns while he was developing his library. Thanks again, sir.

The purpose of this library is to decode a 315MHz or 433MHz remote control with a minimal amount of overhead -- but not so minimal that it couldn’t be easily modified. Since I plan to use this library with a talking, arm-moving, skeleton – and do it all on one chip, I needed to decode the values and interfere with any other operations on the chip, else I would have just used RcSwitchReceiver (I don’t mean to cast any shade on RcSwitchReceiver, it’s brilliant and I shamelessly borrowed from it to do what I’ve done).

As written, this library supports only the PT2262 protocol, but is easily modified to handle any protocol in the RcSwitchReceiver library (and probably any IR receiver, but your mileage may vary). I think you’ll only need to modify one line in the library (the template in the .h file) for this to work for another protocol.

In the header file, near the bottom, you’ll find a line that looks a like this:

static makeTimingSpec< 1, 450, 20,   1,   31,    1,  3,    3,  1, false>   rxProtocol; //PT2262

the second field (450) is the base pulse width in microseconds – it’s the average pulse width for my 4-button key fob. I have an 8-button remote with an average pulse width of 350. If I want to use that instead, I can change the 350 to 450. If I want to use them both at the same time, I’d open up the error window a bit wider and change the 20 to 30 or so.

How do you find out your average pulse width to stick in there? Well, either use a $USD20 logic analyzer and PulseView software (free) to look at the pulses, or you can use the example in RcSwitchReceiver called LearnRemoteControl.ino. Another alternative is to use AdvanceReceiveDemo in the rc-switch library. I (unnecessarily) used all three, but the point of the exercise is to know your average pulse width and use their tools to figure out the protocol being used by your remote. Either RcSwitchReceiver or AdvanceReceiveDemo will tell you which protocol you have, if you don’t already know it. I duplicated the format in RcSwitchReceiver, so it’s a simple matter of replacing the PT2262 protocol I have with whatever you’re using.

My hardware:

1. ESP32-S3-DEVKITC-1-N8R2 (WROOM-1); about $USD15 from Digikey
2. 3DMakerWorld Adafruit Keyfob 4 Button RF Remote Control – 315MHz; about $USD10 off eBay
3. 315MHz transmitter/receiver kit; about $USD5 off eBay
4. A breadboard.

I wired everything together using wires taken from an ethernet cable and kept them as short as possible to keep the EMF to a minimum; those receivers are cheap and kind of noisy. I recommend adding an antenna to the receiver. I’m presently powering the receiver off the 5V and GND pins on the ESP32, but that will change when I put the final system together. 




