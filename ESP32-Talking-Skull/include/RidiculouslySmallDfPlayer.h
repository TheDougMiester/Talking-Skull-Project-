/*
  RidiculouslySmallDfPlayer - Ridiculously small Arduino libary for
  playing DFPlayer files. Program is highly limited in functionality, 
  but is easily extended (provided you read the DFPLAYER documentation)
  It assumes an SD card and that all tracks are in SD_ROOT/MP3.
  All tracks must be numbered with a 4 digit number, i.e.
  track 1 is "0001.mpg"

  Copyright (c) 2025 Torai Madjid. All right reserved.
  Other contributors:
  Doug Brann
  
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
#ifndef __RIDICULOUSLY_SMALL_DF_PLAYER_H__
#define __RIDICULOUSLY_SMALL_DF_PLAYER_H__
#define DF_W_use_sd 2
#define DF_W_max_vol 31
#define DF_W_dac_on 0
#define DF_W_dac_off 1

#define DF_reply_yes 1
#define DF_reply_no 0

#define DF_CMD_reset 0x0c
#define DF_CMD_play  0x0d
#define DF_CMD_stop  0x16
#define DF_CMD_pause 0x0e
#define DF_CMD_vol0  0x06 // zero volume, wParam is 0-31 to set volume
#define DF_CMD_sd    0x09 // to use SD, wParam = 2
#define DF_CMD_song0 0x12 // wParam is 1-2999 for song number
#define DF_CMD_DACen 0x1A // wParam = 1 turns it off; 0 on. Default is on
#define DF_CMD_query 0x42 // query status (sends reply)

class RidiculouslySmallDfPlayer {
  private:
    uint8_t busyPin;
    Stream* DFSerial;
    uint8_t cmd6_buf[10] = {0x7E, 0xFF, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x00 ,0xEF}; //default
    //uint8_t  cmd6_buf[10] = {0x7E, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x01, 0xFF ,0xE6 ,0xEF}; //play #1

    void dfplayer_cmd6(uint8_t cmd, uint8_t reply, uint16_t wParam) { //handles 4 byte commands only
      /* 6 byte commands, which seems to be all of them...
      * cmd from defines above, 
      * reply = 0 for no response, 1 to respond,
      * wParam = 0 if no other specific value (e.g. song nr, volume)
      * poor man's assembler language.. Code is for little endian (atmega)
      */

      uint8_t* p = (uint8_t *) cmd6_buf + 1;
      uint8_t* q = (uint8_t *) &wParam;
      uint16_t     ccrc = 0xffff;
                  ccrc-= *p; p++;
                  ccrc-= *p; p++;
      *p = cmd;    ccrc-= *p; p++;
      *p = reply;  ccrc-= *p; p++;
      *p = *(q + 1); ccrc-= *p; p++; //little endian!
      *p = *q;     ccrc-= *p;   
      ccrc+=  1;

      p = (uint8_t *) cmd6_buf + 7;
      q = (uint8_t *) &ccrc;
      //Serial.print(ccrc,HEX);
      *p = *(q + 1); p++;
      *p = *q;
      DFSerial->write(cmd6_buf,10);  
    };

  public:

    void begin(Stream &l_DFSerial, uint8_t l_busyPin) {
      DFSerial = &l_DFSerial;
      busyPin = l_busyPin;
      pinMode(busyPin, INPUT_PULLUP);
    };
    void playTrack(uint8_t trackNum) {
      dfplayer_cmd6(DF_CMD_song0, DF_reply_yes,trackNum );
    };
    void stopPlaying(){
      dfplayer_cmd6(DF_CMD_stop, DF_reply_no, 0);
    };
    void resetPlayer() {
      dfplayer_cmd6(DF_CMD_reset, DF_reply_no, 0);
    };
    void setVolume(uint8_t vol) {
      dfplayer_cmd6(DF_CMD_vol0, DF_reply_yes, vol);
    };
    bool isBusy() {
      //upside down logic here, if pin is high, player not busy.
      if(digitalRead(busyPin) == 1) {
        return false;
      }
      return true;
    };
    RidiculouslySmallDfPlayer(){};
	  ~RidiculouslySmallDfPlayer(){};

};
#endif //#define __RIDICULOUSLY_SMALL_DF_PLAYER_H__
