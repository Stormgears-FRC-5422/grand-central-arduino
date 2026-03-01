/*********************************
**Wire: 
*Pin10 - player TX; 
*Pin11 - player RX;
*pin3  - player BUSY
**Board : Uno
*By: LEFF
**********************************/
#include <SoftwareSerial.h>
#include <DFPlayer_Mini_Mp3.h>

SoftwareSerial mySerial(10, 11); // RX, TX

void setup () {
  Serial.begin (9600);
  mySerial.begin (9600);
  mp3_set_serial (mySerial);  //set softwareSerial for DFPlayer-mini mp3 module 
  delay(5000); // delay 1ms to set volume
      mp3_set_volume (2);
     delay(10);
     mp3_set_volume (2); // value 0~30
}
void loop () {   
  boolean play_state = digitalRead(3);// connect Pin3 to BUSY pin of player
  if(play_state == HIGH){
    mp3_next ();
  }
}
