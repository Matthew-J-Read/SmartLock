
/*----------------------------------------------------------------------%
%                     GNU GENERAL PUBLIC LICENSE                        %  
%                       Version 3, 29 June 2007                         %
%                                                                       %
%                  Smart Lock Dissertation Project                      %
%                                                                       %
%           https://github.com/Matthew-J-Read/SmartLock.git             %
%                  Copyright © 2020  Matthew J Read                     %
% ----------------------------------------------------------------------%


// All copyrights included below are for sections of code that I have used in this project // 


 /********************************************************************************************* Copyright for code used in my listenToSecretKnock() 
 * Henrik Ekblad <henrik.ekblad@mysensors.org>                                                    and validateKnock() functions, lines 846 & 877
 * Copyright (C) 2013-2019 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 *
 * DESCRIPTION
 *
 * Secret Knock Sensor
 * http://www.mysensors.org/build/knock
 *
 * See original instructions here (note: The MySensors adopted code might differ in wiring.
 * The instructions below is correct):
 * https://learn.adafruit.com/secret-knock-activated-drawer-lock/
 * Version 13.10.31  Built with Arduino IDE 1.0.5
 *
 * By Steve Hoefer http://grathio.com
 * Adapted to MySensors by Henrik Ekblad
 *
 * Licensed under Creative Commons Attribution-Noncommercial-Share Alike 3.0
 * http://creativecommons.org/licenses/by-nc-sa/3.0/us/
 * (In short: Do what you want, as long as you credit me, don't relicense it, 
 * and don't sell it or use it in anything you sell without contacting me.)
/********************************************************************************************* Copyright for code used in my song() function line 791
 * 
  More songs available at https://github.com/robsoncouto/arduino-songs                                            
                                              
  Robson Couto, 2019

  Every sketch here has been written by myself, although based on scores
  I found online or books I own. These scores are linked in each file when possible. 
  You can use the sketches for anything, I only kindly ask that you give credit if 
  you use these codes on a tutorial, video, example, etc.

/********************************************************************************************* Copyright for code used in my thp() function line 495
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
/********************************************************************************************* Copyright for code used in my rfid() and Enrolment() functions lines 263 & 356
Software License Agreement (BSD License)
 
Adafruit PN532 example code and libraries

Copyright (c) 2012, Adafruit Industries
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holders nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
/********************************************************************************************* 
 */

#include <SoftwareSerial.h>
#include <SD.h>
#include <PN532_SWHSU.h>
#include <PN532.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <EEPROM.h>

#define maxData 12                        // 12 bits data. start and two stop bit + paraty bit 

SoftwareSerial SWSerial( 0, 1 );        // RX0, TX1
SoftwareSerial BLE(21, 20);            // RX21, TX20

PN532_SWHSU pn532swhsu( SWSerial );  // pn532swhsu
PN532 nfc( pn532swhsu );            // PN532 config 
Adafruit_BME280 bme;               // I2C config

unsigned long previousTime = 0;  // previousTime var 
unsigned long startMillis;      // var to store milliseconds passed scince boot up
unsigned long currentMillis;   // Var to store curent milliseconds

const unsigned long MaxUnlockTime = 10000; // max solinoide on time is 10 seconds
const unsigned long eventInterval = 5000; // for the timer to send data to iphone every 5 seconds

const int threshold = 3;                // Minimum signal from the piezo to register as a knock
const int rejectValue = 25;            // If an individual knock is off by this percentage of a knock we don't unlock..
const int averageRejectValue = 15;    // If the average timing of the knocks is off by this percent we don't unlock.
const int knockFadeTime = 150;       // milliseconds we allow a knock to fade before we listen for another one. (Debounce timer.)
const int maximumKnocks = 20;       // Maximum number of knocks to listen for.
const int knockComplete = 1200;    // Longest time to wait for a knock before we assume that it's finished.

int appData;                                                                                            // var to store app data interger 
int outData[maxData] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};                                           // 12 bit array to store ble data
int secretCode[maximumKnocks] = {50, 25, 25, 50, 100, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // Initial setup: "Shave and a Hair Cut, two bits."
int knockReadings[maximumKnocks];                                                                    // fills array with delays between knocks.
int knockSensorValue = 0;                                                                           // initilise knock sensor var.       
int thpflag = 1;                                                                                   // Flag for loop in thp function
int UNLOCKamount = 0;                                                                             // var to store how many times the door has been unlocked 

uint8_t white_list1[] = {186, 226, 20, 133};             // RFID card 1 4-bit array, always on system 
uint8_t white_list2[] = {0, 0, 0, 0};                   // initalise RFID card 2 4-bit array
uint8_t white_list3[] = {0, 0, 0, 0};                  // initalise RFID card 3 4-bit array
uint8_t white_list4[] = {0, 0, 0, 0};                 // initalise RFID card 4 4-bit array
uint8_t white_list5[] = {0, 0, 0, 0};                // initalise RFID card 5 4-bit array 
uint8_t white_list1_size = sizeof(white_list1);     // find array size of RFID card 1 
uint8_t white_list2_size = sizeof(white_list2);    // find array size of RFID card 2
uint8_t white_list3_size = sizeof(white_list3);   // find array size of RFID card 3 
uint8_t white_list4_size = sizeof(white_list4);  // find array size of RFID card 4 
uint8_t white_list5_size = sizeof(white_list5); // find array size of RFID card 5 
uint8_t failAttempt = 1;                       // counter to log fail attemps
uint8_t ENROL;                                // flag to use in enrolment function
uint8_t CheckSum;                            // checksum var to store the length of incoming ble data

boolean Read_success;                      // RFID bollen var that will be true if RFID card is detected 
boolean rfid_en = false;                  // RFID enrolment flag

const bool StartBit = false;            // start bit should be 0
const bool StopBit1 = true;            // first stop bit should be 1
const bool StopBit2 = true;           // second stop bit should be 1
const bool parityBit = true;         // use even parity proticol

bool RFID = true;                  // rfid will be on on first boot
bool KNOCK = false;               // secret knock will be on off on first boot
bool ALARM = true;               // alarm will be on on first boot
bool Loop = true;               // loop flag

char charVal[] = "0000000";   //temporarily holds data 

const int knockSensor = A0;             // Piezo sensor on pin 14 / A0.
const int lockPin = 11;                // relay for solinode pin.
const int REED = 7;                   // magnetic reed sensor
const int LED = 4;                   // external door led pin
const int Pizo = 8;                 // alarm transistor 
const int buzzer = 2;              // pizo for start and conection status sounds
const int lockOperateTime = 2500; // 2.5 seconds to operate the lock solenoid before de-enigising it.

byte i = 0;                       // 8-bit var 
const byte eepromValid = 862752; // eeprom validatin bit.

void setup(void) {
  
  Serial.begin(115200);                              // 115,200 baud rate
  nfc.begin();                                      // start nfc conection
  uint32_t versiondata = nfc.getFirmwareVersion(); // retreve rfid module firmware version

  if (! versiondata) {                           // if rfid module is not found hult !!!!!
    Serial.print("PN53x Module not detected");  // debug msg "not detected"
    while (1); // Halt                         // hult as rfid might be broken or tampered with !!!!!
  }

  Serial.print("Found chip PN5"); Serial.println((versiondata >> 24) & 0xFF, HEX);  // debug version data msg
  Serial.print("Firmware ver. "); Serial.print((versiondata >> 16) & 0xFF, DEC);   // debug firmware version data msg
  Serial.print('.'); Serial.println((versiondata >> 8) & 0xFF, DEC);              // debug firmware version data msg

  nfc.SAMConfig();   // nfc config
  bme.begin();      // start BME I2C conection 
  BLE.begin(9600); // BLE serial at 9600 baud rate

  pinMode(buzzer, OUTPUT);         // pizo buzzer is an output
  pinMode(Pizo, OUTPUT);          // alarm transistor is an output
  pinMode(LED_BUILTIN, OUTPUT);  // mcu board led is output
  pinMode(lockPin, OUTPUT);     // solinoide relay is an output
  pinMode(REED, INPUT_PULLUP); // REED is digital input with internal pull-up resistor enabled 
  pinMode(LED, OUTPUT);       // LED is digital output
  digitalWrite(LED, HIGH);   // LED OFF "active low"
   
  RFID_EEPROM();           // go to EEPROM function to retreve all stroed rfid card data     
  song(false);            // play cannon in D boot music
}

void loop(void) {

  BLE.listen();                 // listen on the BLE port
  if (BLE.available() > 0) {   // if BLE data is available
    Bluetooth();              // goto BLE function
  } else {                   

    if (KNOCK == true) {   // ble data not available so check if secret knock is enabled 
      
      knockSensorValue = analogRead(knockSensor); // Listen for a knock.
      if (knockSensorValue >= threshold) {       // if the knock is above the ADC threshold of 3 
        listenToSecretKnock();                  // go to knock function to read the rest of the knocks  
      }
    }

    if (RFID == true) { // if the RFID is enabled
      rfid();          // goto rfid function
    }
    
    unsigned long currentTime = millis();               // store milliseconds passed scince boot up
    if (currentTime - previousTime >= eventInterval) { // if statment is true when 5 seconds have passed 
      thp();                                          // send data to iphone every 5 seconds
      previousTime = currentTime;                    // Update the timing for the next time
    }
  }
}

void RFID_EEPROM() {                                    // function to retreve rfid card data 
  ENROL = EEPROM.read(20);                             // read how many cards are on the system
  UNLOCKamount = EEPROM.read(21);                     // read how many door unlocks there have been 
  for (int i = 0; i < 4 ; i++) {                     // card 2 data is located at position 1 to 4 in eeprom 
    white_list2[i] =  EEPROM.read(i + 1);           // read RFID card 2 data from eeprom
  }
  for (int i = 0; i < 4 ; i++) {                  // card 3 data is located at position 5 to 8 in eeprom 
    white_list3[i] =  EEPROM.read(i + 5);        // read RFID card 3 data from eeprom
  }
  for (int i = 0; i < 4 ; i++) {               // card 4 data is located at position 9 to 12 in eeprom 
    white_list4[i] =  EEPROM.read(i + 9);     // read RFID card 4 data from eeprom
  }
  for (int i = 0; i < 4 ; i++) {            // card 5 data is located at position 13 to 16 in eeprom 
    white_list5[i] =  EEPROM.read(i + 13); // read RFID card 5 data from eeprom
  }
}

void rfid() {                          // function to detect rfid cards

  uint8_t uid[] = {0, 0, 0, 0, 0};   // Buffer to store the returned UID     
  uint8_t uid_size = sizeof(uid);   // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  int error_uid1 = false;          // initilse rfid 1 error flag
  int error_uid2 = false;         // initilse rfid 2 error flag
  int error_uid3 = false;        // initilse rfid 3 error flag
  int error_uid4 = false;       // initilse rfid 4 error flag
  int error_uid5 = false;      // initilse rfid 5 error flag

  Read_success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uid_size); // retreve MIFARE data from functions in the PN532_SWHSU.h PN532.h files

  if (Read_success) {                                     // will be true if a card has been detected
    if ((uid_size != white_list1_size)) {                // check if the size of the detected card maches card-1 stored data 
      error_uid1 = true;                                // set uid-1 error flag to true
    } else {                                           
      for (int i = 0;  i < white_list1_size; i++) {   // for loop to check data in each array position 
        if ( uid[i] != white_list1[i] ) {            // check if the data at position i of the detected card is the same as the data in card-1 array
          error_uid1 = true;                        // set uid-1 error flag to true
        }
      }   if (error_uid1 == false) {              // if uid-1 data has no errors unlock the door 
        Serial.println("RFID PASS");             // pass debug msg
        doorUnlock(lockOperateTime);            // goto the unlock function to unlock the door
      }
      else if (error_uid1 == true) {          // if there was an error check for the next card

        if ((uid_size != white_list2_size)) {               // check if the size of the detected card maches card-2 stored data 
          error_uid2 = true;                               // set uid-2 error flag to true
        } else {
          for (int i = 0;  i < white_list2_size; i++) {  // for loop to check data in each array position
            if ( uid[i] != white_list2[i] ) {           // check if the data at position i of the detected card is the same as the data in card-2 array
              error_uid2 = true;                       // set uid-2 error flag to true
            }
          }   if (error_uid2 == false) {             // if uid-2 data has no errors unlock the door 
            Serial.println("RFID PASS");            // pass debug msg
            doorUnlock(lockOperateTime);           // goto the unlock function to unlock the door
          }
          else if (error_uid2 == true) {         // if there was an error check for the next card

            if ((uid_size != white_list3_size)) {                 // check if the size of the detected card maches card-3 stored data 
              error_uid3 = true;                                 // set uid-3 error flag to true
            } else {
              for (int i = 0;  i < white_list3_size; i++) {    // for loop to check data in each array position
                if ( uid[i] != white_list3[i] ) {             // check if the data at position i of the detected card is the same as the data in card-3 array
                  error_uid3 = true;                         // set uid-3 error flag to true
                }
              }   if (error_uid3 == false) {               // if uid-3 data has no errors unlock the door 
                Serial.println("RFID PASS");              // pass debug msg
                doorUnlock(lockOperateTime);             // goto the unlock function to unlock the door
              }
              else if (error_uid3 == true) {           // if there was an error check for the next card

                if ((uid_size != white_list4_size)) {                // check if the size of the detected card maches card-4 stored data 
                  error_uid4 = true;                                // set uid-4 error flag to true
                } else {
                  for (int i = 0;  i < white_list4_size; i++) {   // for loop to check data in each array position
                    if ( uid[i] != white_list4[i] ) {            // check if the data at position i of the detected card is the same as the data in card-4 array
                      error_uid4 = true;                        // set uid-4 error flag to true
                    }
                  }   if (error_uid4 == false) {              // if uid-4 data has no errors unlock the door
                    Serial.println("RFID PASS");             // pass debug msg
                    doorUnlock(lockOperateTime);            // goto the unlock function to unlock the door
                  }
                  else if (error_uid4 == true) {          // if there was an error check for the next card

                    if ((uid_size != white_list5_size)) {                 // check if the size of the detected card maches card-5 stored data
                      error_uid5 = true;                                 // set uid-5 error flag to true
                    } else {
                      for (int i = 0;  i < white_list5_size; i++) {    // for loop to check data in each array position
                        if ( uid[i] != white_list5[i] ) {             // check if the data at position i of the detected card is the same as the data in card-5 array
                          error_uid5 = true;                         // set uid-5 error flag to true
                        }
                      }   if (error_uid5 == false) {               // if uid-5 data has no errors unlock the door
                        Serial.println("RFID PASS");              // pass debug msg
                        doorUnlock(lockOperateTime);             // goto the unlock function to unlock the door
                      }
                      else if (error_uid5 == true) {           // if there was an error after checking all data
                        Fail();                               // goto the fail function do sound the fail alert tone
                      }
                    }
                  }
                }
              }
            }
          }
        }

      }
    }

  }
}

void Enrolment() {                              // RFID Enrolment functon to add user card to the system  
  uint8_t white_list[] = {0, 0, 0, 0};         // initilise array to temporarily store the card data
  uint8_t i = 1;                              // var for timer (not implemented)
  boolean success;                           // RFID bollen var that will be true if RFID card is detected 
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                       // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  while (!success) {                                                                 // loop until a card is detected
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength); // retreve MIFARE data from functions in the PN532_SWHSU.h PN532.h files
    if (success) {                                                                 // wait for a card detection
      
      for (uint8_t a = 0; a < uidLength; a++)       // for loop to store data at position a in the array 
      {
        Serial.print(" "); Serial.print(uid[a]);  // uid data debug msg
        white_list[a] = uid[a];                  // temporarily store data in white list array for all position increments
      }
      ENROL++;                                 // increment counter so we know how many users are enroled on the system
      EEPROM.write(20, ENROL);                // write counter value to eeprom so not to forget if power is lost 
    }
    else       
    {
      delay(1000);                        // pause for 1 second befor looping back             
      Serial.println("Timed out");       // timeout debug msg
      i++;                              // increment timeout counter (not implemented yet)
    }
  }
  
  tone(Pizo, 700); // play a 700 Hz tone (card detection noise for sucsesfull enrollment) 
  delay(300);     // pause for 0.3 seconds
  noTone(Pizo);  // turn off buzzer

  switch (ENROL) {
    case 2:                                          // case statment 2
      for (uint8_t a = 0; a < 4; a++)               // for loop to increment through array positions
      {
        Serial.print(" "); Serial.print(uid[a]);  // uid debug msg
        white_list2[a] = white_list[a];          // move temp data to perminant card 2 array 
        EEPROM.write(a + 1, white_list[a]);     // write card 2 to EEPROM so not to forget if power is lost 
      }
      break;                                  // break statment
      
    case 3:                                          // case statment 3
      for (uint8_t a = 0; a < 4; a++)               // for loop to increment through array positions
      {
        Serial.print(" "); Serial.print(uid[a]);  // uid debug msg
        white_list3[a] = white_list[a];          // move temp data to perminant card 3 array
        EEPROM.write(a + 5, white_list[a]);     // write card 3 to EEPROM so not to forget if power is lost
      }
      break;                                  // break statment
      
    case 4:                                          // case statment 4
      for (uint8_t a = 0; a < 4; a++)               // for loop to increment through array positions
      {
        Serial.print(" "); Serial.print(uid[a]);  // uid debug msg
        white_list4[a] = white_list[a];          // move temp data to perminant card 4 array
        EEPROM.write(a + 9, white_list[a]);     // write card 4 to EEPROM so not to forget if power is lost      
      }
      break;                                  // break statment

    case 5:                                           // case statment 5
      for (uint8_t a = 0; a < 4; a++)                // for loop to increment through array positions
      {
        Serial.print(" "); Serial.print(uid[a]);   // uid debug msg
        white_list5[a] = white_list[a];           // move temp data to perminant card 5 array
        EEPROM.write(a + 13, white_list[a]);     // write card 5 to EEPROM so not to forget if power is lost 
      }
      break;                                   // break statment
                                             
    default:                                        // defult statment
      Serial.println("Max enrolment is 5");        // Max enrolment debug msg 
      ENROL = 1;                                  // restet enrollment varible
      EEPROM.write(20, ENROL);                   // write counter value to eeprom so not to forget if power is lost 
      Fail();                                   // goto fail function so the user knows there was a problem with enrollment
      break;                                   // break statment
  }
  delay(1000);                               // pause for 1 second to avoid door unlocking after inrollment as the rfid reader is very fast at reading
}

void DELETE() {                        // function to delete user RFID cards off the system

  for (uint8_t a = 1; a < 17; a++)   // for loop to wipe eeprom data from 1 to 16 bits of data (2-Bytes)
  {
    EEPROM.write(a, 0);            // wipe 2-Bytes of rfid eeprom data by zeroing it
  }
}

void Bluetooth(void) {

  appData = BLE.read();               // store BLE string data in temp var   
  if (appData == 48) {               // if incoming data is the string "0" ie ASCII value of 48
    outData[i] = false;             // turn the string "0" to logic 0
    i++;                           // increment counter
  }
  else if (appData == 49) {      // if incoming data is the string "1" ie ASCII value of 49
    outData[i] = true;          // turn the string "1" to logic 1
    i++;                       // increment counter
  }
  else if (appData > 1) {    // if data is not 1 or 0 it must me a conection state msg 
    i = i;                  // ignore the conection state msgs
    thpflag++;             // increment conection msg counter
    if (thpflag == 7) {   // wait for con+OK OR CON+LOST to finish
      thp();             // send other the firt data pack to the iphone
      song(true);       // goto song function to play conection melody
    }
  }

  if ((i == maxData) && (sizeof(outData) / sizeof(int))) { // finish storing the data if counter has reached 12-bits and check-sum is correct 
    i = 0;                                                // reset counter for next BLE recived msg
    DataCheck();                                         // goto data check function to valididate the data
  } else if (i > maxData) {                             // if statment for debug
    Serial.println("i > max");                         // counter state debug msg
  }
}

void DataCheck(void) {                          //  data check function

  uint8_t sum = 0;                            // initalise var to 0
  for (uint8_t a = 0; a < maxData; a++ ) {   // for loop to increment through array positions
    Serial.print(outData[a]);               // data debug msg
    sum = sum + outData[a];                // sum + data at position a
  } if (sum % 2 == 0) {                   // if remander = 0
    
    Serial.print(" CheckSum = 12, 1 start bit, ");              // data debug msg          
    Serial.print(" 8 bits of data, 1 parity bit, ");           // data debug msg
    Serial.print(" and 2 stop bits. (even parity protocol)"); // data debug msg

    // data is in the valid for smart lock format if the first bit of data is 0 and the last to stop bits of the data are 1
    // the iphone aplication deals with the parity
    if ((outData[0] == false) && (outData[maxData - 2] == true) && (outData[maxData - 1] == true)) {

      Serial.println("Data Ok");  // valid data format debug msg
      DataCheck2();              // goto second stage in data checking function 
      
    } else {
      Serial.println("Start-Stop Bit Errors"); // Start-Stop Bit format debug error msg
    }
  } else {
    Serial.println("Even Parity Error");    // data parity debug error msg
  }
}

void thp() {  // temp, humid, press function for enviroment readings

  thpflag = 1;                                   // initalise flag to 1
  float Temp = bme.readTemperature();           // read and store Temperature data reading
  float Press = (bme.readPressure() / 100.0F); // read and store Pressure data reading and convert to hPa  
  float Humid = bme.readHumidity();           // read and store Humidity data reading

  Serial.print("Temperature = ");           // Temperature debug msg 
  Serial.print(Temp);                      // data debug msg 
  Serial.println(" *C");                  // data type debug msg 
  dtostrf(Temp, 4, 0, charVal);          // converts a float into  character array for BLE transmision
  BLE.write(charVal);                   // send Temperature data to BLE for transmision
  BLE.write(" °C");                    // send Temperature system type to BLE for transmision
  delay(20);                          // serial buffer acts like hardwear buffer so wait 20 ms untill it fills to max size 

  Serial.print("Humidity = ");      // Humidity debug msg
  Serial.print(Humid);             // data debug msg
  Serial.println(" %");           // data type debug msg
  dtostrf(Humid, 4, 0, charVal); // converts a float into  character array for BLE transmision
  BLE.write(charVal);           // send Humidity data to BLE for transmision
  BLE.write(" %");             // send % string to BLE for transmision
  delay(20);                  // serial buffer acts like hardwear buffer so wait 20 ms untill it fills to max size 

  Serial.print("Pressure = ");        // Pressure debug msg
  Serial.print(Press);               // data debug msg
  Serial.println(" hPa");           // data type debug msg
  dtostrf(Press, 4, 0, charVal);   // converts a float into  character array for BLE transmision
  BLE.write(charVal);             // send Pressure data to BLE for transmision
  BLE.write(" hPa");             // send Pressure system type to BLE for transmision
  delay(20);                    // serial buffer acts like hardwear buffer so wait 20 ms untill it fills to max size

  dtostrf(UNLOCKamount, 4, 0, charVal);  // converts a float into  character array for BLE transmision
  BLE.write(charVal);                   // send door unlock counter amount to BLE for transmision
  delay(20);                           // serial buffer acts like hardwear buffer so wait 20 ms untill it fills to max size
  Vbatt();                            // go to the battery state function
}

bool magnet(void) {          // REED sensor function        
  if (digitalRead(REED)) {  // if REED switch is closed ie TRUE, the door is closed
    return true;           // return TRUE
  } else {                // else the REED switch is open ie FALSE, the door is open
    return false;        // return FALSE
  }
}

void doorUnlock(const int lockOperateTime) {     // function to unlocks the door.
  UNLOCKamount++;                               // increment door unlock amount timer
  EEPROM.write(21, UNLOCKamount);              // write counter value to eeprom so not to forget if power is lost 
  failAttempt = 0;                            // reset the fail attempt counter to 0
  startMillis = millis();                    // store how many miliseconds have passed since boot up
  digitalWrite(LED_BUILTIN, HIGH);          // activate mcu led for debug
  digitalWrite(LED, LOW);                  // activate external led (active-low) for user to see 
  digitalWrite(lockPin, HIGH);            // activate relay to engage the solinoide to unlock the door
  delay(lockOperateTime);                // delay for 2.5 seconds
                                        
  if (magnet() == true) {              // if REED switch is closed ie TRUE, the door is closed
    digitalWrite(lockPin, LOW);       // so the relay can now be disactervated to de-engage the solanoide
    digitalWrite(LED_BUILTIN, LOW);  // de-activate mcu led for debug
    digitalWrite(LED, HIGH);        // de-activate external led (active-low) for user to see
  } else {                         // else if the the REED switch is open ie FALSE, the door is still open 
                                  // if 2.5 seconds have passed and the door is still open

    while ((MaxUnlocktime() != true) && (magnet() != true)) { // loop untill the door is closed or 10 seconds have passed by until de-engaring the solenoide 
      Serial.println("Do nothing");    // debug msg
    }
    digitalWrite(lockPin, LOW);      // de-engage the solanoide as ether 10 seconds have passed or the door closed 
    digitalWrite(LED_BUILTIN, LOW); // de-activate mcu led for debug 
    digitalWrite(LED, HIGH);       // de-activate external led (active-low) for user to see
  }
}

bool MaxUnlocktime(void) {     // timer function for max time the solenoide can be activated 
  
  currentMillis = millis();                           // store milliseconds passed scince boot up
  if (currentMillis - startMillis >= MaxUnlockTime)  // find whether the period has elapsed
  {
    startMillis = currentMillis;                   // Update the timing for the next time
    return true;                                  // return TRUE
  } else {
    return false;                               // else return FALSE
  }
}

void Vbatt() {                                               // MCU's ADC Voltage monitoring function 
  
  int sensorValue = analogRead(A1);                        // read the input on ADC analog pin A0 pin 14:
  float voltage = sensorValue * (3.0 / 1023.0) + 1;       // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 3V). +1 is for config
  Serial.println(voltage);                               // Voltage data debug msg

  if (voltage <= 5.0) {                                 // if voltage is less or equal to 5V 
    voltage = 0;                                       // batt @ 0 % charge
  } else if (voltage <= 5.4) {                        // if voltage is less or equal to 5.4V 
    voltage = 10;                                    // batt @ 10 % charge
  } else if (voltage <= 5.58) {                     // if voltage is less or equal to 5.58V 
    voltage = 20;                                  // batt @ 20 % charge
  } else if (voltage <= 6.5) {                    // if voltage is less or equal to 6.5V 
    voltage = 30;                                // batt @ 30 % charge
  } else if (voltage <= 6.9) {                  // if voltage is less or equal to 6.9V 
    voltage = 40;                              // batt @ 40 % charge
  } else if (voltage <= 7.06) {               // if voltage is less or equal to 7.06V 
    voltage = 50;                            // batt @ 50 % charge
  } else if (voltage <=  8.20) {            // if voltage is less or equal to 8.20V 
    voltage = 60;                          // batt @ 60 % charge
  } else if (voltage <=  9.32) {          // if voltage is less or equal to 9.32V 
    voltage = 70;                        // batt @ 70 % charge
  } else if (voltage <= 10.42) {        // if voltage is less or equal to 10.42V 
    voltage = 80;                      // batt @ 80 % charge
  } else if (voltage <= 11.5) {       // if voltage is less or equal to 11.5V 
    voltage = 90;                    // batt @ 90 % charge
  } else if (voltage <= 12.0) {     // if voltage is less or equal to 12.0V 
    voltage = 100;                 // batt @ 100 % charge
  } else if (voltage >= 12.6) {   // if voltage is greater or equal to 12.6V 
    voltage = 100;               // batt @ 100 % charge
  }


  dtostrf(voltage, 4, 0, charVal); // converts a float into  character array for BLE transmision
  BLE.write(charVal);             // send data to BLE for transmission
  BLE.write(" %");               // send string "%" to BLE for transmission
  delay(20);                    // serial buffer acts like hardwear buffer so wait 20 ms untill it fills to max size 
}

void alarm() {                         // function to play alarm sound
  failAttempt = 0;                    // reset fail attempt counter
  if (ALARM == true) {               // If the use has enabled the alarm
    for (int i = 0; i < 10; i++) {  // loop 10 times 0 to 9
      tone(Pizo, 2200);            // play 2.2 KHz tone
      delay(200);                 // pause 0.2 seconds
      noTone(Pizo);              // turn off buzzer
      tone(Pizo, 2000);         // play 2 KHz tone
      delay(200);              // pause 0.2 seconds
      noTone(Pizo);           // turn off buzzer 
    }
  }
}

void Fail() {       
  if (ALARM == true) {                        // If the use has enabled the alarm
    for (int i = 0; i < 3; i++) {            // loop 3 times 0 to 2
      tone(Pizo, 1000);                     // play 1 KHz tone
      digitalWrite(LED_BUILTIN, HIGH);     // activate mcu led for debug
      digitalWrite(LED, LOW);             // activate external led (active-low) for user to see
      delay(30);                         // pause 30 miliseconds
      digitalWrite(LED_BUILTIN, LOW);   // de-activate mcu led for debug
      digitalWrite(LED, HIGH);         // de-activate external led (active-low) for user to see
      noTone(Pizo);                   // turn off buzzer 
      delay(100);                    // pause 0.1 seconds
    }
    if (failAttempt == 3) {        // has there been 3 failed attempts 
      alarm();                    // if there have, goto alarm function
    }
    failAttempt++;              // increment the failed attempts counter
    delay(1000);               // delay 1 second to avoid another faile read as the RFID module is very fast at reads
  }
}

void DataCheck2(void) {      // 2nd data check function to detumin what the incoming bluetooth data means

  bool dataError = false;  // initalise data error to false

  int UL[maxData]    = {0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1}; int RFON[maxData]  = {0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1};      // allowed data packets 
  int RFOFF[maxData] = {0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1}; int KNON[maxData]  = {0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1};     // allowed data packets
  int KNOFF[maxData] = {0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1}; int ALON[maxData]  = {0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1};    // allowed data packets
  int ALOFF[maxData] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1}; int ENRO[maxData]  = {0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1};   // allowed data packets
  int DELL[maxData]  = {0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1};                                                             // allowed data packets

  for (int i = 0;  i < maxData; i++)    // nested loops to check the recived bluetooth data maches the above allowed data packets
  {
    if ( UL[i] != outData[i] )       // cheack that both data values at possition i equate to eachother
    {
      dataError = true;            // make error flag true
    }
  }
  if (dataError == true) {                           // is the error flag true
    dataError = false;                              // make error flag false
    for (int i = 0;  i < maxData; i++)             // for loop to iterate through array positions for data validation
    {
      if ( RFON[i] != outData[i] )               // cheack that both data values at possition i equate to eachother
      {
        dataError = true;                      // make error flag true
      }
    }
    if (dataError == true) {                // is the error flag true
      dataError = false;                   // make error flag false
      for (int i = 0;  i < maxData; i++)  // for loop to iterate through array positions for data validation
      {
        if ( RFOFF[i] != outData[i] )  // cheack that both data values at possition i equate to eachother
        {
          dataError = true;         // make error flag true
        }
      }
      if (dataError == true) {  // is the error flag true
        dataError = false;                              // make error flag false
        for (int i = 0;  i < maxData; i++)             // for loop to iterate through array positions for data validation
        {
          if ( KNON[i] != outData[i] )               // cheack that both data values at possition i equate to eachother
          {
            dataError = true;                      // make error flag true
          }
        }
        if (dataError == true) {                // is the error flag true
          dataError = false;                   // make error flag false
          for (int i = 0;  i < maxData; i++)  // for loop to iterate through array positions for data validation
          {
            if ( KNOFF[i] != outData[i] )   // cheack that both data values at possition i equate to eachother
            {
              dataError = true;          // make error flag true
            }
          }
          if (dataError == true) {    // is the error flag true
            dataError = false;                               // make error flag false
            for (int i = 0;  i < maxData; i++)              // for loop to iterate through array positions for data validation
            {
              if ( ALON[i] != outData[i] )                // cheack that both data values at possition i equate to eachother
              {
                dataError = true;                       // make error flag true
              }
            }
            if (dataError == true) {                // is the error flag true
              dataError = false;                   // make error flag false
              for (int i = 0;  i < maxData; i++)  // for loop to iterate through array positions for data validation
              {
                if ( ALOFF[i] != outData[i] )  // cheack that both data values at possition i equate to eachother
                {
                  dataError = true;          // make error flag true
                }
              }
              if (dataError == true) {   // is the error flag true
                dataError = false;                             // make error flag false
                for (int i = 0;  i < maxData; i++)            // for loop to iterate through array positions for data validation
                {
                  if ( ENRO[i] != outData[i] )              // cheack that both data values at possition i equate to eachother
                  {
                    dataError = true;                     // make error flag true
                  }
                }
                if (dataError == true) {                // is the error flag true
                  dataError = false;                   // make error flag false
                  for (int i = 0;  i < maxData; i++)  // for loop to iterate through array positions for data validation
                  {
                    if ( DELL[i] != outData[i] )    // cheack that both data values at possition i equate to eachother
                    {
                      dataError = true;           // make error flag true
                    }
                  }
                  if (dataError == true) {  // is the error flag true

                  } else {
                    Serial.println("DELETE RFID"); // debug msg
                    DELETE();                     // goto delete function to delete all rfid cards
                  }

                } else {
                  Serial.println("ENROl RFID"); // debug msg
                  Enrolment();                 // goto Enrolment function to enrol a user rfid card
                }

              } else {
                Serial.println("ALARM OFF"); // debug msg
                ALARM = false;              // user has disabled the alarm
              }

            } else {
              Serial.println("ALARM ON");  // debug msg
              ALARM = true;               // user has enabled the alarm
            }

          } else {
            Serial.println("KNOCK OFF");  // debug msg
            KNOCK = false;               // when RFID on secret knock has to be disabled
            RFID = true;                // user has enabled RFID
          }

        } else {
          Serial.println("KNOCK ON"); // debug msg
          KNOCK = true;              // user has enabled secret knock
          RFID = false;             // when secret knock on RFID has to be disabled
        }

      } else {
        Serial.println("RFID OFF");  // debug msg
        RFID = false;               // user has disabled RFID
      }

    } else {
      Serial.println("RFID ON"); // debug msg
      RFID = true;              // user has enabled RFID
    }

  } else {
    Serial.println("UNLOCKED");   // debug msg
    doorUnlock(lockOperateTime); // unlock the door for 2.5 seconds
  }

}

void song(bool var) {  // song function to play boot and conection status melodys

  int melody1[] = { 659, 8, 659, 8, 0, 8, 659, 8, 0, 8, 0, 8, 659, 8, 784, 4, 0, 4, 392, 8, 0, 4,};    // array of frequency data in Hz, conection status melody 
  int melody2[] = { 1175, 4, 1480, 8, 1568, 8, 1760, 4, 1480, 8, 1568, 8, 1760, 4, 988, 8, 1109, 8,   // array of frequency data in Hz, boot melody
  1175, 8, 1319, 8, 1480, 8, 1568, 8,1480, 4, 1175, 8, 1319, 8, 1480, 4, 740, 8, 784, 8, 880, 8, 784, 
  8, 740, 8, 784, 8, 880, 2, 784, 4, 988, 8, 880, 8, 784, 4, 740, 8, 659, 8, 740, 4, 587, 8, 659, 8, 
  740, 8, 784, 8, 880, 8, 988, 8, 784, 4, 988, 8, 880, 8, 988, 4, 1109, 8, 1175, 8, 880, 8, 988, 8, 
  1109, 8, 1175, 8, 1319, 8, 1480, 8, 1568, 8, 1760, 2,};                                           

  int tempo = 200;                                          // use a tempo of 200 BPM 
  int notes1 = sizeof(melody1) / sizeof(melody1[0]) / 2;   // sizeof to get the number of bytes, each note value is composed of two bytes
  int notes2 = sizeof(melody2) / sizeof(melody2[0]) / 2;  // two values per note pitch and duration, each note there are four bits
  int wholenote = (60000 * 4) / tempo;                   // find the duration of a note in miliseconds
  int divider = 0;                                      // initalise var
  int noteDuration = 0;                                // initalise var

  switch (var) {  // switch statment to play the boot melody of the bluetooth conection state melody
    case true:   // if true play the conection status melody
    
      for (int thisNote = 0; thisNote < notes1 * 2; thisNote = thisNote + 2) { // for loop to play all notes and durations

        divider = melody1[thisNote + 1];                  // calculate the duration of each note
        if (divider > 0) {                               // regular note, just proceed
          noteDuration = (wholenote) / divider;         // calculate the duration of each note
        } else if (divider < 0) {                      // dotted notes are represented with negative durations
          noteDuration = (wholenote) / abs(divider);  // duration of a note in miliseconds divided by the absoulute of the divider
          noteDuration *= 1.5;                       // increases the duration by 1.5 (dotted note)
        }
        tone(buzzer, melody1[thisNote], noteDuration * 0.9); // only play the note for 90% of the duration, leaving 10% as a pause
        delay(noteDuration);                                // Wait for the duration before playing the next note
        noTone(buzzer);                                    // stop the tone/note for the next note to play
      }

      break;     // break statement
    case false: // if true play the boot melody

      for (int thisNote = 0; thisNote < notes2 * 2; thisNote = thisNote + 2) { // for loop to play all notes and durations

        divider = melody2[thisNote + 1];                  // calculate the duration of each note
        if (divider > 0) {                               // regular note, just proceed
          noteDuration = (wholenote) / divider;         // calculate the duration of each note
        } else if (divider < 0) {                      // dotted notes are represented with negative durations
          noteDuration = (wholenote) / abs(divider);  // duration of a note in miliseconds divided by the absoulute of the divider
          noteDuration *= 1.5;                       // increases the duration by 1.5 (dotted note)
        }
        tone(buzzer, melody2[thisNote], noteDuration * 0.9); // only play the note for 90% of the duration, leaving 10% as a pause
        delay(noteDuration);                                // Wait for the duration before playing the next note
        noTone(buzzer);                                    // stop the tone/note for the next note to play
      }
      break;     // break statement
    default:    // default statement (state not possible)
      break;   // do nothing break statement
  }
}

void listenToSecretKnock() {                  // function that records the timing of knocks                           
  for (int i = 0; i < maximumKnocks; i++) {  // for loop to fill array with zeros for initalisation
    knockReadings[i] = 0;                   // reset the listening array.
  }

  int currentKnockNumber = 0;  // initalise var for the array.
  int startTime = millis();   // milisecond reference for when knock started
  int now = millis();        // now() seconds since Jan 1 1970 = milisecond since boot 
  delay(knockFadeTime);     // pause for the peak to fade before the next ADC read

  do {                                            // listen for the next knock or wait for it to timeout.
    knockSensorValue = analogRead(knockSensor);  // read the value from the ADC
    if (knockSensorValue >= threshold) {        // got another knock...
                                               // record the delay time. 
      Serial.println("knock.");               // knock acurance debug msg 
      now = millis();                        // store miliseconds since boot
      
      knockReadings[currentKnockNumber] = now - startTime;  // find curent time                                              
      currentKnockNumber ++;                               // increment the counter
      startTime = now;                                    // reset timer for the next knock                                               
      delay(knockFadeTime);                              // small delay for the knock decay/fade-out
    }
    now = millis();                                                                      // store miliseconds since boot
  } while ((now - startTime < knockComplete) && (currentKnockNumber < maximumKnocks));  // did the timer timeout or run out of knocks
  if (validateKnock() == true) {                                                       // all knocks recorded goto function to check, if true knocks are valid
    doorUnlock(lockOperateTime);                                                      // unlock the door 
  } else {
    Serial.println("knock failed.");                                                // fail debug msg
  }
}

boolean validateKnock() {                     // function to check if knocks matches the secret knock. Will returns true if valid, false if not
  int i = 0;                                 // initalise i array position counter
  int currentKnockCount = 0;                // initalise current knock count var
  int secretKnockCount = 0;                // initalise knock count var 
  int maxKnockInterval = 0;               // initalise var that normalizes the knock times
  
  for (i = 0; i < maximumKnocks; i++) { // loop to check if the number of knocks recorded is greater than 20
    if (knockReadings[i] > 0) {        // is it greater than 0
      currentKnockCount++;            // if it is increment counter
    }
    if (secretCode[i] > 0) {        // is it greater than 0
      secretKnockCount++;          // if it is increment counter
    }
    if (knockReadings[i] > maxKnockInterval) {  // store normalization data while in loop
      maxKnockInterval = knockReadings[i];     // store knock data in var
    }
  }
  if (currentKnockCount != secretKnockCount) { // check if counters are the same 
    return false;                             // if not return FALSE and do not open the door
  }
                                           // need to account for human error in knocks as tempo and times may differ slightly
  int totaltimeDifferences = 0;           // initalise var
  int timeDiff = 0;                      // initalise var
  for (i = 0; i < maximumKnocks; i++) { // for loop for to normalize the knock times
    
    knockReadings[i] = map(knockReadings[i], 0, maxKnockInterval, 0, 100);   // normalize
    timeDiff = abs(knockReadings[i] - secretCode[i]);                       // find the time difrence
    if (timeDiff > rejectValue) {                                          // individual knock is over 25 ms out of the acseptble time
      return false;                                                       // return false, dont open door
    }
    totaltimeDifferences += timeDiff;                                   // sum the two varibles 
  }                                                              
  if (totaltimeDifferences / secretKnockCount > averageRejectValue) { // also dont open door if the whole knock sequence is too out by 15 ms
    return false;
  }
  return true; // open the door if timings are ok
}
