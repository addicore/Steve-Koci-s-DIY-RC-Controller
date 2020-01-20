//**********************************************************************************************************************
//
//                                      Steve Koci's DIY Remote Controller
//                                        by Addicore.com & Boffintronics
//
// Revision History:
//       01-03-2020   ARM     Pre Release Version 0.5
//       01-18-2020   ARM     Initial Release Version 1.0
//
//**********************************************************************************************************************
// Copyright (C) 2020  Boffintronics, LLC
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//**********************************************************************************************************************

#include <SPI.h>
#include <EEPROM.h>
#include <nRF24L01.h>       // NRF24L01 library by TMRh20 https://github.com/TMRh20/RF24
#include <RF24.h>

//Analog inputs
const int L_SLD  = A6;     // Left Slide Pot (PCB R1)
const int L_JS_X = A1;     // Left Joystick X Pot (PCB A1)
const int L_JS_Y = A2;     // Left Joystick Y Pot (PCB A1)
const int R_JS_Y = A3;     // Right Joystick Y Pot (PCB A2)
const int R_JS_X = A7;     // Right Joystick X Pot (PCB A2)
const int R_SLD  = A0;     // Right Slide Pot  (PCB R2)

// Digital inputs
const int L_BTN1 = 9;      // Left Shoulder & Lower Buttons (PCB S8 & S4)
const int L_TGL  = 7;      // Left Toggle Switch  (PCB S2)
const int L_JS_B = 8;      // Left Joystick Button  (PCB A1)
const int L_BTN2 = A5;     // Left Center Button  (PCB S5)
const int R_BTN2 = A4;     // Right Center Button  (PCB S6)
const int R_JS_B = 5;      // Right Joystick Button  (PCB A2)
const int R_TGL  = 6;      // Right Toggle Switch  (PCB S3)
const int R_BTN1 = 10;     // Right Shoulder & Lower Buttons  (PCB S7 & S9)

const int BLUE_LED = 2;    // Blue LED
const int LED_ON = 0;      // LED on
const int LED_OFF = 1;     // LED off

const int PROG_BTN_1 = L_JS_B;    // Program mode button #1
const int PROG_BTN_2 = R_JS_B;    // Program mode button #2
const int PROG_LED = BLUE_LED;    // Program mode status LED

// NRF24L01 pipe address table
const byte pipes[][6] = {"Pipe0", "Pipe1", "Pipe2", "Pipe3", "Pipe4", "Pipe5"};

RF24 radio(3, 4);     // NRF24L01 CE,CSN pins

struct Data_Package { // NRF24L01 Data payload package
  int LsPot;
  int LjPotX;
  int LjPotY;
  int RjPotX;
  int RjPotY;
  int RsPot;
  byte Switches;
};
Data_Package data;

int Program = false;
int NRFpipeIndex = 0;

void (* ResetFunct)(void) = 0;

//**********************************************************************************************************************
void setup() {
  pinMode(L_TGL, INPUT_PULLUP);
  pinMode(R_TGL, INPUT_PULLUP);

  pinMode(L_BTN1, INPUT_PULLUP);
  pinMode(L_BTN2, INPUT_PULLUP);
  pinMode(R_BTN1, INPUT_PULLUP);
  pinMode(R_BTN2, INPUT_PULLUP);

  pinMode(L_JS_B, INPUT_PULLUP);
  pinMode(R_JS_B, INPUT_PULLUP);

  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, LED_OFF);    // Turn blue led off


  Serial.begin(19200);  // Initialize serial communications with PC
  Serial.println("----------------------------------");
  Serial.println("Steve Koci's DIY Remote Controller");
  Serial.println("         by Addicore.com");
  Serial.println("      Controller Version 1.0");
  Serial.println("----------------------------------");

  CheckEeprom();    // Check eeprom data and initilize if necessary

  if ((digitalRead(PROG_BTN_1) == LOW) || (digitalRead (PROG_BTN_2) == LOW)) { // check for either program buttons to be down
    ProgramMode();                                                               // enter program mode if so
  }

  SetUpRadio(); // Setup Radio for transmit

}

//**********************************************************************************************************************
void loop() {

  // Read analog inputs and place them in to the Data Package, int = 2 bytes per
  data.LjPotX = analogRead(L_JS_X);
  data.LjPotY = analogRead(L_JS_Y);
  data.RjPotX = analogRead(R_JS_X);
  data.RjPotY = analogRead(R_JS_Y);
  data.LsPot = analogRead(L_SLD);
  data.RsPot = analogRead(R_SLD);

  // Read switch inputs and pack into a byte
  bitWrite(data.Switches, 7, digitalRead(L_BTN1));
  bitWrite(data.Switches, 6, digitalRead(L_TGL));
  bitWrite(data.Switches, 5, digitalRead(L_JS_B));
  bitWrite(data.Switches, 4, digitalRead(L_BTN2));
  bitWrite(data.Switches, 3, digitalRead(R_BTN2));
  bitWrite(data.Switches, 2, digitalRead(R_JS_B));
  bitWrite(data.Switches, 1, digitalRead(R_TGL));
  bitWrite(data.Switches, 0, digitalRead(R_BTN1));

  // adjust pipe if in multi rx mode
  NRFpipeIndex = 0;

  if (EEPROM.read(4) == 1) {                           // check if in multi rx mode
    bitWrite(NRFpipeIndex, 1, !digitalRead(L_TGL));    // if so, read toggle switches
    bitWrite(NRFpipeIndex, 0, !digitalRead(R_TGL));    // and build up pipe index
    NRFpipeIndex = NRFpipeIndex + 2;                   // adjust pipe index, 2 to 5
    radio.openWritingPipe(pipes[NRFpipeIndex]);
  }

  //String pipe = pipes[NRFpipeIndex];
  //Serial.println (pipe);

  // Transmit data package
  radio.write(&data, sizeof(Data_Package));
  //Serial.print("*");
}
//**********************************************************************************************************************


//--------------------------------------------------------------------------------------------------------------------
// NRF24L01 radio setup
//--------------------------------------------------------------------------------------------------------------------


// EEPROM locations used to store radio parameters
const int EEP_FREQ_TABLE_INDEX = 2;   // EEPROM address of table index
const int EEP_PIPE_TABLE_INDEX = 3;   // EEPROM address of pipe table index
const int EEP_MULTI_RX = 4;           // EEPROM address of multi rx flag

// NRF24L01 frequency table
// this table contains 11 selected frequencies out of the 125 possible for the NRF24
// [0] = the master frequency used only in program mode to sync with the receiver
// [1] through [10] are the available operating frequencies that can be set in program mode
// valid NRF24 frequencies are numbered 0 to 124, frequencies 100-124 are reccomended as are they above wifi
const int NRFfrequencyTable[12] = {100, 102, 104, 106, 108, 110, 112, 114, 116, 120, 124};


void SetUpRadio(void) {
  int NRFfrequencyIndex = 0;

  if (Program == false) {
    NRFfrequencyIndex = EEPROM.read(EEP_FREQ_TABLE_INDEX); // read the frequency table index from EEPROM
    NRFpipeIndex = EEPROM.read(EEP_PIPE_TABLE_INDEX);      // read the pipe table index from EEPROM
  } else {
    NRFfrequencyIndex = 0;  // program mode, set for master frequency and pipe
    NRFpipeIndex = 1;       //
  }

  radio.begin();
  radio.setChannel(NRFfrequencyTable[NRFfrequencyIndex]);  // use the index to look up the frequency and set the NRF24l01
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);   // Set data rate to 250kbps
  radio.stopListening();
  radio.openWritingPipe(pipes[NRFpipeIndex]);

  Serial.print ("Frequency = ");
  Serial.print (NRFfrequencyIndex);
  Serial.print ("  ");
  Serial.println (NRFfrequencyTable[NRFfrequencyIndex]);
  Serial.print ("Pipe = ");
  Serial.print (NRFpipeIndex);
  Serial.print ("  ");
  String pipe = pipes[NRFpipeIndex];
  Serial.println (pipe);
  return;
}


//--------------------------------------------------------------------------------------------------------------------
//   Program mode
//--------------------------------------------------------------------------------------------------------------------

const unsigned long PROG_TIMOUT = 30000;            // program timeout (in ms)
const unsigned long LED_ON_TIME_SHORT = 50;         // led off time (in ms)
const unsigned long LED_ON_TIME = 100;              // led on time (in ms)
const unsigned long LED_OFF_TIME_SHORT = 50;        // led off time (in ms)
const unsigned long LED_OFF_TIME = 300;             // led off time (in ms)
const unsigned long LED_DELAY_TIME = 1000;          // time between flash sequences (in ms)
const unsigned long SHORT_PRESS = 100;              // short button press (in ms), inc fequency number
const unsigned long LONG_PRESS = 5000;              // long button press (in ms), lock in new frequency number, exit program mode

struct Program_Data_Package {
  int pHead = 0xaa;
  int pFreq = 0;
  int pPipe = 0;
  int pTail = 0x55;
};
Program_Data_Package pdata;

int FlashCount;
int NewCount;

void ProgramMode(void) {

  unsigned long ButtonUpCount = 1;
  unsigned long ButtonDownCount = 0;
  unsigned long LedOnCount = 0;
  unsigned long LedOffCount = 0;
  unsigned long ProgramTimeout = 0;

  digitalWrite(PROG_LED, LED_ON);                       // Indicate we are in program mode

  if (digitalRead(PROG_BTN_2) == LOW) {                 // check program button 2
    Program = 2;                                        // set for rx mode program
    if (EEPROM.read(EEP_MULTI_RX) == false) {
      FlashCount = 1;
    } else {
      FlashCount = 4;
    }
  } else {
    Program = 1;                                       // set for frequency mode program
    FlashCount = EEPROM.read(EEP_FREQ_TABLE_INDEX);
  }

  while ((digitalRead(PROG_BTN_1) == LOW) || (digitalRead(PROG_BTN_2) == LOW)) {   // wait for program button to be released
  }

  digitalWrite(PROG_LED, LED_OFF);                       // clear program led

  Serial.println("Program mode - ");

  LedOffCount = millis();
  ProgramTimeout = millis();
  NewCount = FlashCount;

  
  if (Program == 2) {
  Serial.print ("Mode = ");
    if (NewCount == 4) {
      Serial.println ("Multi Rx");
    } else {
      Serial.println("Single Rx");
    }
  } else {
    Serial.print ("Frequency = ");
    Serial.print (NewCount);
    Serial.print ("  ");
    Serial.println (NRFfrequencyTable[NewCount]);

  }

  while (1) {

  if (millis() >= (ProgramTimeout + PROG_TIMOUT)) {   // program mode time out
      delay(1000);
      digitalWrite(PROG_LED, LED_ON);
      delay(1000);
      digitalWrite(PROG_LED, LED_OFF);
      delay(500);
      digitalWrite(PROG_LED, LED_ON);
      delay(1000);
      digitalWrite(PROG_LED, LED_OFF);
      Serial.println ("timeout");
      Program = false;
      //return;
      Serial.println ("Reset");
      delay(1000);
      ResetFunct();
    }

    if (Program == 1) {                 // which program mode?
      // fall throught to frequency program

      //--------------------------------------------------------------------------------------------------------------------
      // frequency program mode
      //--------------------------------------------------------------------------------------------------------------------
      // flash the current frequency number on program LED


      if ((digitalRead(PROG_LED) == LED_ON)
          && (millis() >= (LedOnCount + LED_ON_TIME))) {
        digitalWrite(PROG_LED, LED_OFF);
        LedOffCount = millis();
        FlashCount--;
      }

      if ((FlashCount > 0)
          && (digitalRead(PROG_LED) == LED_OFF)
          && (millis() >= (LedOffCount + LED_OFF_TIME))) {
        digitalWrite(PROG_LED, LED_ON);
        LedOnCount = millis();
      }

      if ((FlashCount == 0)
          && (digitalRead(PROG_LED) == LED_OFF)
          && (millis() >= (LedOffCount + LED_DELAY_TIME))) {
        digitalWrite(PROG_LED, LED_ON);
        LedOnCount = millis();
        FlashCount = NewCount;
      }

      //--------------------------------------------------------------------------------------------------------------------

      if (digitalRead(PROG_BTN_1) == LOW) {
        if (ButtonDownCount == 0) {    // falling edge of button press
          ButtonDownCount = millis();
          ButtonUpCount = 0;
          ProgramTimeout = millis();
          //Serial.println ("button down");
        }

        if (millis() >= (ButtonDownCount + LONG_PRESS)) { // look for long press to program new data and exit

          digitalWrite(PROG_LED, LED_OFF);
          delay(1000);
          digitalWrite(PROG_LED, LED_ON);

          pdata.pFreq = NewCount;                           // Write new frequency to tx data package
          EEPROM.write(EEP_FREQ_TABLE_INDEX, pdata.pFreq);  // and save in EEPROM

          if (EEPROM.read(4) == 1) {                        // check if in multi rx mode
            pdata.pPipe = 0;                                // if so, read toggle switches
            bitWrite(pdata.pPipe, 1, !digitalRead(L_TGL));  // and build up pipe number
            bitWrite(pdata.pPipe, 0, !digitalRead(R_TGL));  //
            pdata.pPipe = pdata.pPipe + 2;                  // adjust pipe number, 2 to 5
          } else {
            pdata.pPipe = 1;                                // if not multi rx mode, pipe is 1
          }

          EEPROM.write(EEP_PIPE_TABLE_INDEX, pdata.pPipe);  // save pipe number in EEPROM

          SetUpRadio();  // set radio to master frequency and pipe (Program flag is true)
          radio.write(&pdata, sizeof(Program_Data_Package));  // Transmit data package

          delay(2000);
          digitalWrite(PROG_LED, LED_OFF);
          Program = false;

          Serial.print ("exit program mode");
          Serial.print ("  ");
          Serial.print (pdata.pFreq);
          Serial.print (" ");
          Serial.println (pdata.pPipe);

          //return;
          Serial.println ("Reset");
          delay(1000);
          ResetFunct();
        }
      }

      if ((digitalRead(PROG_BTN_1) == HIGH) && (ButtonUpCount == 0)) {    // rising edge of button press
        ButtonUpCount = millis();
        //Serial.println ("button up");

        if (ButtonUpCount >= (ButtonDownCount + SHORT_PRESS)) {
          //Serial.println ("short press");
          NewCount++;
          if (NewCount == 11) {
            NewCount = 1;
          }

          Serial.print ("Frequency = ");
          Serial.print (NewCount);
          Serial.print ("  ");
          Serial.println (NRFfrequencyTable[NewCount]);

        }
        ButtonDownCount = 0;
      }

      //--------------------------------------------------------------------------------------------------------------------

    } else {

      //--------------------------------------------------------------------------------------------------------------------
      // Rx mode program

      //--------------------------------------------------------------------------------------------------------------------
      // flash the current Rx mode on leds, 1 flash = single receiver, 4 flashes = 4 receivers

      if ((digitalRead(PROG_LED) == LED_ON)
          && (millis() >= (LedOnCount + LED_ON_TIME_SHORT))) {
        digitalWrite(PROG_LED, LED_OFF);
        LedOffCount = millis();
        FlashCount--;
      }

      if ((FlashCount > 0)
          && (digitalRead(PROG_LED) == LED_OFF)
          && (millis() >= (LedOffCount + LED_OFF_TIME_SHORT))) {
        digitalWrite(PROG_LED, LED_ON);
        LedOnCount = millis();
      }

      if ((FlashCount == 0)
          && (digitalRead(PROG_LED) == LED_OFF)
          && (millis() >= (LedOffCount + LED_DELAY_TIME))) {
        digitalWrite(PROG_LED, LED_ON);
        LedOnCount = millis();
        FlashCount = NewCount;
      }

      //--------------------------------------------------------------------------------------------------------------------

      if (digitalRead(PROG_BTN_2) == LOW) {
        if (ButtonDownCount == 0) {    // falling edge of button press
          ButtonDownCount = millis();
          ButtonUpCount = 0;
          ProgramTimeout = millis();
          //Serial.println ("button down");
        }

        if (millis() >= (ButtonDownCount + LONG_PRESS)) { // look for long press to program new data and exit

          if (NewCount == 1) {
            EEPROM.write(EEP_MULTI_RX, 0);
          } else {
            EEPROM.write(EEP_MULTI_RX, 1);
          }

          digitalWrite(PROG_LED, LED_OFF);
          delay(1000);
          digitalWrite(PROG_LED, LED_ON);
          delay(2000);
          digitalWrite(PROG_LED, LED_OFF);
          Program = false;

          Serial.print ("exit RX program mode");
          Serial.print ("  ");
          Serial.println (NewCount);

          //return;
          Serial.println ("Reset");
          delay(1000);
          ResetFunct();
        }
      }

      if ((digitalRead(PROG_BTN_2) == HIGH) && (ButtonUpCount == 0)) {    // rising edge of button press
        ButtonUpCount = millis();
        //Serial.println ("button up");

        if (ButtonUpCount >= (ButtonDownCount + SHORT_PRESS)) {
          //Serial.println ("short press");
          NewCount = NewCount + 3;
          if (NewCount > 4) {
            NewCount = 1;
          }

          Serial.print ("Mode = ");
          if (NewCount == 4) {
            Serial.println ("Multi Rx");
          } else {
            Serial.println("Single Rx");
          }

        }
        ButtonDownCount = 0;
      }
    }
  }
}
//--------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------
// Check eeprom for valid head, tail, and that the frequency and pipe table indexes are in the correct range
// if any are incorrect, rewrite the entire block with default values
//--------------------------------------------------------------------------------------------------------------------

void CheckEeprom(void) {

  if ((EEPROM.read(1) != 0xAA)
      || ((EEPROM.read(2) < 0) || (EEPROM.read(2) > 10))
      || ((EEPROM.read(3) < 0) || (EEPROM.read(3) > 5))
      || (EEPROM.read(4) > 1)
      || (EEPROM.read(5) != 0x55)) {

    Serial.println ("EEPROM Corrupt - Rewriting with defaults ");
    EEPROM.write(1, 0xAA);  // block head
    EEPROM.write(2, 5);     // default frequency
    EEPROM.write(3, 1);     // default pipe
    EEPROM.write(4, 0);     // default Multi Rx flag (off)
    EEPROM.write(5, 0x55);  // block tail
  }

  Serial.print ("EEPROM Valid: ");
  Serial.print(EEPROM.read(1), HEX);
  Serial.print("  ");
  Serial.print(EEPROM.read(2), HEX);
  Serial.print("  ");
  Serial.print(EEPROM.read(3), HEX);
  Serial.print("  ");
  Serial.print(EEPROM.read(4), HEX);
  Serial.print("  ");
  Serial.println(EEPROM.read(5), HEX);

  return;
}
