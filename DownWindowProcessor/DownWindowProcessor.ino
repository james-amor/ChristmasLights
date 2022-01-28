//===================================================================
/* 
 * Copyright (C) 2022 James Amor.   All Rights Reserved.
 * 
 * NOTICE:  All information contained herein is, and remains
 * the property of James Amor. The intellectual and technical concepts 
 * contained herein are proprietary to James Amor and may be covered by 
 * UK and Foreign Patents, patents in process, and are protected by 
 * trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from James Amor.
 */
//===================================================================


#include "RenardInterface.h"

//REQUIRES MEGA FOR PWM COUNT
//***DON'T USE PINS 5 + 6 - Interactions with on-board timer functionss!

//Channel to VIXEN to Output:
//1 -> Net-L  -> D3  (Plus D50 & D51 for H-bridge)
//2 -> Net-R  -> D4  (Plus D52 & D53 for H-bridge)
//3 -> HHH-L  -> D7
//4 -> HHH-R  -> D8
//5 -> Candy-L -> D9
//6 -> Candy-R -> D10
//7 -> (Spare) -> D11
//8 -> (Spare) -> D12
//9 -> (Spare) -> D44
//10 -> (Spare) -> D45
//11 -> (Spare) -> D46

#define MESSAGE_FEEDBACK_PIN 13
#define STRING_CHANGE_ISR_PIN 2

#define CONTROLLER_CHANNEL_COUNT 11

#define IN_BUFF_SIZE 100

const byte WINDOW_L_PINA = 50;
const byte WINDOW_L_PINB = 51;
const byte WINDOW_R_PINA = 52;
const byte WINDOW_R_PINB = 53;

const int chanToPinMapping [CONTROLLER_CHANNEL_COUNT] = 
    {3, //Net-L
     4, //Net-R
     7, //HHH-L
     8, //HHH-R
     9, //Cane-L
     10, //Cane-R
     11, //Spare-1
     12, //Spare-2
     44, //Spare-3
     45, //Spare-6
     46}; //Spare-7

uint8_t lastChanValues [CONTROLLER_CHANNEL_COUNT];

uint8_t inputBuffer[IN_BUFF_SIZE];
uint16_t renardBytesRead;

//==================================================================================
void setup() 
{
  Serial.begin (38400);
  Serial.println ("Starting");

  pinMode (MESSAGE_FEEDBACK_PIN, OUTPUT);
  digitalWrite (MESSAGE_FEEDBACK_PIN, HIGH);

  //Initialise data structures
  memset (&lastChanValues[0], 0, sizeof(uint8_t) * CONTROLLER_CHANNEL_COUNT);  

  //Setup Window Net Interrupt Handling
  pinMode (WINDOW_L_PINA, OUTPUT);
  pinMode (WINDOW_L_PINB, OUTPUT);
  pinMode (WINDOW_R_PINA, OUTPUT);
  pinMode (WINDOW_R_PINB, OUTPUT);

  digitalWrite (WINDOW_L_PINA, LOW);
  digitalWrite (WINDOW_L_PINB, LOW);
  digitalWrite (WINDOW_R_PINA, LOW);
  digitalWrite (WINDOW_R_PINB, LOW);

  pinMode (STRING_CHANGE_ISR_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(STRING_CHANGE_ISR_PIN), stringChange2ms_ISR, CHANGE);  
  analogWrite (STRING_CHANGE_ISR_PIN, 128);  

  delay(50);

  //Configure PWM output
  for (int idx = 0; idx < CONTROLLER_CHANNEL_COUNT; idx++)
  {
    pinMode (chanToPinMapping[idx], OUTPUT);
    analogWrite (chanToPinMapping[idx], 0);
  }

  //Test PWM Outputs  
  for (int idx = 0; idx < CONTROLLER_CHANNEL_COUNT; idx++)
  {
    analogWrite (chanToPinMapping[idx], 255);
    delay(250);
  }
  
  delay(500);
  
  for (int idx = 0; idx < CONTROLLER_CHANNEL_COUNT; idx++)
  {
    analogWrite (chanToPinMapping[idx], 0);
    delay(250);
  }
  
  resetInBuffer();
  
  Serial.println ("Started");
  
  digitalWrite (MESSAGE_FEEDBACK_PIN, LOW);  
}
//==================================================================================
void loop() 
{     
  while (true)
  {
    renardBytesRead = renardRead(&inputBuffer[0], IN_BUFF_SIZE);
    processBuffer();
  }
}
//==================================================================================
void stringChange2ms_ISR (void)
{
  static bool lastLedState = LOW; 
  static byte lastNetChange = 0;
  
  if (lastNetChange == 0)
  {
    lastLedState = !lastLedState;
    digitalWrite (WINDOW_L_PINA, (lastLedState ? HIGH : LOW));
    digitalWrite (WINDOW_L_PINB, (lastLedState ? LOW  : HIGH));
  }
  else
  {
    digitalWrite (WINDOW_R_PINA, (lastLedState ? HIGH : LOW));
    digitalWrite (WINDOW_R_PINB, (lastLedState ? LOW  : HIGH));
  }

  lastNetChange = 1 - lastNetChange;
}
//==================================================================================
void processBuffer (void)
{
  static unsigned long lastCommandRx = 0;
  
//  Serial.print (F("Start: "));
//  Serial.print (startPos);
//  Serial.print (F(", End: "));
//  Serial.print (endPos);
//  Serial.print (F(", Len: "));
//  Serial.print (endPos-startPos);
//  Serial.print (F(", Tim: "));    
//  Serial.print (millis() - lastCommandRx);
//  Serial.println (F("ms"));    
//  printInBuffer();


  if ((millis() - lastCommandRx) > 500)
  {
    Serial.print (F("*** TIMING : "));
    Serial.println (millis() - lastCommandRx);
  }
  
  if (renardBytesRead == CONTROLLER_CHANNEL_COUNT)
  {
    bool channelsChanged = false;

    digitalWrite (MESSAGE_FEEDBACK_PIN, (digitalRead(MESSAGE_FEEDBACK_PIN) == LOW ? HIGH : LOW));
    
    for (byte chan = 0; chan < CONTROLLER_CHANNEL_COUNT; chan ++)
    {
      if (lastChanValues[chan] != inputBuffer[chan])
      {
        channelsChanged = true;
        lastChanValues[chan] = inputBuffer[chan];
      }
    }

    if (channelsChanged == true)
    {
      //Now process the channel info into specifics      
      for (byte chan = 0; chan < CONTROLLER_CHANNEL_COUNT; chan ++)
      {
        analogWrite (chanToPinMapping[chan], lastChanValues[chan]);
      }
    }
  }
  else
  {
    Serial.println (F("*****>> Corrupt In Buffer Len"));    
    printInBuffer();    
  }
  
  resetInBuffer();
  
  lastCommandRx = millis();  
}
//==================================================================================
void resetInBuffer (void)
{
  memset (&inputBuffer[0], 0, sizeof(uint8_t) * IN_BUFF_SIZE);
  renardBytesRead = 0;
}
//==================================================================================
void printInBuffer ()
{
  Serial.print (F("InBuff Len: "));
  Serial.print (renardBytesRead);
  Serial.print (F(" [")); 

  for (int idx = 0; idx < renardBytesRead; idx++)
  {  
    Serial.print ((int)inputBuffer[idx], HEX);
    Serial.print (" ");
  }
  
  Serial.println (F("]"));  
}
//=========================================================================
//=========================================================================
