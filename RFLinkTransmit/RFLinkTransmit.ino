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


#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "RenardInterface.h"

//-------------------Pin Definitions-------------------
const byte RF24_IRQ_PIN = 2;
const byte TX_LED_PIN   = 3;
const byte TX_LED_GND_PIN = 5;
const byte RF24_CE_PIN = 9;
const byte RF24_CSN_PIN = 10;
const byte RF24_MO_PIN = 11;
const byte RF24_MI_PIN = 12;
const byte RF24_SCK_PIN = 13;

//-------------------Radio Definitions-------------------
const uint64_t MASTER_TO_SLAVE_PIPE = 0xF0F0F0F06BLL;
const uint64_t SLAVE_TO_MASTER_PIPE = 0xF0F0F0F06CLL;

const rf24_pa_dbm_e radioTxLevel = RF24_PA_MAX; //RF24_PA_LOW;
const rf24_datarate_e radioDataRate = RF24_250KBPS; //RF24_1MBPS; //RF24_250KBPS;

RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);

typedef struct 
{ 
  byte dyntag;
  byte channel_demands [20];
} RadioPayloadType;

const int RadioPayloadTypeSize = sizeof(RadioPayloadType);

RadioPayloadType messageToSend;
bool messageReady = false;

//-------------------Serial Steam Definitions-------------------
#define CONTROLLER_CHANNEL_COUNT 16 
#define IN_BUFF_SIZE 100
uint8_t inputBuffer[IN_BUFF_SIZE];
uint16_t bytesReceived = 0;



//---------------------------------------------------------
//---------------------------------------------------------
void setup ()
{
  Serial.begin (57600);
  Serial.println ("Starting");

  pinMode (TX_LED_GND_PIN, OUTPUT);
  digitalWrite (TX_LED_GND_PIN, LOW);
  pinMode (TX_LED_PIN, OUTPUT);
  digitalWrite (TX_LED_PIN, HIGH);
  
  delay (500);

  radio.begin();

  while (radio.isChipConnected() == false)
  {
    Serial.println ("NO RADIO");
    delay (500);
  }

  radio.setRetries(12, 0);  //P1 = Multiples of 250us between retried : 4 x 250 == 1ms,
  //P2 = number of attempts
  //(4, 1) 4 x 250us = 1000us wait, x 0 retries = max 1ms delay
  radio.setChannel (120);
  radio.setAutoAck (false);
  radio.setPALevel (radioTxLevel);
  radio.setDataRate (radioDataRate);
  radio.setPayloadSize(RadioPayloadTypeSize);

  radio.openReadingPipe(1, SLAVE_TO_MASTER_PIPE);
  radio.startListening();

  resetInBuffer();
  digitalWrite (TX_LED_PIN, LOW);

  Serial.println ("Started");  
}

//---------------------------------------------------------
//---------------------------------------------------------
void loop ()
{
  unsigned long lastMessageTx = 0;
  memset (&messageToSend, 0, RadioPayloadTypeSize);

  while (true)
  {   
    bytesReceived = renardRead(&inputBuffer[0], IN_BUFF_SIZE);
    processBuffer();

    if (messageReady == true)
    {    
      if (messageToSend.dyntag < 0xFF) 
      {
        messageToSend.dyntag++;
      }
      else
      {
        messageToSend.dyntag = 1;
      }
    
      radio.stopListening();    
      radio.openWritingPipe(MASTER_TO_SLAVE_PIPE);
      radio.write(&messageToSend, RadioPayloadTypeSize);
      lastMessageTx = millis();      
      delay (3);
      radio.startListening();   

      //Wait for 10ms since last Tx, but stop if serial data arrives
      while (((millis() - lastMessageTx) < 9) && (Serial.available() == 0))
      {       
        delayMicroseconds(500);
      }

      //If there's no serial data available then retransmit the last message
      if (Serial.available() == 0)
      {
        radio.stopListening();    
        radio.openWritingPipe(MASTER_TO_SLAVE_PIPE);
        radio.write(&messageToSend, RadioPayloadTypeSize);
        delay (3);
        radio.startListening();           
      }
      
      messageReady = false;
    }    
  }
}

//---------------------------------------------------------
//---------------------------------------------------------
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
    Serial.println (millis() - lastCommandRx) ;
  }
 
  
  if (bytesReceived == CONTROLLER_CHANNEL_COUNT)
  {    
    digitalWrite (TX_LED_PIN, (digitalRead(TX_LED_PIN) == LOW ? HIGH : LOW));

    for (byte chan = 0; chan < CONTROLLER_CHANNEL_COUNT; chan ++)
    {
      messageToSend.channel_demands[chan] = inputBuffer[chan];
    }

    messageReady = true;
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
  bytesReceived = 0;  
}
//==================================================================================
void printInBuffer ()
{
  Serial.print (F("InBuff Len: "));
  Serial.print (bytesReceived);
  Serial.print (F(" [")); 

  for (int idx = 0; idx < bytesReceived; idx++)
  {  
    Serial.print ((int)inputBuffer[idx], HEX);
    Serial.print (" ");
  }
  
  Serial.println (F("]"));  
}
//=========================================================================

 
 
