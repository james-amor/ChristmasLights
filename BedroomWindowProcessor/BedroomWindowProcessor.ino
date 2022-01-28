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


#include "PCA9685.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "RenardInterface.h"


//Channel to VIXEN to Output:
//1 -> Net-L  -> PWM0
//2 -> Net-R  -> PWM1
//3 -> HHH-TL  -> PWM2
//4 -> HHH-BL  -> PWM3
//5 -> HHH-TR  -> PWM4
//6 -> HHH-BR  -> PWM5
//7 -> Candy-L -> PWM6
//8 -> Candy-R -> PWM7
//9 -> (Spare) -> PWM8
//10 -> (Spare) -> PWM9
//11 -> LED-Neon  -> (Dogroom window - Not mapped)
//12 -> Projector -> (Dogroom window - Not mapped)
//13 -> Net       -> (Dogroom window - Not mapped)
//14 -> (Spare)   -> (Dogroom window - Not mapped)
//15 -> (Spare)   -> (Dogroom window - Not mapped)
//16 -> (Spare)   -> (Dogroom window - Not mapped)

//---------------------
const byte LED_GND_PIN = 2;
const byte STRING_CHANGE_ISR_PIN = 3;
const byte RED_PIN = 4;
const byte GREEN_PIN = 5;

const byte RECEIVE_MODE_PIN = 6;  //LOW = RF,  HIGH (N/C) = SERIAL

const byte PWM_VCC_PIN = 7;

const byte RF24_IRQ_PIN = 2;
const byte RF24_CE_PIN = 9;
const byte RF24_CSN_PIN = 10;
const byte RF24_MO_PIN = 11;
const byte RF24_MI_PIN = 12;
const byte RF24_SCK_PIN = 13;

const byte PWM_SDA_PIN = A4;
const byte PWM_SCL_PIN = A5;

const byte WINDOW_L_PINA = A0;
const byte WINDOW_L_PINB = A1;
const byte WINDOW_R_PINA = A2;
const byte WINDOW_R_PINB = A3;
//---------------------
#define CONTROLLER_CHANNEL_COUNT 16 
#define PWM_OUTPUT_COUNT 16

#define IN_BUFF_SIZE 100

//---------------------
PCA9685 pwmController; 
uint16_t pwmDemand [PWM_OUTPUT_COUNT];
uint8_t lastChanValues [CONTROLLER_CHANNEL_COUNT];

const int chanToPwmMapping [CONTROLLER_CHANNEL_COUNT] = 
    {0, //Net-L
     1, //Net-R
     2, //HHH-TL
     3, //HHH-BL
     4, //HHH-TR
     5, //HHH-BR
     6, //Cane-L
     7, //Cane-R
     8, //Spare-1
     9, //Spare-2
     -1, //Dog-Room (Not mapped)
     -1, //Dog-Room (Not mapped)
     -1, //Dog-Room (Not mapped)
     -1, //Dog-Room (Not mapped)
     -1, //Dog-Room (Not mapped)
     -1};  //Dog-Room (Not mapped)

//---------------------
const uint64_t MASTER_TO_SLAVE_PIPE = 0xF0F0F0F06BLL;
const uint64_t SLAVE_TO_MASTER_PIPE = 0xF0F0F0F06CLL;

const rf24_pa_dbm_e radioTxLevel = RF24_PA_LOW;
const rf24_datarate_e radioDataRate = RF24_250KBPS; //RF24_1MBPS; //RF24_250KBPS;

RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);

typedef struct 
{ 
  byte dyntag;
  byte channel_demands [20];
} RadioPayloadType;

const int RadioPayloadTypeSize = sizeof(RadioPayloadType);

//---------------------

uint8_t inputBuffer[IN_BUFF_SIZE];
uint16_t bytesReceived = 0;

//---------------------

bool RECEIVE_VIA_RF = false;


//==================================================================================
void setup() 
{
  Serial.begin (38400);
  Serial.println ("Starting");

  pinMode (PWM_VCC_PIN, OUTPUT);
  digitalWrite (PWM_VCC_PIN, HIGH);

  pinMode (LED_GND_PIN, OUTPUT);
  digitalWrite (LED_GND_PIN, LOW);
  
  pinMode (RED_PIN, OUTPUT);
  pinMode (GREEN_PIN, OUTPUT);

  digitalWrite (RED_PIN, HIGH);
  digitalWrite (GREEN_PIN, HIGH);  
  delay (1000);
  digitalWrite (GREEN_PIN, LOW);

  
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
  attachInterrupt(digitalPinToInterrupt(STRING_CHANGE_ISR_PIN), stringChange2ms_ISR, FALLING);  
  analogWrite (STRING_CHANGE_ISR_PIN, 128);  

  //Configure whether we're using serial or RF
  pinMode (RECEIVE_MODE_PIN, INPUT_PULLUP);  
  delay (50);
  if (digitalRead(RECEIVE_MODE_PIN) == HIGH)
  {
    RECEIVE_VIA_RF = false;
  }
  else
  {
    RECEIVE_VIA_RF = true;    
  }
  
  RECEIVE_VIA_RF = true;    
  
  //Initialise data structures
  memset (&lastChanValues[0], 0, sizeof(uint8_t) * CONTROLLER_CHANNEL_COUNT);  
  memset (&pwmDemand[0], 0, sizeof(uint16_t) * PWM_OUTPUT_COUNT);

  //Configure PWM output card
  Wire.begin();  
  pwmController.resetDevices();
  pwmController.init();    
  pwmController.setPWMFrequency(245);
  pwmController.setChannelsPWM(0, PWM_OUTPUT_COUNT, pwmDemand );
  
  for (int idx = 0; idx < PWM_OUTPUT_COUNT; idx++)
  {
    pwmDemand[idx] = (255 << 4);
  }
  pwmController.setChannelsPWM(0, PWM_OUTPUT_COUNT, pwmDemand );        
  
  delay(2000);
  
  for (int idx = 0; idx < 16; idx++)
  {
    pwmDemand[idx] = 0;
  }
  pwmController.setChannelsPWM(0, PWM_OUTPUT_COUNT, pwmDemand );        

  if (RECEIVE_VIA_RF == true)
  {
    radio.begin();
  
    while (radio.isChipConnected() == false)
    {    
      Serial.println ("NO RADIO");
      delay (100);
    }
      
    radio.setRetries(12, 0);  //P1 = Multiples of 250us between retried : 4 x 250 == 1ms,
    //P2 = number of attempts
    //(4, 1) 4 x 250us = 1000us wait, x 0 retries = max 1ms delay
    radio.setChannel (120);
    radio.setAutoAck (false);
    radio.setPALevel (radioTxLevel);
    radio.setDataRate (radioDataRate);
    radio.setPayloadSize(RadioPayloadTypeSize);
  
    radio.openReadingPipe(1, MASTER_TO_SLAVE_PIPE);
    radio.startListening();
  }


  
  //Reset the serial buffer, ready to recieve
  resetInBuffer();
    
  Serial.println ("Started");
  
  digitalWrite (RED_PIN, LOW);
}
//==================================================================================
void loop() 
{     
  if (RECEIVE_VIA_RF == true)
  {
    receiveRFData();    
  }
  else
  {
    receiveSerialData();
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
void receiveRFData ()
{
  static bool hadError = false;
  static byte lastDyntag = 0;
  unsigned long lastMsgReceived = 0;
  bool messageReady = false;
  RadioPayloadType messageReceived;
  
  
  memset (&messageReceived, 0, RadioPayloadTypeSize);

  while (radio.available())
  {
    // Work through all received messages
    radio.read(&messageReceived, RadioPayloadTypeSize);
    messageReady = true;
  }

  //Ensure that all channels are turned off when no data has been received for more than a second
  if ((messageReady == false) && (lastMsgReceived > 0))
  {
    if ((millis() - lastMsgReceived) > 1000UL)
    {
      digitalWrite (RED_PIN, HIGH);
      digitalWrite (GREEN_PIN, LOW);      

      resetInBuffer();
      bytesReceived = CONTROLLER_CHANNEL_COUNT;
      processBuffer();
      
      lastMsgReceived = 0;
    }
  }

  if (messageReady == true)
  {    
    lastMsgReceived = millis();
    
    if (hadError)
    {
      digitalWrite (RED_PIN, (digitalRead(RED_PIN) == HIGH) ? LOW : HIGH);
      digitalWrite (GREEN_PIN, LOW);      
    }
    else
    {
      digitalWrite (GREEN_PIN, (digitalRead(GREEN_PIN) == HIGH) ? LOW : HIGH);
      digitalWrite (RED_PIN, LOW);            
    }

    //Serial.print (">#");
    //for (byte idx = 0; idx < CONTROLLER_CHANNEL_COUNT; idx++)
    //{
    //  Serial.print ((char)messageReceived.channel_demands[idx]);
    //}
    //Serial.println ("#<");

    resetInBuffer();
    
    for (byte idx = 0; idx < CONTROLLER_CHANNEL_COUNT; idx++)
    {
      inputBuffer[idx] = messageReceived.channel_demands[idx];
    }

    bytesReceived = CONTROLLER_CHANNEL_COUNT;
    processBuffer();

    if (messageReceived.dyntag == 0)
    {
      Serial.println ("E1");
      hadError = true;
    }
    else if (messageReceived.dyntag == lastDyntag)
    {
      //This is a retransmit, and we handled the last message, so ignore it
    }
    else if (lastDyntag > 0)
    {
      if (messageReceived.dyntag == 1)
      {
        hadError = false;
        
        if (lastDyntag != 255)
        {
          hadError = true;
          Serial.println ("E2");
        }
      }
      else
      {
        if ((messageReceived.dyntag - lastDyntag) != 1)
        {
          hadError = true;
          Serial.print ("E3-");
          Serial.println ((messageReceived.dyntag - lastDyntag));
        }          
      }
    }
    
    lastDyntag = messageReceived.dyntag;      
    messageReady = false;
  } 
}
//==================================================================================
void receiveSerialData ()
{
    bytesReceived = renardRead(&inputBuffer[0], IN_BUFF_SIZE);
    processBuffer();
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
 
  
  if (bytesReceived == CONTROLLER_CHANNEL_COUNT)
  {
    bool channelsChanged = false;
    
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
        if (chanToPwmMapping[chan] != -1)
        {
          pwmDemand[chanToPwmMapping[chan]] = (lastChanValues[chan] << 4);
        }
      }
      pwmController.setChannelsPWM(0, PWM_OUTPUT_COUNT, pwmDemand );        
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
inline void resetInBuffer (void)
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
