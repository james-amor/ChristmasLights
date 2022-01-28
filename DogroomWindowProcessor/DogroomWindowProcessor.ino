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
#include "PCA9685.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"


//Channel to VIXEN to Output:
//1 -> Net-L  -> (Bedroom - Not Mapped)
//2 -> Net-R  -> (Bedroom - Not Mapped)
//3 -> HHH-TL  -> (Bedroom - Not Mapped)
//4 -> HHH-BL  -> (Bedroom - Not Mapped)
//5 -> HHH-TR  -> (Bedroom - Not Mapped)
//6 -> HHH-BR  -> (Bedroom - Not Mapped)
//7 -> Candy-L -> (Bedroom - Not Mapped)
//8 -> Candy-R -> (Bedroom - Not Mapped)
//9 -> (Spare) -> (Bedroom - Not Mapped)
//10 -> (Spare) -> (Bedroom - Not Mapped)
//11 -> Window Net  -> PWM0 (Plus A0 and A1)
//12 -> Projector -> PWM1
//13 -> LED-Neon  -> PWM2 
//14 -> (Spare)   -> PWM3
//15 -> (Spare)   -> PWM4
//16 -> (Spare)   -> PWM4

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

const byte WINDOW_PINA = A0;
const byte WINDOW_PINB = A1;
//---------------------
#define CONTROLLER_CHANNEL_COUNT 16 
#define PWM_OUTPUT_COUNT 16

#define IN_BUFF_SIZE 100

//---------------------
PCA9685 pwmController; 
uint16_t pwmDemand [PWM_OUTPUT_COUNT];
uint8_t lastChanValues [CONTROLLER_CHANNEL_COUNT];

const int chanToPwmMapping [CONTROLLER_CHANNEL_COUNT] = 
    {-1, //Bedroom (Not mapped)
     -1, //Bedroom (Not mapped)
     -1, //Bedroom (Not mapped)
     -1, //Bedroom (Not mapped)
     -1, //Bedroom (Not mapped)
     -1, //Bedroom (Not mapped)
     -1, //Bedroom (Not mapped)
     -1, //Bedroom (Not mapped)
     -1, //Bedroom (Not mapped)
     -1, //Bedroom (Not mapped)
     0, //LED Neon
     1, //Projector 
     2, //Window Net
     3, //Dog-Room (Not mapped)
     4, //Dog-Room (Not mapped)
     5};  //Dog-Room (Not mapped)

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
uint16_t bytesReceived;

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
  pinMode (WINDOW_PINA, OUTPUT);
  pinMode (WINDOW_PINB, OUTPUT);
  digitalWrite (WINDOW_PINA, LOW);
  digitalWrite (WINDOW_PINB, LOW);
  
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
  pwmController.setPWMFrequency(245); //490);
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

  //Inly chage every two executions (4ms)
  if (lastNetChange == 0)
  {
    lastLedState = !lastLedState;
    digitalWrite (WINDOW_PINA, (lastLedState ? HIGH : LOW));
    digitalWrite (WINDOW_PINB, (lastLedState ? LOW  : HIGH));
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

  if (messageReady == true)
  {      
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
