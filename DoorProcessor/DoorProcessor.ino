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
#include "RenardInterface.h"

//VIXEN Mappings:
//1 -> Robins-1  -> PWM0
//2 -> Robins-2  -> PWM1
//3 -> Robins-3  -> PWM2
//4 -> Robins-4  -> PWM3
//5 -> Robins-5  -> PWM4
//6 -> SPSH      -> PWM5
//7 -> Fill-Red  -> D9  (10mA Sink current limit on i2c board means this needs to use on-board)
//8 -> Fill-Blue -> D10 (10mA Sink current limit on i2c board means this needs to use on-board)
//9 -> Fill-Green-> D11 (10mA Sink current limit on i2c board means this needs to use on-board)
//10 -> Spot Light-> PWM9
//11 -> Door-S1-Blue
//12 -> Door-S1-Green   -> PWM10
//13 -> Door-S1-Yellow
//14 -> Door-S2-Blue 
//15 -> Door-S2-Green
//16 -> Door-S2-Yellow  -> PWM11
//17 -> Door-S3-Blue
//18 -> Door-S3-Green
//19 -> Door-S3-Yellow  -> PWM12
//20 -> Door-S4-Blue
//21 -> Door-S4-Green
//22 -> Door-S4-Yellow  -> PWM13
//23 -> StartShowButton -> PWM6
//24 -> Spare -> PWM7
//25 -> Spare -> PWM8
//26 -> Spare -> PWM14
//27 -> Spare -> PWM15
//28 -> Spare -> D2
//29 -> Spare -> D8
//30 -> Spare -> D12


//#define ENABLE_DEBUG_OUTPUT

#ifdef ENABLE_DEBUG_OUTPUT
#define DebugBegin(a) (Serial.begin(a))
#define DebugFlush() (Serial.flush())
#define DebugPrint(a) (Serial.print(a))
#define DebugPrintVal(a,b) (Serial.print(a,b))
#define DebugPrintln(a) (Serial.println(a))

#define DebugAvailable() (Serial.available())
#define DebugRead() (Serial.read())
#else
#define DebugBegin(a)
#define DebugFlush()
#define DebugPrint(a)
#define DebugPrintVal(a,b)
#define DebugPrintln(a)
#define DebugAvailable() false
#define DebugRead() '\0'
#endif

#define MESSAGE_FEEDBACK_PIN 13
#define STRING_CHANGE_ISR_PIN 3

#define STRING1_PINA  7
#define STRING1_PINB  6
//      STRING1_PWM   PCA-CH-10

#define STRING2_PINA  A1
#define STRING2_PINB  A0
//      STRING1_PWM   PCA-CH-11

#define STRING3_PINA  A3
#define STRING3_PINB  A2 
//      STRING1_PWM   PCA-CH-12

#define STRING4_PINA  4
#define STRING4_PINB  5
//      STRING1_PWM   PCA-CH-13

#define SPARE_OUT_D2   2
#define SPARE_OUT_D8    8
#define SPARE_OUT_D12   12

#define FILL_LIGHT_RED    9
#define FILL_LIGHT_GREEN  10
#define FILL_LIGHT_BLUE   11

#define CONTROLLER_CHANNEL_COUNT 30 
#define PWM_OUTPUT_COUNT 16
#define DISC_OUTPUT_COUNT 14

#define IN_BUFF_SIZE 100

PCA9685 pwmController; 
uint16_t pwmDemand [PWM_OUTPUT_COUNT];

int  discreteOutputPins[DISC_OUTPUT_COUNT]  = {STRING1_PINA, STRING2_PINA, STRING3_PINA, STRING4_PINA,  STRING1_PINB,
                                                STRING2_PINB, STRING3_PINB, STRING4_PINB, SPARE_OUT_D2, SPARE_OUT_D8,
                                                SPARE_OUT_D12, FILL_LIGHT_RED, FILL_LIGHT_GREEN, FILL_LIGHT_BLUE};

enum STRING_STATE { LED_OFF, LED_YELL, LED_BLUE, LED_BOTH };

#define STRING_COUNT 4

volatile bool updatingStringState = false;
volatile byte stringState[STRING_COUNT] = {LED_OFF, LED_OFF, LED_OFF, LED_OFF} ;
byte stringPwm[STRING_COUNT]   = {0,0,0,0};

int  stringPinA[STRING_COUNT]  = {STRING1_PINA, STRING2_PINA, STRING3_PINA, STRING4_PINA};
int  stringPinB[STRING_COUNT]  = {STRING1_PINB, STRING2_PINB, STRING3_PINB, STRING4_PINB};

uint8_t inputBuffer[IN_BUFF_SIZE];
uint16_t renardBytesRead;

uint8_t lastChanValues[CONTROLLER_CHANNEL_COUNT];

//==================================================================================
void setup() 
{
  Serial.begin (38400);
  Serial.println ("Starting");

  pinMode (MESSAGE_FEEDBACK_PIN, OUTPUT);
  digitalWrite (MESSAGE_FEEDBACK_PIN, HIGH);
  
  //Initialise data structures
  memset (&lastChanValues[0], 0, sizeof(uint8_t) * CONTROLLER_CHANNEL_COUNT);  
  memset (&pwmDemand[0], 0, sizeof(uint16_t) * PWM_OUTPUT_COUNT);

  //Configure discrete outputs
  for (byte idx=0; idx < DISC_OUTPUT_COUNT; idx++)
  {
      pinMode (discreteOutputPins[idx], OUTPUT);
      digitalWrite (discreteOutputPins[idx], LOW);
  }

  //Configure PWM output card
  Wire.begin();  
  pwmController.resetDevices();
  pwmController.init();    
  pwmController.setPWMFrequency(245);
  pwmController.setChannelsPWM(0, PWM_OUTPUT_COUNT, pwmDemand );

  pinMode (STRING_CHANGE_ISR_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(STRING_CHANGE_ISR_PIN), stringChange1ms_ISR, CHANGE);  
  analogWrite (STRING_CHANGE_ISR_PIN, 128);  
  

  analogWrite (FILL_LIGHT_RED, 0);
  analogWrite (FILL_LIGHT_GREEN, 0);
  analogWrite (FILL_LIGHT_BLUE, 0);
  
  stringState[0] = LED_BOTH;
  stringState[1] = LED_BOTH;
  stringState[2] = LED_BOTH;
  stringState[3] = LED_BOTH;
  
  for (int idx = 0; idx < PWM_OUTPUT_COUNT; idx++)
  {
    pwmDemand[idx] = (255 << 4);
  }
    
  pwmController.setChannelsPWM(0, PWM_OUTPUT_COUNT, pwmDemand );        
  
  delay(2000);

  stringState[0] = LED_OFF;
  stringState[1] = LED_OFF;
  stringState[2] = LED_OFF;
  stringState[3] = LED_OFF;
  
  for (int idx = 0; idx < PWM_OUTPUT_COUNT; idx++)
  {
    pwmDemand[idx] = 0;
  }
 
  pwmController.setChannelsPWM(0, PWM_OUTPUT_COUNT, pwmDemand );        

  analogWrite (FILL_LIGHT_RED, 255);
  analogWrite (FILL_LIGHT_GREEN, 255);
  analogWrite (FILL_LIGHT_BLUE, 255);
  
  //Reset the serial buffer, ready to recieve
  resetInBuffer();

  digitalWrite (MESSAGE_FEEDBACK_PIN, LOW);
  Serial.println ("Started");
}
//==================================================================================
void stringChange1ms_ISR (void)
{
  if (updatingStringState == false)
  {
    updateOutputs();
  }
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
void processBuffer (void)
{
  static unsigned long lastCommandRx = 0;
//  static byte showButtonHoldOn = 0;
  
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
    digitalWrite (MESSAGE_FEEDBACK_PIN, (digitalRead(MESSAGE_FEEDBACK_PIN) == LOW ? HIGH : LOW));

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
      
      //Channels 0-5, straight copy to pwm outputs 
      for (byte chan = 0; chan < 6; chan ++)
      {
        pwmDemand[chan] = (lastChanValues[chan] << 4);
      }

      //6-8, inverted write to analog pins (Fill light RGB)
      analogWrite (FILL_LIGHT_RED, 255 - lastChanValues[6]);
      analogWrite (FILL_LIGHT_GREEN, 255 - lastChanValues[7]);
      analogWrite (FILL_LIGHT_BLUE, 255 - lastChanValues[8]);

      //Channel 9, copy to output
      pwmDemand[9] = (lastChanValues[9] << 4);
      
      //Channels 10-21, tree outputs     
      byte baseChannelIdx;
      int  pwmValue;

      updatingStringState = true;
      for (byte stringIdx = 0; stringIdx < STRING_COUNT; stringIdx++)
      {
        baseChannelIdx = (stringIdx*3) + 10;
        
        if (lastChanValues[baseChannelIdx] != 0)
        {
          DebugPrint ("BLU,");      
          stringState[stringIdx] = LED_BLUE;
          pwmValue = lastChanValues[baseChannelIdx];
        }
        else if (lastChanValues[baseChannelIdx+1] != 0)
        {
          DebugPrint ("GRN,");      
          stringState[stringIdx] = LED_BOTH;
          pwmValue = lastChanValues[baseChannelIdx+1];
        }
        else if (lastChanValues[baseChannelIdx+2] != 0)
        {
          DebugPrint ("YEL,");      
          stringState[stringIdx] = LED_YELL;
          pwmValue = lastChanValues[baseChannelIdx+2];
        }
        else
        {
          DebugPrint ("OFF,");      
          stringState[stringIdx] = LED_OFF;
          pwmValue = 0;
        }
  
        DebugPrint ("-");      
        DebugPrint (pwmValue);      
        DebugPrint (",");      
        pwmDemand[stringIdx+10] = (pwmValue << 4);
        
      } 
      DebugPrintln("");  
      updatingStringState = false;

      pwmDemand[6] = (lastChanValues[22] << 4);        
      
//      //Special handling for the "Start Show" button to ensure it doesn't reset between sequences
//      if (lastChanValues[22] > 5)
//      {
//        showButtonHoldOn = 10;
//      }
//      else
//      {
//        if (showButtonHoldOn > 0)
//        {
//          showButtonHoldOn = showButtonHoldOn - 1;
//        }
//      }
//      
//      if (showButtonHoldOn > 0)
//      {
//        pwmDemand[6] = (255 << 4);        
//      }
//      else
//      {
//        pwmDemand[6] = 0;                
//      }
        
      pwmDemand[7] = (lastChanValues[23] << 4);
      pwmDemand[8] = (lastChanValues[24] << 4);
      
      pwmDemand[14] = (lastChanValues[25] << 4);
      pwmDemand[15] = (lastChanValues[26] << 4);
      pwmController.setChannelsPWM(0, PWM_OUTPUT_COUNT, pwmDemand );        
  
      digitalWrite (SPARE_OUT_D2,  (lastChanValues[27] == 0 ? LOW : HIGH));
      digitalWrite (SPARE_OUT_D8, (lastChanValues[28] == 0 ? LOW : HIGH));
      digitalWrite (SPARE_OUT_D12, (lastChanValues[29] == 0 ? LOW : HIGH));
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
void updateOutputs (void)
{ 
  static byte stringIdx = 0;  
  static int bothLedState = LOW; 

  switch (stringState[stringIdx])
  {
    case LED_OFF:
      digitalWrite (stringPinA[stringIdx], LOW);
      digitalWrite (stringPinB[stringIdx], LOW);
      break;
    case LED_YELL:
      digitalWrite (stringPinA[stringIdx], HIGH);
      digitalWrite (stringPinB[stringIdx], LOW);
      break;
    case LED_BLUE:
      digitalWrite (stringPinA[stringIdx], LOW);
      digitalWrite (stringPinB[stringIdx], HIGH);
      break;
    case LED_BOTH:
      digitalWrite (stringPinA[stringIdx], !bothLedState);
      digitalWrite (stringPinB[stringIdx], bothLedState);
      break;
  }      
  
  if (stringIdx >= 3)
  {
    stringIdx = 0;
    bothLedState = (bothLedState == LOW ? HIGH : LOW);
  }
  else
  {
    stringIdx++;
  }
}
