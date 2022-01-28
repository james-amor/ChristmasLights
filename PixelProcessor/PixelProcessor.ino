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


#include <Adafruit_NeoPixel.h>
#include "RenardInterface.h"

#define PIXEL_LED_COUNT 50
#define MESSAGE_FEEDBACK_PIN 13
#define PIXEL_DATA_PIN 2

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_LED_COUNT, PIXEL_DATA_PIN, NEO_RGB + NEO_KHZ400);  


void setup()
{
  Serial.begin(57600);
  delay(10);

  pinMode(MESSAGE_FEEDBACK_PIN, OUTPUT);
  digitalWrite (MESSAGE_FEEDBACK_PIN, HIGH);

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  delay(100);

  
  for (byte testPixel = 0; testPixel < PIXEL_LED_COUNT; testPixel++)
  {
    for (byte idx = 0; idx < PIXEL_LED_COUNT; idx++)
    {
      if (idx == testPixel)
      {
        strip.setPixelColor(idx, strip.Color(255, 255, 255));
      }
      else
      {
        strip.setPixelColor(idx, strip.Color(0,0,0));      
      }
    }  

    strip.show();
    delay(20);      
  }

  delay(500);      

  for (byte idx = 0; idx < PIXEL_LED_COUNT; idx++)
  {
    strip.setPixelColor(idx, strip.Color(255, 0, 0));
  }
  strip.show();
  delay(500);      

  for (byte idx = 0; idx < PIXEL_LED_COUNT; idx++)
  {
    strip.setPixelColor(idx, strip.Color(0, 255, 0));
  }
  strip.show();
  delay(500);      

  for (byte idx = 0; idx < PIXEL_LED_COUNT; idx++)
  {
    strip.setPixelColor(idx, strip.Color(0, 0, 255));
  }
  strip.show();
  delay(500);      
  
  for (byte idx = 0; idx < PIXEL_LED_COUNT; idx++)
  {
    strip.setPixelColor(idx, strip.Color(255, 255, 255));
  }
  strip.show();
  delay(500);      
  
  for (byte idx = 0; idx < PIXEL_LED_COUNT; idx++)
  {
    strip.setPixelColor(idx, strip.Color(0, 0, 0));
  }
  strip.show();
  delay(500);      
    
  Serial.println ("Started");
  digitalWrite (MESSAGE_FEEDBACK_PIN, LOW);
}

void loop()
{
  uint8_t bytes[200];
  uint16_t bytes_read;
  bytes_read = renardRead(&bytes[0], 150);

  if ( bytes_read == 150 )
  {
    //Now process the channel info into specifics
    byte red_val = 0;
    byte green_val = 0;
    byte blue_val = 0;

    //First half of the string get output in order
    for (byte pixel = 0; pixel < (PIXEL_LED_COUNT / 2); pixel ++)
    { 
      red_val = bytes[(pixel * 3)];
      green_val = bytes[(pixel * 3) + 1];
      blue_val = bytes[(pixel * 3) + 2];
      
      strip.setPixelColor(pixel, strip.Color(red_val, green_val, blue_val));
    }

    //Second half of the string get output in reverse order 
    for (byte pixel = 0; pixel < (PIXEL_LED_COUNT / 2); pixel ++)
    { 
      red_val = bytes[(PIXEL_LED_COUNT * 3) - ((pixel * 3) + 3)];
      green_val = bytes[(PIXEL_LED_COUNT * 3) - ((pixel * 3) + 2)];
      blue_val = bytes[(PIXEL_LED_COUNT * 3) - ((pixel * 3) + 1)];
      
      strip.setPixelColor(pixel + (PIXEL_LED_COUNT / 2), strip.Color(red_val, green_val, blue_val));
    }
    
    strip.show();

    digitalWrite (MESSAGE_FEEDBACK_PIN, (digitalRead(MESSAGE_FEEDBACK_PIN) == LOW ? HIGH : LOW));     
  }
  else
  {
    Serial.print ("X ");
    Serial.print (bytes_read);
    Serial.println (" X");
  }
}
