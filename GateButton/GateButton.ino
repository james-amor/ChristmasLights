

//================================================================================================
const byte ACTIVE_PIN = 13;
const byte NUM_ILLUM_OUTPUTS = 5;
const byte ILLUM_PINS[NUM_ILLUM_OUTPUTS] = {9, 8, 7, 6, 5};

//================================================================================================
void setup() 
{
  Serial.begin(9600);
  Serial.println ("Starting");

  pinMode (ACTIVE_PIN, OUTPUT);
  digitalWrite (ACTIVE_PIN, HIGH);
  
  for (byte idx = 0; idx < NUM_ILLUM_OUTPUTS; idx++)
  {
    pinMode (ILLUM_PINS[idx], OUTPUT);
    digitalWrite (ILLUM_PINS[idx], LOW);
  }  
} 
//================================================================================================
void loop() 
{  
  updateLightshow();
}
//================================================================================================
void updateLightshow(void)
{
  const byte PATTERN_LEN = 11;
  const byte PATTERN [NUM_ILLUM_OUTPUTS][PATTERN_LEN] =
    {{B01000010, B00010000, B10101011, B00000001, B00000001, B11111111, B10000100, B01001011, B11110111, B10111101, B01010101 },
     {B00100001, B00001000, B10101010, B10000010, B10000010, B10101010, B01000010, B00100111, B11101111, B01111011, B10101010 },
     {B00010000, B10000100, B10101010, B01000100, B01000100, B10101010, B00100001, B00011111, B11011110, B11110111, B01010101 },
     {B10001000, B01000010, B10101010, B00101000, B00101000, B01010101, B00010000, B11111111, B10111101, B11101111, B10101010 },
     {B00000100, B00100001, B10101010, B00010000, B00010000, B01010101, B00001111, B11111111, B01111011, B11011111, B01010101 }};

  static unsigned long lastUpdate = 0;
  static byte currentByte = 0;
  static byte currentBit = 0;

  if ((millis() - lastUpdate) < 300) //350)
  {
    return;
  }
  lastUpdate = millis();

  for (int idx = 0; idx < NUM_ILLUM_OUTPUTS; idx++)
  {
    if ((PATTERN[idx][currentByte] & (128 >> currentBit)) == 0)
    {
      digitalWrite (ILLUM_PINS[idx], LOW);  
    }
    else
    {
      digitalWrite (ILLUM_PINS[idx], HIGH);      
    }
  }
  
  if (currentBit < 7)
  {
    currentBit++;
  }
  else
  {
    currentBit = 0;    
    currentByte++;
    if (currentByte >= PATTERN_LEN)
    { 
      currentByte = 0;
    }
    digitalWrite (ACTIVE_PIN, (digitalRead(ACTIVE_PIN) == LOW ? HIGH : LOW));
  }  
}
//================================================================================================
