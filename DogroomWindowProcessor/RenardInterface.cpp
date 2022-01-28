#include "RenardInterface.h"

static bool renard_sync = false;

void wait_for_serial()
{
  while (Serial.available() == 0) 
  { 
  }
}

int renardReadBytes( uint8_t *bytes, uint16_t bytes_size )
{
  int in_byte = 0;
  int bytes_read;

  for ( bytes_read = 0; bytes_read < bytes_size; )
  {
    wait_for_serial();
    in_byte = Serial.read();

    switch (in_byte)
    {
      case(0x7E): // We saw the sync byte, start over!
        renard_sync = true;
        return bytes_read;

      case(0x7D): // Skip the pad byte
        continue;

      case(0x7F): // Escape character, we need to read one more byte to get our actual data
        wait_for_serial();
        in_byte = (byte) Serial.read();
        switch (in_byte)
        {
          case(0x2F): // renard wants an 0x7D
            in_byte = 0x7D;
          case(0x30): // renard wants an 0x7E
            in_byte = 0x7E;
          case(0x31): // renard wants an 0x7F
            in_byte = 0x7F;
        }
      }
      
    bytes[bytes_read++] = in_byte;
  }

  return bytes_read;
}

int renardRead( uint8_t *bytes, uint16_t byte_count )
{
  int in_byte = 0;

  while (renard_sync == false)
  {
    wait_for_serial();
    in_byte = (byte) Serial.read();
    if ( in_byte == 0x7E ) // Sync byte signifies start of packet
    {
      renard_sync = true;
    }
  }

  if (renard_sync == true)
  {
    renard_sync = false;
    wait_for_serial();
    in_byte = Serial.read();
    if (in_byte == 0x80) // Read from here
    {
      return renardReadBytes(bytes, byte_count);
    }
  }

  return 0;
}
