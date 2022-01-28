#include <Arduino.h>

#ifndef RenardInterface_h
#define RenardInterface_h


void wait_for_serial();
int renardReadBytes( uint8_t *bytes, uint16_t bytes_size );
int renardRead( uint8_t *bytes, uint16_t byte_count );

#endif
