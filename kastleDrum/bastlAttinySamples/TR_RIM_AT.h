#ifndef TR_RIM_H_
#define TR_RIM_H_
 
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <avr/pgmspace.h>
 
#define TR_RIM_NUM_CELLS 98
#define TR_RIM_SAMPLERATE 4000
 
const char __attribute__((section(".progmem.data"))) TR_RIM_DATA [] = {22, 110,
127, 90, -90, -106, -87, -80, 73, 127, 99, -46, -112, -81, -90, -27, 119, 111,
108, 17, -105, -91, -84, -65, 42, 55, -63, -68, -68, -54, -50, 66, 127, 108,
101, 76, 61, 48, 53, 49, 42, 32, -36, -122, -112, -105, -76, -3, 46, 37, -2,
-49, -62, -51, -32, -26, -48, -65, -64, -57, -52, -43, -37, -26, -9, 13, 27, 48,
75, 92, 89, 82, 69, 53, 24, -3, -12, -9, 0, 5, 7, 4, 2, 4, 8, 10, 4, -12, -31,
-46, -50, -46, -38, -30, -23, -17, -11, -7, };
const char* sampleTable = TR_RIM_DATA;
const uint16_t sampleLength = TR_RIM_NUM_CELLS;
#endif /* 909_RIM_H_ */
