#ifndef GL_B_H_
#define GL_B_H_
 
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <avr/pgmspace.h>
 
#define GL_B_NUM_CELLS 73
#define GL_B_SAMPLERATE 16384
 
const char __attribute__((section(".progmem.data"))) GL_B_DATA [] = {22, -1, 2,
-21, -3, 5, 9, 15, -48, -15, 68, 16, -59, -47, 24, 57, 16, -13, -69, -39, 46,
57, -19, -99, 2, 87, 30, -47, -75, -10, 62, 60, -30, -74, -5, 85, 54, -17, -51,
2, 6, -15, -26, -51, -31, -16, 4, -22, -19, 7, 37, 69, 40, 4, 8, 11, 27, -7, -9,
-18, -32, -38, -23, -4, -19, 2, 18, 12, -9, 1, -6, 13, 31, };
const char* sampleTable = GL_B_DATA;
const uint16_t sampleLength = GL_B_NUM_CELLS;
#endif /* GL_B_H_ */
