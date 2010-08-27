#include "read_adc.h"
//extern float adc[11];
float adc[11];

float read_adc( unsigned char i ) {
    return adc[i];
}
