#include "ppm.h"
#include "std.h"
#include "sys_time.h"

#ifndef PPM_NB_CHANNEL
#define PPM_NB_CHANNEL PPM_NB_PULSES
#endif

uint16_t ppm_pulses[PPM_NB_CHANNEL];
volatile bool_t ppm_valid;


