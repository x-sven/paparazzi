#ifndef DAC_HW_H
#define DAC_HW_H

#include "LPC21xx.h"
#include BOARD_CONFIG

static inline void dac_init ( void ) {
  /* turn on DAC pins */
  PINSEL1 &= 1 << 19;
  PINSEL1 |= ~(1 << 18);
}

#define dac_update(x) DACR=((x)<<6)

#endif /* DAC_HW_H */
