#ifndef HM5843_H
#define HM5843_H

#include "std.h"

#define HMC5843_UNINIT     0
#define HMC5843_IDLE       1
#define HMC5843_READ_XYZ   2

extern uint8_t hmc5843_status;
extern int16_t hmc5843_mag_x;
extern int16_t hmc5843_mag_y;
extern int16_t hmc5843_mag_z;
extern float hmc5843_mag_h;
extern bool_t hmc5843_available;

extern volatile bool_t hmc5843_i2c_done;

void hmc5843_init(void);
void hmc5843_periodic(void);
void hmc5843_heading( float roll, float pitch );

#endif
