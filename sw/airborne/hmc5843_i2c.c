/** \file hmc5843_i2c.c
 *
 *  \brief Read the Honywell hmc5843 magnetometer on I2C sensor interface
 *
 *  This reads the values for X Y Z magnitude from the HMC5843 through I2C.
 *
 *  \author Oliver Riesener 2010
 */

#include <string.h>
#include <math.h>

#include "sys_time.h"
#include "i2c.h"
#include "led.h"

#include "hmc5843_i2c.h"

#define HMC5843_SLAVE_ADDR 0x3C

uint8_t hmc5843_status;
int16_t hmc5843_mag_x;
int16_t hmc5843_mag_y;
int16_t hmc5843_mag_z;
float  hmc5843_mag_h;

bool_t hmc5843_available;

volatile bool_t hmc5843_i2c_done;

typedef struct {
    unsigned short offset[3]; // x-, y-, z-offset
    unsigned short range[3]; //x-, y-, z- range
} hmc5843_param_t;

#if (AIRCRAFT == SAS)
#define MAX_X 650
#define MAX_Y 590
#define MAX_Z 580
#define MIN_X -570
#define MIN_Y -620
#define MIN_Z -530
#else
#warning no magent offset ranges
#define MAX_X 600
#define MAX_Y 600
#define MAX_Z 600
#define MIN_X -600
#define MIN_Y -600
#define MIN_Z -600
#endif


static hmc5843_param_t hmc5843_param = {
    { (MAX_X+MIN_X)/2,(MAX_Y+MIN_Y)/2,(MAX_Z+MIN_Z)/2 },
    { MAX_X-(MIN_X),MAX_Y-(MIN_Y),MAX_Z-(MAX_Z) }
};

/**
 * Initialises states from HMC5843 device
 *
 * \author Oliver Riesener
 *
 * \return hmc5843_status, hmc5843_i2c_done, hmc5843_available
 */
void hmc5843_init( void ) {
  hmc5843_status = HMC5843_UNINIT;
  hmc5843_i2c_done = FALSE;
  hmc5843_available = FALSE;
}

/**
 * reads magneto data from HMC5843 device, correct axis and sign
 *
 * return axis in airplane direction and notation
 *
 * \todo scaling is missing, incl. dice to begin
 * \return hmc5843_mag_[xyzh], hmc5843_available
 */
void hmc5843_periodic( void ) {
  if (hmc5843_status == HMC5843_UNINIT && cpu_time_sec > 1) {
    /* initialise device, write 0x00 to 0x01 */
    i2c0_buf[0] = 0x02; // mode register
    i2c0_buf[1] = 0x00; // cont. measurement mode
    hmc5843_i2c_done = FALSE;
    i2c0_transmit(HMC5843_SLAVE_ADDR, 2, &hmc5843_i2c_done);
    hmc5843_status = HMC5843_IDLE;
  } else {
    if (hmc5843_i2c_done) {
      if (hmc5843_status == HMC5843_IDLE) {
	// start 6+1 bytes read sequence 6 data + 1 status mode rr
	hmc5843_status = HMC5843_READ_XYZ;
	hmc5843_i2c_done = FALSE;
	i2c0_transceive(HMC5843_SLAVE_ADDR, 0, 6+1, &hmc5843_i2c_done);
      } else if ( hmc5843_status == HMC5843_READ_XYZ ) {
	// calculate the 3 answer values, switch axis and sign
	hmc5843_mag_y = -((i2c0_buf[0] << 8) | i2c0_buf[1]); // xh + xl
	hmc5843_mag_x = -((i2c0_buf[2] << 8) | i2c0_buf[3]); // yh + yl
	hmc5843_mag_z = -((i2c0_buf[4] << 8) | i2c0_buf[5]); // zh + zl
	// offset correction
	hmc5843_mag_x -= hmc5843_param.offset[0];
	hmc5843_mag_y -= hmc5843_param.offset[1];
	hmc5843_mag_z -= hmc5843_param.offset[2];
	// scaling to +/- 512
	hmc5843_mag_x *= (1024. / hmc5843_param.range[0]); 
	hmc5843_mag_y *= (1024. / hmc5843_param.range[1]); 
	hmc5843_mag_z *= (1024. / hmc5843_param.range[2]); 
	// set data avail
	hmc5843_available = TRUE;
	// next state
	hmc5843_status = HMC5843_IDLE;
	//debug	
	//LED_OFF(2);
      }
    } else {
      // debug no response
      hmc5843_mag_x = hmc5843_i2c_done;
      hmc5843_mag_y = hmc5843_status;
      hmc5843_mag_z++;
      // reinvoke
      // LED_ON(2);
      hmc5843_status == HMC5843_UNINIT;
    }
  }
}


/**
 * calculates actual heading from magnetic data
 *
 * \author Oliver Riesener
 * \param[in] roll actual roll angle
 * \param[in] pitch actual pitch angle
 *
 * \return hmc5843_mag_h
 */
void hmc5843_heading( float roll, float pitch ) {
  // speedup for cos/sin from roll
  float cos_roll = cos(roll);
  float sin_roll = sin(roll);
  // speedup for cos/sin from pitch
  float cos_pitch = cos(pitch);
  float sin_pitch = sin(pitch);
  // tilt compensated magnetic field x
  /** \todo tilt compensation wrong */
  float mag_x = 
    hmc5843_mag_x * cos_pitch + 
    hmc5843_mag_y * sin_roll * sin_pitch +
    hmc5843_mag_z * cos_roll * sin_pitch;
  // tilt compensated magnetic field y
  /** \todo tilt compensation wrong */
  float mag_y = hmc5843_mag_y*cos_roll-hmc5843_mag_z*sin_roll;
  // Magnetic Heading
  hmc5843_mag_h = atan2(mag_y,mag_x); // olri 2010-07-25 -mag_y
}
