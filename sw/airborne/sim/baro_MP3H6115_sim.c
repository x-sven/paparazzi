/*
 * $Id: baro_MP3H6115.c,v 1.2 2007/09/13 12:05:33 olri Exp $
 *  
 * Copyright (C) 2008  Hochschule Bremen
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */

/** \file baro_MP3H6115.c
 *  \brief Handling of the MP3H6115 pressure sensor
 *
 */
#include <math.h>

// #include "airframe.h"

#include <stdlib.h>
#include "estimator.h"
#include "nav.h"
#include "gps.h"
#include "baro_MP3H6115.h"

bool_t baro_MP3H6115_available;
bool_t alt_baro_enabled;
float baro_MP3H6115_volt;
float baro_MP3H6115_off_volt;
float baro_MP3H6115_ground_height;
float baro_MP3H6115_rel_height;

float baro_MP3H6115_r;
float baro_MP3H6115_sigma2;
float baro_MP3H6115_z;

/*
 * baro_MP3H6115_init(): setup adc buffers and baro_MP3H6115_calibrate()
 */
void baro_MP3H6115_init( void ) {
  baro_MP3H6115_r = 20.;
  baro_MP3H6115_sigma2 = 1;
  baro_MP3H6115_available = FALSE;
  alt_baro_enabled = FALSE;
  baro_MP3H6115_ground_height = 0.0;
}
#ifndef BARO_SCALE
#define BARO_SCALE 1.5
#endif

#define bits2volt(bits) ((3.3/1024.0)*bits)
//#define volt2press(volt) ((volt + 0.3547)*360) // 2.468V = 1013.25mBar
// x=(y-b)/m, x(kPa)=x(hPa)*10
#define volt2press(volt) ((volt + 0.28214)*36.84*10) // 2.468V = 101.3KPa, 1013.25mBar 
#define press2height(press) (TEMP_PER_GRADIENT * (1 - pow(press / ATMO_PRESSURE, 0.19)))
// GAIN=390K/(1395+((2400+167)||(12000+833)))+1; Bei Abgleich 2.5 Volt Poti 167+833 = 1K
#define GAIN 104 + 1 // Mittelwert aus 95-115 je nach Poti-Einstellung !!!! Es fehlt ein Impedanzwandler !!!

/*
 * baro_MO3H6115_calibrate(): should be called at ground, zero meters 
 */
void baro_MP3H6115_calibrate( void ) {
  // read bits 
  float baro_bits = 1024/3.3*(3.0-2.5/100*((gps_alt/100.-ground_alt) )); // + ((10.*random()) / RAND_MAX)));

  // calculate volt
  baro_MP3H6115_volt = bits2volt( baro_bits );
   
  // read offset bits
  float baro_off_bits = 1024/3.3*2.5123;
  
  // keep offset voltage
  baro_MP3H6115_off_volt = bits2volt( baro_off_bits );

  // calculate ground pressure
  float baro_ground_pressure = volt2press( baro_MP3H6115_off_volt + baro_MP3H6115_volt / GAIN );

  // calculate ground height
  baro_MP3H6115_ground_height = press2height( baro_ground_pressure ) * BARO_SCALE;
}

void baro_MP3H6115_event_task( void ) {
  // pressure bits holen
  float baro_bits = 1024/3.3*(3.0-2.5/100*((gps_alt/100.-ground_alt) )); //+ ((10.*random()) / RAND_MAX)));

  // calculate volt
  baro_MP3H6115_volt = bits2volt( baro_bits );

  // calculate pressure
  float baro_pressure = volt2press( baro_MP3H6115_off_volt + baro_MP3H6115_volt / GAIN );
  
  // calculate height
  baro_MP3H6115_rel_height = press2height( baro_pressure ) * BARO_SCALE  - baro_MP3H6115_ground_height;

  // set available
  baro_MP3H6115_available = TRUE;
}
