/*
 * $Id: baro_MS5534A.c,v 1.2 2007/09/13 12:05:33 hecto Exp $
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

#include "airframe.h"
#include "interrupt_hw.h"
#include "init_hw.h"
#include "adc.h"
#include "datalink.h"
#ifdef LED
#include "led.h"
#endif

#include "baro_MP3H6115.h"

#ifdef ADC_CHANNEL_PRESSURE
static struct adc_buf buf_pressure;
#else
#error ADC_CHANNEL_PRESSURE nicht da !!!!
#endif

#ifdef ADC_CHANNEL_PRESSURE_OFF
static struct adc_buf buf_pressure_off;
#endif

#ifndef ADC_CHANNEL_PRESSURE_NB_SAMPLES
#define ADC_CHANNEL_PRESSURE_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

#ifndef ADC_CHANNEL_PRESSURE_OFF_NB_SAMPLES
#define ADC_CHANNEL_PRESSURE_OFF_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

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
#ifdef ADC_CHANNEL_PRESSURE
  adc_buf_channel( ADC_CHANNEL_PRESSURE, &buf_pressure,
		   ADC_CHANNEL_PRESSURE_NB_SAMPLES );
  adc_buf_channel( ADC_CHANNEL_PRESSURE_OFF, &buf_pressure_off,
		   ADC_CHANNEL_PRESSURE_OFF_NB_SAMPLES );
#endif
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
  float baro_bits = buf_pressure.sum/(float)buf_pressure.av_nb_sample;

  // calculate volt
  baro_MP3H6115_volt = bits2volt( baro_bits );
   
  // read offset bits
  float baro_off_bits = buf_pressure_off.sum/(float)buf_pressure_off.av_nb_sample;
  
  // keep offset voltage
  baro_MP3H6115_off_volt = bits2volt( baro_off_bits );

  // calculate ground pressure
  float baro_ground_pressure = volt2press( baro_MP3H6115_off_volt + baro_MP3H6115_volt / GAIN );

  // calculate ground height
  baro_MP3H6115_ground_height = press2height( baro_ground_pressure ) * BARO_SCALE;
}

void baro_MP3H6115_event_task( void ) {
  // pressure bits holen
  float baro_bits = buf_pressure.sum/(float)buf_pressure.av_nb_sample;

  // calculate volt
  baro_MP3H6115_volt = bits2volt( baro_bits );

  // calculate pressure
  float baro_pressure = volt2press( baro_MP3H6115_off_volt + baro_MP3H6115_volt / GAIN );
  
  // calculate height
  baro_MP3H6115_rel_height = press2height( baro_pressure ) * BARO_SCALE - baro_MP3H6115_ground_height;

  // set available
  baro_MP3H6115_available = TRUE;
}

// NOTE:
/*
berechnungen der werte stimmen, hoehenaenderung ist auf < 1m bestimmbar, aber ab und zu gibt es ausreisser.
der spannungswert ist etwas zu hoch, es wird ein druck von ~1041 hPa gemessen, NN ist 1013.25, druck in bremen
von 1017 zu der zeit hoehe wird dadurch als -230 meter angegeben
man muesste den sensor evtl kalibrieren, wenn die hoehenaenderung stimmt,
einfach den 0 punkt anpassen,
was man durch die rel. hoehe aber auch nicht braucht

die ausreisser peaks haben immer einen fast gleichen wert nach + bzw -
evtl. probleme bei internen berechnungen oder stoerung?
bei weiteren tests waren die reinen adc werte immer gut,
es gab auch phasen da traten keine peaks auf, meist am netzteil, mit batterie dann mehr peaks, aber dann auf einmal mit batterie weniger peaks

testweise hab ich die sensordaten als imu_gyro ans message fenster gegeben

2008-06-27 olri
Springen kam vom laufend neu gemessenen GROUND Wert mit 10 Bit. !!
wird nun einmal beim calibieren ermittelt und konstant weiter verwendet.

Die ubertragenen Werte liegen bei 14422 ? Habe ich da eine BUG reingebaut ?
Tendenzen sind ok, allerdings macht ein Stockwerk (5m) einen Sprung ca. 15 aus.

Bitte noch mal nachrechen.

2008-06-27 hesa
Bei der Spannungsberechnung in der event_task Funktion wurde ground_volt dazu addiert
und nicht die Offset-Spannung, evtl. behebt das den falschen Wert?
Habe auch noch die Bodenhoehe von der berechneten Hoehe abgezogen.
Die Steigung der Geradenfunktion müsste angepasst werden, denke nicht, dass die genau genug ist, da ja auch eine zu hohe Spannung gemessen wird,
Koennte mit einer Fahrstuhlfahrt gut klappen wenn wir die Hoehe genau genug kennen.
*/

