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

#include "baro_nobaro.h"

bool_t alt_baro_enabled;
