/*
 * Copyright (C) 2010  Hochschule Bremen
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

/** \file baro_scp.h
 *  \brief Handling of the SCP1000 pressure sensor
 *
 */

#ifndef BARO_SCP_H
#define BARO_SCP_H

#include "std.h"

extern bool_t alt_baro_enabled;
extern float baro_scp_rel_height;
extern float baro_scp_z;

#ifdef USE_BARO_SCP

#include "baro_scp_hw.h"

// Values for estimator
extern float baro_scp_r;
extern float baro_scp_sigma2;

// global variables
extern uint32_t baro_scp_ground_pressure;
extern float baro_scp_ground_height;

// functions
void baro_scp_calibrate( float h_ueber_nn);
float baro_scp_height( unsigned int Ph, float Th );

#endif // USE_BARO_SCP

#endif // BARO_SCP_H
