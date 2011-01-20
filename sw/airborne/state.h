/*
 * Copyright (C) 2010 The Paparazzi Team
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
 */

/** \file state.h
 * \brief general inteface for the main vehicle states
 */

#ifndef STATE_H
#define STATE_H

#include "math/pprz_algebra_int.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_float.h"

/* abstract state interface */
struct State {
  /* Earth Centered Earth Fixed in centimeters */
  struct EcefCoor_i ecef_pos;

  /* lon, lat in radians*1e7  */
  /* alt in centimeters above MSL  */
  struct LlaCoor_i lla_pos;

  /* definition of the local (flat earth) coordinate system */
  struct LtpDef_i ltp_def;
  bool_t ltp_initialised;

  /* North East Down local tangent plane */
  struct NedCoor_i ltp_pos;
  struct NedCoor_i ltp_speed;
  struct NedCoor_i ltp_accel;

  /* vehicle attitude */
  struct Int32Quat   ltp_to_body_quat;
  struct Int32Eulers ltp_to_body_euler;
  struct Int32RMat   ltp_to_body_rmat;
  struct Int32Rates  body_rate;

  /* wind and airspeed*/
  struct Int32Vect3 airspeed;
  struct Int32Vect3 windspeed;
};

struct StateFloat {
  /* Position within UTM zone in meters, z in meters above MSL */
  struct FloatVect3 utm_pos;
  uint8_t utm_zone;
  /* altitude above ground level in meters */
  float alt_agl;

  /* accelerations in North East Down local tangent plane */
  struct FloatVect3 ltp_accel;

  /* speed in North East Down local tangent plane */
  struct FloatVect3 ltp_speed;
  /* horizontal ground speed in norm and dir (m/s, rad (CW/North)) */
  float hspeed_norm;
  float hspeed_dir;

  struct FloatVect2 windspeed; /* m/s ; x = north, y = east */
  float airspeed; /* m/s */

  struct FloatEulers ltp_to_body_euler;
  struct FloatRates  body_rate;
}

extern struct State state;
extern struct StateFloat state_float;

#endif /* STATE_H */
