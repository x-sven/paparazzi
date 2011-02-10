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

  /****** Int representations ******/

  /* Earth Centered Earth Fixed in centimeters */
  struct EcefCoor_i ecef_pos;

  /* lon, lat in radians*1e7  */
  /* alt in centimeters above MSL  */
  struct LlaCoor_i lla_pos;

  /* definition of the local (flat earth) coordinate system */
  struct LtpDef_i ned_origin;
  bool_t ned_initialised;

  /* North East Down local tangent plane */
  struct NedCoor_i ned_pos;
  struct NedCoor_i ned_speed;
  struct NedCoor_i ned_accel;

  /* vehicle attitude */
  struct Int32Quat   ned_to_body_quat;
  struct Int32Eulers ned_to_body_euler;
  struct Int32RMat   ned_to_body_rmat;
  struct Int32Rates  body_rate;

  /* horizontal windspeed x = north, y = east */
  struct Int32Vect2 h_windspeed;

  struct int32_t airspeed;


  /****** float representations ******/

  /* Position within UTM zone in meters, z in meters above MSL */
  struct FloatVect3 utm_pos;
  uint8_t utm_zone;
  /* altitude above ground level in meters */
  float alt_agl;

  /* accelerations in North East Down local tangent plane */
  struct FloatVect3 ned_accel;

  /* speed in North East Down local tangent plane */
  struct FloatVect3 ned_speed;
  /* horizontal ground speed in norm and dir (m/s, rad (CW/North)) */
  float h_speed_norm;
  float h_speed_dir;

  struct FloatVect2 h_windspeed; /* m/s ; x = north, y = east */
  float airspeed; /* m/s */

  /* vehicle attitude */
  struct FloatQuat   ned_to_body_quat;
  struct FloatEulers ned_to_body_euler;
  struct FloatRMat   ned_to_body_rmat;
  struct FloatRates  body_rate;


  /********** one time computation bookkeeping ********/
  
}

extern struct State state;


/*** functions to set state (int versions) ***/
inline void StateSetPositionEcef_i(EcefCoor_i ecef_pos);
inline void StateSetPositionNed_i(NedCoor_i ned_pos);
inline void StateSetPositionLla_i(LlaCoor_i lla_pos);

inline void StateSetSpeedNed_i(NedCoor_i ned_speed);
inline void StateSetAccelNed_i(NedCoor_i ned_accel);

inline void StateSetNedToBodyQuat_i(Int32Quat ned_to_body_quat);
inline void StateSetNedToBodyRMat_i(Int32RMat ned_to_body_rmat);
inline void StateSetNedToBodyEulers_i(Int32Eulers ned_to_body_eulers);
inline void StateSetBodyRates_i(Int32Rates body_rate);

inline void StateSetHorizontalWindspeed_i(Int32Vect2 h_windspeed);
inline void StateSetAirspeed_i(int32_t airspeed);

/*** functions to get state (int versions) ***/
inline EcefCoor_i StateGetPositionEcef_i(void);
inline NedCoor_i  StateGetPositionNed_i(void);
inline LlaCoor_i  StateGetPositionLla_i(void);

inline NedCoor_i StateGetSpeedNed_i(void);
inline NedCoor_i StateGetAccelNed_i(void);

inline Int32Quat   StateGetNedToBodyQuat_i(void);
inline Int32RMat   StateGetNedToBodyRMat_i(void);
inline Int32Eulers StateGetNedToBodyEulers_i(void);
inline Int32Rates  StateGetBodyRates_i(void);

inline Int32Vect2 StateGetHorizontalWindspeed_i(void);
inline int32_t StateGetAirspeed_i(void);


/*** functions to set state (float versions) ***/
inline void StateSetPositionUtm_f(FloatVect3 utm_pos);
//inline void StateSetPositionEcef_f(EcefCoor_f ecef_pos);
//inline void StateSetPositionNed_f(NedCoor_f ned_pos);
//inline void StateSetPositionLla_f(LlaCoor_f lla_pos);

inline void StateSetSpeedNed_f(NedCoor_f ned_speed);
inline void StateSetAccelNed_f(NedCoor_f ned_accel);

inline void StateSetNedToBodyQuat_f(FloatQuat ned_to_body_quat);
inline void StateSetNedToBodyRMat_f(FloatRMat ned_to_body_rmat);
inline void StateSetNedToBodyEulers_f(FloatEulers ned_to_body_eulers);
inline void StateSetBodyRates_f(FloatRates body_rate);

inline void StateSetHorizontalGroundSpeedNorm(float h_speed_norm); //a bit long, isn't it? returns h_speed_norm
inline void StateSetHorizontalGroundSpeedDirection(float h_speed_dir); //a bit long, isn't it? returns h_speed_dir
inline void StateSetHorizontalWindspeed(FloatVect2 h_windspeed);
inline void StateSetAirspeed(float airspeed);

/*** functions to get state (float versions) ***/
inline FloatVect3 StateSetPositionUtm_f(void);
//inline EcefCoor_f StateGetPositionEcef_f(void);
//inline NedCoor_f  StateGetPositionNed_f(void);
//inline LlaCoor_f  StateGetPositionLla_f(void);

inline NedCoor_f StateGetSpeedNed_f(void);
inline NedCoor_f StateGetAccelNed_f(void);

inline FloatQuat   StateGetNedToBodyQuat_f(void);
inline FloatRMat   StateGetNedToBodyRMat_f(void);
inline FloatEulers StateGetNedToBodyEulers_f(void);
inline FloatRates  StateGetBodyRates_f(void);

inline float StateGetHorizontalGroundSpeedNorm_f(void); //a bit long, isn't it? returns h_speed_norm
inline float StateGetHorizontalGroundSpeedDirection_f(void); //a bit long, isn't it? returns h_speed_dir
inline FloatVect2 StateGetHorizontalWindspeed_f(void);
inline float StateGetAirspeed_f(void);


#endif /* STATE_H */
