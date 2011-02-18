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
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"

#include "std.h"
#include <string.h>

#define POS_ECEF_I 1<<0
#define POS_NED_I  1<<1
#define POS_LLA_I  1<<2
#define POS_UTM_I  1<<3
#define POS_ECEF_F 1<<4
#define POS_NED_F  1<<5
#define POS_LLA_F  1<<6
#define POS_UTM_F  1<<7

#define ATT_QUAT_I  1<<0
#define ATT_EULER_I 1<<1
#define ATT_RMAT_I  1<<2
#define ATT_QUAT_F  1<<3
#define ATT_EULER_F 1<<4
#define ATT_RMAT_F  1<<5


/* abstract state interface */
struct State {

  /****** Int representations ******/

  /* Earth Centered Earth Fixed in centimeters */
  struct EcefCoor_i ecef_pos_i;

  /* lon, lat in radians*1e7  */
  /* alt in centimeters above MSL  */
  struct LlaCoor_i lla_pos_i;

  /* definition of the local (flat earth) coordinate system */
  struct LtpDef_i ned_origin_i;
  bool_t ned_initialised_i;

  /* North East Down local tangent plane */
  struct NedCoor_i ned_pos_i;
  struct NedCoor_i ned_speed_i;
  struct NedCoor_i ned_accel_i;

  /* vehicle attitude */
  struct Int32Quat   ned_to_body_quat_i;
  struct Int32Eulers ned_to_body_eulers_i;
  struct Int32RMat   ned_to_body_rmat_i;
  struct Int32Rates  body_rate_i;

  /* horizontal windspeed x = north, y = east */
  struct Int32Vect2 h_windspeed_i;

  int32_t airspeed_i;


  /****** float representations ******/

  /* Position within UTM zone in meters, z in meters above MSL */
  struct FloatVect3 utm_pos_f;
  uint8_t utm_zone_f;
  /* altitude above ground level in meters */
  float alt_agl_f;

  /* accelerations in North East Down local tangent plane */
  struct FloatVect3 ned_accel_f;

  /* speed in North East Down local tangent plane */
  struct FloatVect3 ned_speed_f;
  /* horizontal ground speed in norm and dir (m/s, rad (CW/North)) */
  float h_speed_norm_f;
  float h_speed_dir_f;

  struct FloatVect2 h_windspeed_f; /* m/s ; x = north, y = east */
  float airspeed_f; /* m/s */

  /* vehicle attitude */
  struct FloatQuat   ned_to_body_quat_f;
  struct FloatEulers ned_to_body_eulers_f;
  struct FloatRMat   ned_to_body_rmat_f;
  struct FloatRates  body_rate_f;


  /********** one time computation bookkeeping ********/
  uint8_t pos_status;
  uint8_t att_status;

};

extern struct State state;


/*** functions to set state (int versions) ***/
inline void StateSetPositionEcef_i(struct EcefCoor_i* ecef_pos);
inline void StateSetPositionNed_i(struct NedCoor_i* ned_pos);
inline void StateSetPositionLla_i(struct LlaCoor_i* lla_pos);

inline void StateSetSpeedNed_i(struct NedCoor_i* ned_speed);
inline void StateSetAccelNed_i(struct NedCoor_i* ned_accel);

inline void StateSetNedToBodyQuat_i(struct Int32Quat* ned_to_body_quat);
inline void StateSetNedToBodyRMat_i(struct Int32RMat* ned_to_body_rmat);
inline void StateSetNedToBodyEulers_i(struct Int32Eulers* ned_to_body_eulers);
inline void StateSetBodyRates_i(struct Int32Rates* body_rate);

inline void StateSetHorizontalWindspeed_i(struct Int32Vect2* h_windspeed);
inline void StateSetAirspeed_i(int32_t* airspeed);

/*** functions to get state (int versions) ***/
inline struct EcefCoor_i* StateGetPositionEcef_i(void);
inline struct NedCoor_i*  StateGetPositionNed_i(void);
inline struct LlaCoor_i*  StateGetPositionLla_i(void);

inline struct NedCoor_i* StateGetSpeedNed_i(void);
inline struct NedCoor_i* StateGetAccelNed_i(void);

inline struct Int32Quat*   StateGetNedToBodyQuat_i(void);
inline struct Int32RMat*   StateGetNedToBodyRMat_i(void);
inline struct Int32Eulers* StateGetNedToBodyEulers_i(void);
inline struct Int32Rates*  StateGetBodyRates_i(void);

inline struct Int32Vect2* StateGetHorizontalWindspeed_i(void);
inline int32_t* StateGetAirspeed_i(void);


/*** functions to set state (float versions) ***/
inline void StateSetPositionUtm_f(struct FloatVect3* utm_pos);
//inline void StateSetPositionEcef_f(struct EcefCoor_f* ecef_pos);
//inline void StateSetPositionNed_f(struct NedCoor_f* ned_pos);
//inline void StateSetPositionLla_f(struct LlaCoor_f* lla_pos);

inline void StateSetSpeedNed_f(struct NedCoor_f* ned_speed);
inline void StateSetAccelNed_f(struct NedCoor_f* ned_accel);

inline void StateSetNedToBodyQuat_f(struct FloatQuat* ned_to_body_quat);
inline void StateSetNedToBodyRMat_f(struct FloatRMat* ned_to_body_rmat);
inline void StateSetNedToBodyEulers_f(struct FloatEulers* ned_to_body_eulers);
inline void StateSetBodyRates_f(struct FloatRates* body_rate);

inline void StateSetHorizontalGroundSpeedNorm(float* h_speed_norm); //a bit long, isn't it? returns h_speed_norm
inline void StateSetHorizontalGroundSpeedDirection(float* h_speed_dir); //a bit long, isn't it? returns h_speed_dir
inline void StateSetHorizontalWindspeed(struct FloatVect2* h_windspeed);
inline void StateSetAirspeed(float* airspeed);

/*** functions to get state (float versions) ***/
inline struct FloatVect3* StateGetPositionUtm_f(void);
//inline struct EcefCoor_f* StateGetPositionEcef_f(void);
//inline struct NedCoor_f*  StateGetPositionNed_f(void);
//inline struct LlaCoor_f*  StateGetPositionLla_f(void);

inline struct NedCoor_f* StateGetSpeedNed_f(void);
inline struct NedCoor_f* StateGetAccelNed_f(void);

inline struct FloatQuat*   StateGetNedToBodyQuat_f(void);
inline struct FloatRMat*   StateGetNedToBodyRMat_f(void);
inline struct FloatEulers* StateGetNedToBodyEulers_f(void);
inline struct FloatRates*  StateGetBodyRates_f(void);

inline float* StateGetHorizontalGroundSpeedNorm_f(void); //a bit long, isn't it? returns h_speed_norm
inline float* StateGetHorizontalGroundSpeedDirection_f(void); //a bit long, isn't it? returns h_speed_dir
inline struct FloatVect2* StateGetHorizontalWindspeed_f(void);
inline float* StateGetAirspeed_f(void);


#endif /* STATE_H */
