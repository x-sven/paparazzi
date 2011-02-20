/*
 * Copyright (C) 2011 The Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file state.h
 *   @brief General inteface for the main vehicle states.
 *
 *   This is the more detailed description of this file.
 *
 *   @author Felix Ruess <felix.ruess@gmail.com>
 *
 */

#ifndef STATE_H
#define STATE_H

#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"

#include "std.h"
#include <string.h>

/**
 * @defgroup PosGroup position representations
 * @{
 */
#define POS_ECEF_I 1<<0
#define POS_NED_I  1<<1
#define POS_LLA_I  1<<2
#define POS_UTM_I  1<<3
#define POS_ECEF_F 1<<4
#define POS_NED_F  1<<5
#define POS_LLA_F  1<<6
#define POS_UTM_F  1<<7
/**@}*/

/**
 * @defgroup SpeedGroup ground-speed representations
 * @{
 */
#define SPEED_ECEF_I  1<<0
#define SPEED_NED_I   1<<1
#define SPEED_HNORM_I 1<<2
#define SPEED_HDIR_I  1<<3
#define SPEED_ECEF_F  1<<4
#define SPEED_NED_F   1<<5
#define SPEED_HNORM_F 1<<6
#define SPEED_HDIR_F  1<<7
/**@}*/

/**
 * @defgroup AccelGroup acceleration representations
 * @{
 */
#define ACCEL_ECEF_I 1<<0
#define ACCEL_NED_I  1<<1
#define ACCEL_ECEF_F 1<<3
#define ACCEL_NED_F  1<<4
/**@}*/

/**
 * @defgroup AttGroup attitude representations
 * @{
 */
#define ATT_QUAT_I  1<<0
#define ATT_EULER_I 1<<1
#define ATT_RMAT_I  1<<2
#define ATT_QUAT_F  1<<3
#define ATT_EULER_F 1<<4
#define ATT_RMAT_F  1<<5
/**@}*/

/**
 * @defgroup RateGroup angular rate representations
 * @{
 */
#define RATE_I 1<<0
#define RATE_F 1<<1
/**@}*/

/**
 * @defgroup WindAirGroup wind- and airspeed representations
 * @{
 */
#define WINDSPEED_I 1<<0
#define AIRSPEED_I  1<<1
#define WINDSPEED_F 1<<2
#define AIRSPEED_F  1<<3
/**@}*/


/**
 * @brief structure holding vehicle state data
 */
struct State {

  /*************************************
   * Int representations
   ************************************/

  /**
   * @brief position in EarthCenteredEarthFixed coordinates
   * @details Units: centimeters */
  struct EcefCoor_i ecef_pos_i;

  /**
   * @brief speed in EarthCenteredEarthFixed coordinates
   * @details Units: m/s in BFP with INT32_SPEED_FRAC */
  struct EcefCoor_i ecef_speed_i;

  /**
   * @brief acceleration in EarthCenteredEarthFixed coordinates
   * @details Units: m/s^2 in BFP with INT32_ACCEL_FRAC */
  struct EcefCoor_i ecef_accel_i;

  /**
   * @brief position in Latitude, Longitude and Altitude
   * @details Units lat,lon: radians*1e7
   * Units alt: centimeters above MSL
   */
  struct LlaCoor_i lla_pos_i;

  /**
   * @brief definition of the local (flat earth) coordinate system
   * @details Defines the origin of the local coordinate system
   * in ECEF and LLA coordinates and the roation matrix from
   * ECEF to local frame */
  struct LtpDef_i ned_origin_i;

  /**
   * @brief true if local int coordinate frame is initialsed */
  bool_t ned_initialised_i;

  /**
   * @brief position in North East Down coordinates
   * @details @details with respect to ned_origin_i (flat earth)
   * Units: m in BFP with INT32_POS_FRAC */
  struct NedCoor_i ned_pos_i;

  /**
   * @brief speed in North East Down coordinates
   * @details Units: m/s in BFP with INT32_SPEED_FRAC */
  struct NedCoor_i ned_speed_i;

  /**
   * @brief acceleration in North East Down coordinates
   * @details @details Units: m/s^2 in BFP with INT32_ACCEL_FRAC */
  struct NedCoor_i ned_accel_i;

  /**
   * @brief norm of horizontal ground speed
   * @details Units: m/s in BFP with INT32_SPEED_FRAC */
  int32_t h_speed_norm_i;

  /**
   * @brief dir of horizontal ground speed
   * @details Units: rad in BFP with INT32_ANGLE_FRAC */
  int32_t h_speed_dir_i;

  /**
   * @brief horizontal windspeed in north/east
   * @details Units: m/s in BFP with INT32_SPEED_FRAC */
  struct Int32Vect2 h_windspeed_i;

  /**
   * @brief norm of horizontal ground speed
   * @details Units: m/s in BFP with INT32_SPEED_FRAC */
  int32_t airspeed_i;

  /**
   * @brief attitude as quaternion
   * @details Specifies rotation from local NED frame to body frame.
   * Units: INT32_QUAT_FRAC
   *
   * @code
   * struct Int32Vect3 body_accel;
   * INT32_QUAT_VMULT(body_accel, StateGetNedToBodyQuat_i(), StateGetAccelNed_i());
   * @endcode
   */
  struct Int32Quat ned_to_body_quat_i;

  /**
   * @brief attitude in zyx euler angles
   * @details Specifies rotation from local NED frame to body frame.
   * Units: rad in BFP with INT32_ANGLE_FRAC */
  struct Int32Eulers ned_to_body_eulers_i;

  /**
   * @brief attitude rotation matrix
   * @details Specifies rotation from local NED frame to body frame.
   * Units: rad in BFP with INT32_TRIG_FRAC */
  struct Int32RMat   ned_to_body_rmat_i;

  /**
   * @brief angular rates in body frame
   * @details Units: rad/s^2 in BFP with INT32_RATE_FRAC */
  struct Int32Rates  body_rates_i;


  /*************************************
   * float representations
   ************************************/

  /**
   * @brief position in UTM coordinates
   * @details Units x,y: meters.
   * Units z: meters above MSL */
  struct FloatVect3 utm_pos_f;

  /**
   * @brief UTM zone number */
  uint8_t utm_zone_f;

  /**
   * @brief altitude above ground level
   * @details Unit: meters */
  float alt_agl_f;

  /**
   * @brief position in Latitude, Longitude and Altitude
   * @details Units lat,lon: radians
   * Units alt: meters above MSL
   */
  struct LlaCoor_f lla_pos_f;

  /**
   * @brief position in EarthCenteredEarthFixed coordinates
   * @details Units: meters */
  struct EcefCoor_f ecef_pos_f;

  /**
   * @brief speed in EarthCenteredEarthFixed coordinates
   * @details Units: m/s */
  struct EcefCoor_f ecef_speed_f;

  /**
   * @brief acceleration in EarthCenteredEarthFixed coordinates
   * @details Units: m/s^2 */
  struct EcefCoor_f ecef_accel_f;

  /**
   * @brief definition of the local (flat earth) coordinate system
   * @details Defines the origin of the local coordinate system
   * in ECEF and LLA coordinates and the roation matrix from
   * ECEF to local frame */
  struct LtpDef_f ned_origin_f;

  /**
   * @brief true if local float coordinate frame is initialsed */
  bool_t ned_initialised_f;

  /**
   * @brief position in North East Down coordinates
   * @details @details with respect to ned_origin_i (flat earth)
   * Units: meters */
  struct NedCoor_f ned_pos_f;

  /**
   * @brief speed in North East Down coordinates
   * @details Units: m/s */
  struct NedCoor_f ned_speed_f;

  /**
   * @brief acceleration in North East Down coordinates
   * @details Units: m/s^2 */
  struct NedCoor_f ned_accel_f;

  /**
   * @brief norm of horizontal ground speed
   * @details Units: m/s */
  float h_speed_norm_f;

  /**
   * @brief dir of horizontal ground speed
   * @details Units: rad (clockwise, zero=north)*/
  float h_speed_dir_f;

  /**
   * @brief horizontal windspeed
   * @details Units: m/s with x=north, y=east */
  struct FloatVect2 h_windspeed_f;

  /**
   * @brief norm of horizontal ground speed
   * @details Units: m/s */
  float airspeed_f;

  /**
   * @brief attitude as quaternion
   * @details Specifies rotation from local NED frame to body frame.
   * Units: unit length
   *
   * @code
   * struct FloatVect3 body_accel;
   * FLOAT_QUAT_VMULT(body_accel, StateGetNedToBodyQuat_f(), StateGetAccelNed_f());
   * @endcode
   */
  struct FloatQuat   ned_to_body_quat_f;

  /**
   * @brief attitude in zyx euler angles
   * @details Specifies rotation from local NED frame to body frame.
   * Units: rad */
  struct FloatEulers ned_to_body_eulers_f;

  /**
   * @brief attitude rotation matrix
   * @details Specifies rotation from local NED frame to body frame.
   * Units: rad */
  struct FloatRMat   ned_to_body_rmat_f;

  /**
   * @brief angular rates in body frame
   * @details Units: rad/s^2 */
  struct FloatRates  body_rates_f;


  /********** one time computation bookkeeping ********/

  /**
   * @brief holds the status bits for all position representations
   * @details When the corresponding bit is one the representation
   * is already computed. */
  uint8_t pos_status;

  /**
   * @brief holds the status bits for all ground speed representations
   * @details When the corresponding bit is one the representation
   * is already computed. */
  uint8_t speed_status;

  /**
   * @brief holds the status bits for all acceleration representations
   * @details When the corresponding bit is one the representation
   * is already computed. */
  uint8_t accel_status;

  /**
   * @brief holds the status bits for all attitude representations
   * @details When the corresponding bit is one the representation
   * is already computed. */
  uint8_t att_status;

  /**
   * @brief holds the status bits for all angular rate representations
   * @details When the corresponding bit is one the representation
   * is already computed. */
  uint8_t rate_status;

  /**
   * @brief holds the status bits for all wind- and airspeed representations
   * @details When the corresponding bit is one the representation
   * is already computed. */
  uint8_t wind_air_status;

};

extern struct State state;

/*******************************************************************
 * Set State functions (int versions)
 *******************************************************************/

/** @brief Set position from ECEF coordinates (int). */
inline void StateSetPositionEcef_i(struct EcefCoor_i* ecef_pos);

/** @brief Set position from local NED coordinates (int). */
inline void StateSetPositionNed_i(struct NedCoor_i* ned_pos);

/** @brief Set position from LLA coordinates (int). */
inline void StateSetPositionLla_i(struct LlaCoor_i* lla_pos);

/** @brief Set ground speed in local NED coordinates (int). */
inline void StateSetSpeedNed_i(struct NedCoor_i* ned_speed);

/** @brief Set ground speed in ECEF coordinates (int). */
inline void StateSetSpeedEcef_i(struct EcefCoor_i* ecef_speed);

/** @brief Set acceleration in NED coordinates (int). */
inline void StateSetAccelNed_i(struct NedCoor_i* ned_accel);

/** @brief Set acceleration in ECEF coordinates (int). */
inline void StateSetAccelEcef_i(struct EcefCoor_i* ecef_accel);

/** @brief Set vehicle body attitude from quaternion (int). */
inline void StateSetNedToBodyQuat_i(struct Int32Quat* ned_to_body_quat);

/** @brief Set vehicle body attitude from rotation matrix (int). */
inline void StateSetNedToBodyRMat_i(struct Int32RMat* ned_to_body_rmat);

/** @brief Set vehicle body attitude from euler angles (int). */
inline void StateSetNedToBodyEulers_i(struct Int32Eulers* ned_to_body_eulers);

/** @brief Set vehicle body angular rate (int). */
inline void StateSetBodyRates_i(struct Int32Rates* body_rate);

/** @brief Set horizontal windspeed (int). */
inline void StateSetHorizontalWindspeed_i(struct Int32Vect2* h_windspeed);

/** @brief Set airspeed (int). */
inline void StateSetAirspeed_i(int32_t* airspeed);


/*******************************************************************
 * Get State functions (int versions)
 *******************************************************************/

/** @brief Get position in ECEF coordinates (int). */
inline struct EcefCoor_i StateGetPositionEcef_i(void);

/** @brief Get position in local NED coordinates (int). */
inline struct NedCoor_i StateGetPositionNed_i(void);

/** @brief Get position in LLA coordinates (int). */
inline struct LlaCoor_i StateGetPositionLla_i(void);

/** @brief Get ground speed in local NED coordinates (int). */
inline struct NedCoor_i StateGetSpeedNed_i(void);

/** @brief Get ground speed in ECEF coordinates (int). */
inline struct EcefCoor_i StateGetSpeedEcef_i(void);

/** @brief Get norm of horizontal ground speed (int). */
inline int32_t StateGetHorizontalSpeedNorm_i(void);

/** @brief Get dir of horizontal ground speed (int). */
inline int32_t StateGetHorizontalSpeedDir_i(void);

/** @brief Get acceleration in NED coordinates (int). */
inline struct NedCoor_i StateGetAccelNed_i(void);

/** @brief Get acceleration in ECEF coordinates (int). */
inline struct EcefCoor_i StateGetAccelEcef_i(void);

/** @brief Get vehicle body attitude quaternion (int). */
inline struct Int32Quat StateGetNedToBodyQuat_i(void);

/** @brief Get vehicle body attitude rotation matrix (int). */
inline struct Int32RMat StateGetNedToBodyRMat_i(void);

/** @brief Get vehicle body attitude euler angles (int). */
inline struct Int32Eulers StateGetNedToBodyEulers_i(void);

/** @brief Get vehicle body angular rate (int). */
inline struct Int32Rates StateGetBodyRates_i(void);

/** @brief Get horizontal windspeed (int). */
inline struct Int32Vect2 StateGetHorizontalWindspeed_i(void);

/** @brief Get airspeed (int). */
inline int32_t StateGetAirspeed_i(void);

/*******************************************************************
 * Set State functions (float versions)
 *******************************************************************/

/** @brief Set position from UTM coordinates (float). */
inline void StateSetPositionUtm_f(struct FloatVect3* utm_pos);

/** @brief Set position from ECEF coordinates (float). */
inline void StateSetPositionEcef_f(struct EcefCoor_f* ecef_pos);

/** @brief Set position from local NED coordinates (float). */
inline void StateSetPositionNed_f(struct NedCoor_f* ned_pos);

/** @brief Set position from LLA coordinates (float). */
inline void StateSetPositionLla_f(struct LlaCoor_f* lla_pos);

/** @brief Set ground speed in local NED coordinates (float). */
inline void StateSetSpeedNed_f(struct NedCoor_f* ned_speed);

/** @brief Set ground speed in ECEF coordinates (float). */
inline void StateSetSpeedEcef_f(struct EcefCoor_f* ecef_speed);

/** @brief Set acceleration in NED coordinates (float). */
inline void StateSetAccelNed_f(struct NedCoor_f* ned_accel);

/** @brief Set acceleration in ECEF coordinates (float). */
inline void StateSetAccelEcef_f(struct EcefCoor_f* ecef_accel);

/** @brief Set vehicle body attitude from quaternion (float). */
inline void StateSetNedToBodyQuat_f(struct FloatQuat* ned_to_body_quat);

/** @brief Set vehicle body attitude from rotation matrix (float). */
inline void StateSetNedToBodyRMat_f(struct FloatRMat* ned_to_body_rmat);

/** @brief Set vehicle body attitude from euler angles (float). */
inline void StateSetNedToBodyEulers_f(struct FloatEulers* ned_to_body_eulers);

/** @brief Set vehicle body angular rate (float). */
inline void StateSetBodyRates_f(struct FloatRates* body_rate);

/** @brief Set horizontal windspeed (float). */
inline void StateSetHorizontalWindspeed_f(struct FloatVect2* h_windspeed);

/** @brief Set airspeed (float). */
inline void StateSetAirspeed_f(float* airspeed);


/*******************************************************************
 * Get State functions (float versions)
 *******************************************************************/

/** @brief Get position in UTM coordinates (float). */
inline struct FloatVect3 StateGetPositionUtm_f(void);

/** @brief Get position in ECEF coordinates (float). */
inline struct EcefCoor_f StateGetPositionEcef_f(void);

/** @brief Get position in local NED coordinates (float). */
inline struct NedCoor_f  StateGetPositionNed_f(void);

/** @brief Get position in LLA coordinates (float). */
inline struct LlaCoor_f  StateGetPositionLla_f(void);

/** @brief Get ground speed in local NED coordinates (float). */
inline struct NedCoor_f StateGetSpeedNed_f(void);

/** @brief Get ground speed in ECEF coordinates (float). */
inline struct EcefCoor_f StateGetSpeedEcef_f(void);

/** @brief Get norm of horizontal ground speed (float). */
inline float StateGetHorizontalSpeedNorm_f(void);

/** @brief Get dir of horizontal ground speed (float). */
inline float StateGetHorizontalSpeedDir_f(void);

/** @brief Get acceleration in NED coordinates (float). */
inline struct NedCoor_f StateGetAccelNed_f(void);

/** @brief Get acceleration in ECEF coordinates (float). */
inline struct EcefCoor_f StateGetAccelEcef_f(void);

/** @brief Get vehicle body attitude quaternion (float). */
inline struct FloatQuat StateGetNedToBodyQuat_f(void);

/** @brief Get vehicle body attitude rotation matrix (float). */
inline struct FloatRMat StateGetNedToBodyRMat_f(void);

/** @brief Get vehicle body attitude euler angles (float). */
inline struct FloatEulers StateGetNedToBodyEulers_f(void);

/** @brief Get vehicle body angular rate (float). */
inline struct FloatRates StateGetBodyRates_f(void);

/** @brief Get horizontal windspeed (float). */
inline struct FloatVect2 StateGetHorizontalWindspeed_f(void);

/** @brief Get airspeed (float). */
inline float StateGetAirspeed_f(void);


#endif /* STATE_H */
