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

/** \file state.c
 * \brief general inteface for the main vehicle states
 */

#include "state.h"

struct State state;

/*
 * Set State functions (int versions)
 */
inline void StateSetPositionEcef_i(struct EcefCoor_i* ecef_pos) {
  INT32_VECT3_COPY(state.ecef_pos_i, (*ecef_pos));
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_ECEF_I);
}

inline void StateSetPositionNed_i(struct NedCoor_i* ned_pos) {
  INT32_VECT3_COPY(state.ned_pos_i, (*ned_pos));
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_NED_I);
}

inline void StateSetPositionLla_i(struct LlaCoor_i* lla_pos) {
  LLA_COPY(state.lla_pos_i, (*lla_pos));
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_LLA_I);
}

inline void StateSetSpeedNed_i(struct NedCoor_i* ned_speed) {

}

inline void StateSetAccelNed_i(struct NedCoor_i* ned_accel) {

}

inline void StateSetNedToBodyQuat_i(struct Int32Quat* ned_to_body_quat) {
  QUAT_COPY(state.ned_to_body_quat_i, (*ned_to_body_quat));
  /* clear bits for all attitude representations and only set the new one */
  state.att_status = (1 << ATT_QUAT_I);
}

inline void StateSetNedToBodyRMat_i(struct Int32RMat* ned_to_body_rmat) {
  RMAT_COPY(state.ned_to_body_rmat_i, (*ned_to_body_rmat));
  /* clear bits for all attitude representations and only set the new one */
  state.att_status = (1 << ATT_RMAT_I);
}

inline void StateSetNedToBodyEulers_i(struct Int32Eulers* ned_to_body_eulers){
  EULERS_COPY(state.ned_to_body_eulers_i, (*ned_to_body_eulers));
  /* clear bits for all attitude representations and only set the new one */
  state.att_status = (1 << ATT_EULER_I);
}

inline void StateSetBodyRates_i(struct Int32Rates* body_rate){

}


/*
 * Get State functions (int versions)
 */
inline struct EcefCoor_i StateGetPositionEcef_i(void) {
  if (!bit_is_set(state.pos_status, POS_ECEF_I)) {
    //transform_pos(xx,xx);
    /* set bit to indicate this representation is computed */
    SetBit(state.pos_status, POS_ECEF_I);
  }
  return state.ecef_pos_i;
}

inline struct NedCoor_i StateGetPositionNed_i(void) {
  if (!bit_is_set(state.pos_status, POS_NED_I)) {
    //transform_pos(xx,xx);
    /* set bit to indicate this representation is computed */
    SetBit(state.pos_status, POS_NED_I);
  }
  return state.ned_pos_i;
}

inline struct LlaCoor_i StateGetPositionLla_i(void) {
  if (!bit_is_set(state.pos_status, POS_LLA_I)) {
    //transform_pos(xx,xx);
    /* set bit to indicate this representation is computed */
    SetBit(state.pos_status, POS_LLA_I);
  }
  return state.lla_pos_i;
}

/*
  inline struct NedCoor_i StateGetSpeedNed_i(void) {

  }

  inline struct NedCoor_i StateGetAccelNed_i(void) {

  }
*/

inline struct Int32Quat StateGetNedToBodyQuat_i(void) {
  if (!bit_is_set(state.att_status, ATT_QUAT_I)) {
    //transform_pos(xx,xx);
    /* set bit to indicate this representation is computed */
    SetBit(state.att_status, ATT_QUAT_I);
  }
  return state.ned_to_body_quat_i;
}

inline struct Int32RMat StateGetNedToBodyRMat_i(void) {
  if (!bit_is_set(state.att_status, POS_LLA_I)) {
    //transform_pos(xx,xx);
    /* set bit to indicate this representation is computed */
    SetBit(state.att_status, POS_LLA_I);
  }
  return state.ned_to_body_rmat_i;
}

inline struct Int32Eulers StateGetNedToBodyEulers_i(void) {
  if (!bit_is_set(state.att_status, ATT_EULER_I)) {
    //transform_pos(xx,xx);
    /* set bit to indicate this representation is computed */
    SetBit(state.att_status, ATT_EULER_I);
  }
  return state.ned_to_body_eulers_i;
}

/*
  inline struct Int32Rates StateGetBodyRates_i(void) {

  }
*/
