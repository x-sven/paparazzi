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
* @file state.c
*   @brief General inteface for the main vehicle states.
*
*   This is the more detailed description of this file.
*
*   @author Felix Ruess <felix.ruess@gmail.com>
*
*/

#include "state.h"

struct State state;



/*******************************************************************************
 *                                                                             *
 * Set and Get functions for the POSITION representations                      *
 *                                                                             *
 ******************************************************************************/
/** @addtogroup PosGroup
 *  @{ */
/************************ Set functions ****************************/
inline void StateSetPositionEcef_i(struct EcefCoor_i* ecef_pos) {
  INT32_VECT3_COPY(state.ecef_pos_i, *ecef_pos);
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_ECEF_I);
}

inline void StateSetPositionNed_i(struct NedCoor_i* ned_pos) {
  INT32_VECT3_COPY(state.ned_pos_i, *ned_pos);
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_NED_I);
}

inline void StateSetPositionLla_i(struct LlaCoor_i* lla_pos) {
  LLA_COPY(state.lla_pos_i, *lla_pos);
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_LLA_I);
}

//TODO utm in fixedpoint as well?

inline void StateSetPositionUtm_f(struct FloatVect3* utm_pos) {
  //TODO utm zone??
  VECT3_COPY(state.utm_pos_f, *utm_pos);
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (uint8_t)(1 << POS_UTM_F);
}

inline void StateSetPositionEcef_f(struct EcefCoor_f* ecef_pos) {
  VECT3_COPY(state.ecef_pos_f, *ecef_pos);
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_ECEF_F);
}

inline void StateSetPositionNed_f(struct NedCoor_f* ned_pos) {
  VECT3_COPY(state.ned_pos_f, *ned_pos);
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_NED_F);
}

inline void StateSetPositionLla_f(struct LlaCoor_f* lla_pos) {
  LLA_COPY(state.lla_pos_f, *lla_pos);
  /* clear bits for all position representations and only set the new one */
  state.pos_status = (1 << POS_LLA_F);
}

/************************ Get functions ****************************/
inline struct EcefCoor_i StateGetPositionEcef_i(void) {
  if (bit_is_set(state.pos_status, POS_ECEF_I))
    return state.ecef_pos_i;

  if (bit_is_set(state.pos_status, POS_ECEF_F)) {
    ECEF_BFP_OF_REAL(state.ecef_pos_i, state.ecef_pos_f);
  }
  else if (bit_is_set(state.pos_status, POS_NED_I)) {
    //TODO check if resolution is good enough
    ecef_of_ned_point_i(&state.ecef_pos_i, &state.ned_origin_i, &state.ned_pos_i);
  }
  else if (bit_is_set(state.pos_status, POS_NED_F)) {
    /* transform ned_f to ecef_f, set status bit, then convert to int */
    ecef_of_ned_point_f(&state.ecef_pos_f, &state.ned_origin_f, &state.ned_pos_f);
    SetBit(state.pos_status, POS_ECEF_F);
    ECEF_BFP_OF_REAL(state.ecef_pos_i, state.ecef_pos_f);
  }
  else if (bit_is_set(state.pos_status, POS_LLA_F)) {
    /* transform lla_f to ecef_f, set status bit, then convert to int */
    ecef_of_lla_f(&state.ecef_pos_f, &state.lla_pos_f);
    SetBit(state.pos_status, POS_ECEF_F);
    ECEF_BFP_OF_REAL(state.ecef_pos_i, state.ecef_pos_f);
  }
  else if (bit_is_set(state.pos_status, POS_LLA_I)) {
    ecef_of_lla_i(&state.ecef_pos_i, &state.lla_pos_i);
  }
  else {
    /* could not get this representation,  set errno */
    struct EcefCoor_i _ecef_zero = {0};
    return _ecef_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.pos_status, POS_ECEF_I);

  return state.ecef_pos_i;
}

inline struct NedCoor_i StateGetPositionNed_i(void) {
  if (bit_is_set(state.pos_status, POS_NED_I))
    return state.ned_pos_i;

  int errno = 0;
  if (state.ned_initialised_i) {
    if (bit_is_set(state.pos_status, POS_NED_F)) {
      NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
    }
    else if (bit_is_set(state.pos_status, POS_ECEF_I)) {
      ned_of_ecef_point_i(&state.ned_pos_i, &state.ned_origin_i, &state.ecef_pos_i);
    }
    else if (bit_is_set(state.pos_status, POS_ECEF_F)) {
      /* transform ecef_f -> ned_f, set status bit, then convert to int */
      ned_of_ecef_point_f(&state.ned_pos_f, &state.ned_origin_f, &state.ecef_pos_f);
      SetBit(state.pos_status, POS_NED_F);
      NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
    }
    else if (bit_is_set(state.pos_status, POS_LLA_F)) {
      /* transform lla_f -> ecef_f -> ned_f, set status bits, then convert to int */
      ecef_of_lla_f(&state.ecef_pos_f, &state.lla_pos_f);
      SetBit(state.pos_status, POS_ECEF_F);
      ned_of_ecef_point_f(&state.ned_pos_f, &state.ned_origin_f, &state.ecef_pos_f);
      SetBit(state.pos_status, POS_NED_F);
      NED_BFP_OF_REAL(state.ned_pos_i, state.ned_pos_f);
    }
    else if (bit_is_set(state.pos_status, POS_LLA_I)) {
      ned_of_lla_point_i(&state.ned_pos_i, &state.ned_origin_i, &state.lla_pos_i);
    }
    else { /* could not get this representation,  set errno */
      errno = 1;
    }
  } else { /* ned coordinate system not initialized,  set errno */
    errno = 2;
  }
  if (errno) {
    struct NedCoor_i _ned_zero = {0};
    return _ned_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.pos_status, POS_NED_I);

  return state.ned_pos_i;
}

inline struct LlaCoor_i StateGetPositionLla_i(void) {
  if (bit_is_set(state.pos_status, POS_LLA_I))
    return state.lla_pos_i;

  if (bit_is_set(state.pos_status, POS_LLA_F)) {
    LLA_BFP_OF_REAL(state.lla_pos_i, state.lla_pos_f);
  }
  else if (bit_is_set(state.pos_status, POS_ECEF_I)) {
    lla_of_ecef_i(&state.lla_pos_i, &state.ecef_pos_i);
  }
  else if (bit_is_set(state.pos_status, POS_ECEF_F)) {
    /* transform ecef_f -> lla_f, set status bit, then convert to int */
    lla_of_ecef_f(&state.lla_pos_f, &state.ecef_pos_f);
    SetBit(state.pos_status, POS_LLA_F);
    LLA_BFP_OF_REAL(state.lla_pos_i, state.lla_pos_f);
  }
  else if (bit_is_set(state.pos_status, POS_NED_F)) {
    /* transform ned_f -> ecef_f -> lla_f -> lla_i, set status bits */
    ecef_of_ned_point_f(&state.ecef_pos_f, &state.ned_origin_f, &state.ned_pos_f);
    SetBit(state.pos_status, POS_ECEF_F);
    lla_of_ecef_f(&state.lla_pos_f, &state.ecef_pos_f);
    SetBit(state.pos_status, POS_LLA_F);
    LLA_BFP_OF_REAL(state.lla_pos_i, state.lla_pos_f);
  }
  else if (bit_is_set(state.pos_status, POS_NED_I)) {
    /* transform ned_i -> ecef_i -> lla_i, set status bits */
    //TODO check if resolution is enough
    ecef_of_ned_point_i(&state.ecef_pos_i, &state.ned_origin_i, &state.ned_pos_i);
    SetBit(state.pos_status, POS_ECEF_I);
    lla_of_ecef_i(&state.lla_pos_i, &state.ecef_pos_i); /* uses double version internally */
  }
  else {
    /* could not get this representation,  set errno */
    struct LlaCoor_i _lla_zero = {0};
    return _lla_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.pos_status, POS_LLA_I);

  return state.lla_pos_i;
}

inline struct EcefCoor_f StateGetPositionEcef_f(void) {
  if (bit_is_set(state.pos_status, POS_ECEF_F))
    return state.ecef_pos_f;

  if (bit_is_set(state.pos_status, POS_ECEF_I)) {
    ECEF_FLOAT_OF_BFP(state.ecef_pos_f, state.ecef_pos_i);
  }
  else if (bit_is_set(state.pos_status, POS_NED_F)) {
    ecef_of_ned_point_f(&state.ecef_pos_f, &state.ned_origin_f, &state.ned_pos_f);
  }
  else if (bit_is_set(state.pos_status, POS_NED_I)) {
    /* transform ned_i -> ecef_i -> ecef_f, set status bits */
    //TODO check if resolution is good enough
    ecef_of_ned_point_i(&state.ecef_pos_i, &state.ned_origin_i, &state.ned_pos_i);
    SetBit(state.pos_status, POS_ECEF_F);
    ECEF_FLOAT_OF_BFP(state.ecef_pos_f, state.ecef_pos_i);
  }
  else if (bit_is_set(state.pos_status, POS_LLA_F)) {
    ecef_of_lla_f(&state.ecef_pos_f, &state.lla_pos_f);
  }
  else if (bit_is_set(state.pos_status, POS_LLA_I)) {
    struct EcefCoor_i _ecef_i2;
    ecef_of_lla_i(&_ecef_i2, &state.lla_pos_i);
    ECEF_FLOAT_OF_BFP(state.ecef_pos_f, _ecef_i2);
  }
  else {
    /* could not get this representation,  set errno */
    struct EcefCoor_f _ecef_zero = {0.0f};
    return _ecef_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.pos_status, POS_ECEF_F);

  return state.ecef_pos_f;
}

inline struct NedCoor_f StateGetPositionNed_f(void) {
  if (bit_is_set(state.pos_status, POS_NED_F))
    return state.ned_pos_f;

  int errno = 0;
  if (state.ned_initialised_f) {
    if (bit_is_set(state.pos_status, POS_NED_I)) {
      NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
    }
    else if (bit_is_set(state.pos_status, POS_ECEF_F)) {
      ned_of_ecef_point_f(&state.ned_pos_f, &state.ned_origin_f, &state.ecef_pos_f);
    }
    else if (bit_is_set(state.pos_status, POS_ECEF_I)) {
      /* transform ecef_i -> ned_i -> ned_f, set status bits */
      ned_of_ecef_point_i(&state.ned_pos_i, &state.ned_origin_i, &state.ecef_pos_i);
      SetBit(state.pos_status, POS_NED_I);
      NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
    }
    else if (bit_is_set(state.pos_status, POS_LLA_F)) {
      ned_of_lla_point_f(&state.ned_pos_f, &state.ned_origin_f, &state.lla_pos_f);
    }
    else if (bit_is_set(state.pos_status, POS_LLA_I)) {
      /* transform lla_i -> ecef_i -> ned_i -> ned_f, set status bits */
      ecef_of_lla_i(&state.ecef_pos_i, &state.lla_pos_i); /* converts to doubles internally */
      SetBit(state.pos_status, POS_ECEF_I);
      ned_of_ecef_point_i(&state.ned_pos_i, &state.ned_origin_i, &state.ecef_pos_i);
      SetBit(state.pos_status, POS_NED_I);
      NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
    }
    else { /* could not get this representation,  set errno */
      errno = 1;
    }
  } else { /* ned coordinate system not initialized,  set errno */
    errno = 2;
  }
  if (errno) {
    struct NedCoor_f _ned_zero = {0.0f};
    return _ned_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.pos_status, POS_NED_F);

  return state.ned_pos_f;
}

inline struct LlaCoor_f StateGetPositionLla_f(void) {
  if (bit_is_set(state.pos_status, POS_LLA_F))
    return state.lla_pos_f;

  if (bit_is_set(state.pos_status, POS_LLA_I)) {
    LLA_FLOAT_OF_BFP(state.lla_pos_f, state.lla_pos_f);
  }
  else if (bit_is_set(state.pos_status, POS_ECEF_F)) {
    lla_of_ecef_f(&state.lla_pos_f, &state.ecef_pos_f);
  }
  else if (bit_is_set(state.pos_status, POS_ECEF_I)) {
    /* transform ecef_i -> ecef_f -> lla_f, set status bits */
    ECEF_FLOAT_OF_BFP(state.ecef_pos_f, state.ecef_pos_i);
    SetBit(state.pos_status, POS_ECEF_F);
    lla_of_ecef_f(&state.lla_pos_f, &state.ecef_pos_f);
  }
  else if (bit_is_set(state.pos_status, POS_NED_F)) {
    /* transform ned_f -> ecef_f -> lla_f, set status bits */
    ecef_of_ned_point_f(&state.ecef_pos_f, &state.ned_origin_f, &state.ned_pos_f);
    SetBit(state.pos_status, POS_ECEF_F);
    lla_of_ecef_f(&state.lla_pos_f, &state.ecef_pos_f);
  }
  else if (bit_is_set(state.pos_status, POS_NED_I)) {
    /* transform ned_i -> ned_f -> ecef_f -> lla_f, set status bits */
    NED_FLOAT_OF_BFP(state.ned_pos_f, state.ned_pos_i);
    SetBit(state.pos_status, POS_NED_F);
    ecef_of_ned_point_f(&state.ecef_pos_f, &state.ned_origin_f, &state.ned_pos_f);
    SetBit(state.pos_status, POS_ECEF_F);
    lla_of_ecef_f(&state.lla_pos_f, &state.ecef_pos_f);
  }
  else {
    /* could not get this representation,  set errno */
    struct LlaCoor_f _lla_zero = {0.0};
    return _lla_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.pos_status, POS_LLA_F);

  return state.lla_pos_f;
}
/** @}*/





/******************************************************************************
 *                                                                            *
 * Set and Get functions for the SPEED representations                        *
 *                                                                            *
 *****************************************************************************/
/** @addtogroup SpeedGroup
 *  @{ */
/************************ Set functions ****************************/
inline void StateSetSpeedNed_i(struct NedCoor_i* ned_speed) {
  INT32_VECT3_COPY(state.ned_speed_i, *ned_speed);
  /* clear bits for all speed representations and only set the new one */
  state.speed_status = (1 << SPEED_NED_I);
}

inline void StateSetSpeedEcef_i(struct EcefCoor_i* ecef_speed) {
  INT32_VECT3_COPY(state.ecef_speed_i, *ecef_speed);
  /* clear bits for all speed representations and only set the new one */
  state.speed_status = (1 << SPEED_ECEF_I);
}

inline void StateSetSpeedNed_f(struct NedCoor_f* ned_speed) {
  VECT3_COPY(state.ned_speed_f, *ned_speed);
  /* clear bits for all speed representations and only set the new one */
  state.speed_status = (1 << SPEED_NED_F);
}

inline void StateSetSpeedEcef_f(struct EcefCoor_f* ecef_speed) {
  VECT3_COPY(state.ecef_speed_f, *ecef_speed);
  /* clear bits for all speed representations and only set the new one */
  state.speed_status = (1 << SPEED_ECEF_F);
}

/************************ Get functions ****************************/
inline struct NedCoor_i StateGetSpeedNed_i(void) {
  if (bit_is_set(state.speed_status, SPEED_NED_I))
    return state.ned_speed_i;

  int errno = 0;
  if (state.ned_initialised_i) {
    if (bit_is_set(state.speed_status, SPEED_NED_F)) {
      SPEEDS_BFP_OF_REAL(state.ned_speed_i, state.ned_speed_f);
    }
    else if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
      ned_of_ecef_vect_i(&state.ned_speed_i, &state.ned_origin_i, &state.ecef_speed_i);
    }
    else if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
      /* transform ecef_f -> ecef_i -> ned_i , set status bits */
      SPEEDS_BFP_OF_REAL(state.ecef_speed_i, state.ecef_speed_f);
      SetBit(state.speed_status, SPEED_ECEF_I);
      ned_of_ecef_vect_i(&state.ned_speed_i, &state.ned_origin_i, &state.ecef_speed_i);
    }
    else { /* could not get this representation,  set errno */
      errno = 1;
    }
  } else { /* ned coordinate system not initialized,  set errno */
    errno = 2;
  }
  if (errno) {
    struct NedCoor_i _ned_zero = {0};
    return _ned_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_NED_I);

  return state.ned_speed_i;
}

inline struct EcefCoor_i StateGetSpeedEcef_i(void) {
  if (bit_is_set(state.speed_status, SPEED_ECEF_I))
    return state.ecef_speed_i;

  if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
    SPEEDS_BFP_OF_REAL(state.ecef_speed_i, state.ecef_speed_f);
  }
  else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
    ecef_of_ned_vect_i(&state.ecef_speed_i, &state.ned_origin_i, &state.ned_speed_i);
  }
  else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
    /* transform ned_f -> ned_i -> ecef_i , set status bits */
    SPEEDS_BFP_OF_REAL(state.ned_speed_i, state.ned_speed_f);
    SetBit(state.speed_status, SPEED_NED_I);
    ecef_of_ned_vect_i(&state.ecef_speed_i, &state.ned_origin_i, &state.ned_speed_i);
  }
  else {
    /* could not get this representation,  set errno */
    struct EcefCoor_i _ecef_zero = {0};
    return _ecef_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_ECEF_I);

  return state.ecef_speed_i;
}

inline int32_t StateGetHorizontalSpeedNorm_i(void) { //TODO
  if (bit_is_set(state.speed_status, SPEED_HNORM_I))
    return state.h_speed_norm_i;

  if (bit_is_set(state.speed_status, SPEED_HNORM_F)){
    state.h_speed_norm_i = SPEED_BFP_OF_REAL(state.h_speed_norm_f);
  }
  else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
    //TODO consider INT32_SPEED_FRAC
    //INT32_VECT2_NORM(state.h_speed_norm_i, state.ned_speed_i);
  }
  else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
    float _norm_f;
    FLOAT_VECT2_NORM(_norm_f, state.ned_speed_f);
    state.h_speed_norm_i = SPEED_BFP_OF_REAL(_norm_f);
  }
  else if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
    /* transform ecef speed to ned, set status bit, then compute norm */
    //foo
    //TODO consider INT32_SPEED_FRAC
    //INT32_VECT2_NORM(state.h_speed_norm_i, state.ned_speed_i);
  }
  else if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
    float _norm_f;
    FLOAT_VECT2_NORM(_norm_f, state.ned_speed_f);
    state.h_speed_norm_i = SPEED_BFP_OF_REAL(_norm_f);
  }
  else {
    int32_t _norm_zero = 0;
    return _norm_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_HNORM_I);

  return state.h_speed_norm_i;
}

inline int32_t StateGetHorizontalSpeedDir_i(void) { //TODO
  if (bit_is_set(state.speed_status, SPEED_HDIR_I))
    return state.h_speed_dir_i;

  if (bit_is_set(state.speed_status, SPEED_HDIR_F)){
    state.h_speed_dir_i = SPEED_BFP_OF_REAL(state.h_speed_dir_f);
  }
  else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
    //foo
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_HDIR_I);

  return state.h_speed_dir_i;
}

inline struct NedCoor_f StateGetSpeedNed_f(void) {
  if (bit_is_set(state.speed_status, SPEED_NED_F))
    return state.ned_speed_f;

  int errno = 0;
  if (state.ned_initialised_f) {
    if (bit_is_set(state.speed_status, SPEED_NED_I)) {
      SPEEDS_FLOAT_OF_BFP(state.ned_speed_f, state.ned_speed_i);
    }
    else if (bit_is_set(state.speed_status, SPEED_ECEF_F)) {
      ned_of_ecef_vect_f(&state.ned_speed_f, &state.ned_origin_f, &state.ecef_speed_f);
    }
    else if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
      /* transform ecef_i -> ecef_f -> ned_f , set status bits */
      SPEEDS_FLOAT_OF_BFP(state.ecef_speed_f, state.ecef_speed_i);
      SetBit(state.speed_status, SPEED_ECEF_F);
      ned_of_ecef_vect_f(&state.ned_speed_f, &state.ned_origin_f, &state.ecef_speed_f);
    }
    else { /* could not get this representation,  set errno */
      errno = 1;
    }
  } else { /* ned coordinate system not initialized,  set errno */
    errno = 2;
  }
  if (errno) {
    struct NedCoor_f _ned_zero = {0.0f};
    return _ned_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_NED_F);

  return state.ned_speed_f;
}

inline struct EcefCoor_f StateGetSpeedEcef_f(void) {
  if (bit_is_set(state.speed_status, SPEED_ECEF_F))
    return state.ecef_speed_f;

  if (bit_is_set(state.speed_status, SPEED_ECEF_I)) {
    SPEEDS_FLOAT_OF_BFP(state.ecef_speed_f, state.ned_speed_i);
  }
  else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
    ecef_of_ned_vect_f(&state.ecef_speed_f, &state.ned_origin_f, &state.ned_speed_f);
  }
  else if (bit_is_set(state.speed_status, SPEED_NED_I)) {
    /* transform ned_f -> ned_i -> ecef_i , set status bits */
    SPEEDS_FLOAT_OF_BFP(state.ned_speed_f, state.ned_speed_i);
    SetBit(state.speed_status, SPEED_NED_F);
    ecef_of_ned_vect_f(&state.ecef_speed_f, &state.ned_origin_f, &state.ned_speed_f);
  }
  else {
    /* could not get this representation,  set errno */
    struct EcefCoor_f _ecef_zero = {0.0f};
    return _ecef_zero;
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_ECEF_F);

  return state.ecef_speed_f;
}

inline float StateGetHorizontalSpeedNorm_f(void) { //TODO
  if (!bit_is_set(state.speed_status, SPEED_HNORM_F))
    return state.h_speed_norm_f;return state.h_speed_norm_f;

  if (bit_is_set(state.speed_status, SPEED_HNORM_I)){
    state.h_speed_norm_f = SPEED_FLOAT_OF_BFP(state.h_speed_norm_i);
  } else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
    FLOAT_VECT2_NORM(state.h_speed_norm_f, state.ned_speed_f);
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_HNORM_F);

  return state.h_speed_norm_f;
}

inline float StateGetHorizontalSpeedDir_f(void) { //TODO
  if (bit_is_set(state.speed_status, SPEED_HDIR_F))
    return state.h_speed_dir_f;

  if (bit_is_set(state.speed_status, SPEED_HDIR_I)){
    state.h_speed_dir_f = SPEED_FLOAT_OF_BFP(state.h_speed_dir_i);
  } else if (bit_is_set(state.speed_status, SPEED_NED_F)) {
    //foo
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.speed_status, SPEED_HDIR_F);

  return state.h_speed_dir_f;
}
/** @}*/



/******************************************************************************
 *                                                                            *
 * Set and Get functions for the ACCELERATION representations                 *
 *                                                                            *
 *****************************************************************************/
/** @addtogroup AccelGroup
 *  @{ */
/************************ Set functions ****************************/
inline void StateSetAccelNed_i(struct NedCoor_i* ned_accel) {
  INT32_VECT3_COPY(state.ned_accel_i, *ned_accel);
  /* clear bits for all accel representations and only set the new one */
  state.accel_status = (1 << ACCEL_NED_I);
}

inline void StateSetAccelEcef_i(struct EcefCoor_i* ecef_accel) {
  INT32_VECT3_COPY(state.ecef_accel_i, *ecef_accel);
  /* clear bits for all accel representations and only set the new one */
  state.accel_status = (1 << ACCEL_ECEF_I);
}

inline void StateSetAccelNed_f(struct NedCoor_f* ned_accel) {
  VECT3_COPY(state.ned_accel_f, *ned_accel);
  /* clear bits for all accel representations and only set the new one */
  state.accel_status = (1 << ACCEL_NED_F);
}

inline void StateSetAccelEcef_f(struct EcefCoor_f* ecef_accel) {
  VECT3_COPY(state.ecef_accel_f, *ecef_accel);
  /* clear bits for all accel representations and only set the new one */
  state.accel_status = (1 << ACCEL_ECEF_F);
}

/************************ Get functions ****************************/
inline struct NedCoor_i StateGetAccelNed_i(void) {
  if (bit_is_set(state.accel_status, ACCEL_NED_I))
    return state.ned_accel_i;

  if (bit_is_set(state.accel_status, ACCEL_ECEF_I)) {
    if (state.ned_initialised_i) {
      ned_of_ecef_vect_i(&state.ned_accel_i, &state.ned_origin_i, &state.ecef_accel_i);
    }
  } else {
    //try floats....
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.accel_status, ACCEL_NED_I);

  return state.ned_accel_i;
}

inline struct EcefCoor_i StateGetAccelEcef_i(void) {
  if (bit_is_set(state.accel_status, ACCEL_ECEF_I))
    return state.ecef_accel_i;

  if (bit_is_set(state.accel_status, ACCEL_NED_I)) {
    if (state.ned_initialised_i) {
      //ecef_of_ned_vect_i(&state.ecef_accel_i, &state.ned_origin_i, &state.ned_accel_i);
    }
  } else {
    //try floats....
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.accel_status, ACCEL_ECEF_I);

  return state.ecef_accel_i;
}

inline struct NedCoor_f StateGetAccelNed_f(void) {
  if (bit_is_set(state.accel_status, ACCEL_NED_F))
    return state.ned_accel_f;

  if (bit_is_set(state.accel_status, ACCEL_ECEF_F)) {
    if (state.ned_initialised_f) {
      ned_of_ecef_vect_f(&state.ned_accel_f, &state.ned_origin_f, &state.ecef_accel_f);
    }
  } else {
    //try ints....
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.accel_status, ACCEL_NED_F);

  return state.ned_accel_f;
}

inline struct EcefCoor_f StateGetAccelEcef_f(void) {
  if (bit_is_set(state.accel_status, ACCEL_ECEF_F))
    return state.ecef_accel_f;

  if (bit_is_set(state.accel_status, ACCEL_NED_F)) {
    //ecef_of_ned_vect_f(&state.ecef_accel_f, &state.ned_origin_f, &state.ned_accel_f);
  } else {
    //try ints....
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.accel_status, ACCEL_ECEF_F);

  return state.ecef_accel_f;
}
/** @}*/



/******************************************************************************
 *                                                                            *
 * Set and Get functions for the ATTITUDE representations                     *
 *                                                                            *
 *****************************************************************************/
/** @addtogroup AttGroup
 *  @{ */
/************************ Set functions ****************************/
inline void StateSetNedToBodyQuat_i(struct Int32Quat* ned_to_body_quat) {
  QUAT_COPY(state.ned_to_body_quat_i, *ned_to_body_quat);
  /* clear bits for all attitude representations and only set the new one */
  state.att_status = (1 << ATT_QUAT_I);
}

inline void StateSetNedToBodyRMat_i(struct Int32RMat* ned_to_body_rmat) {
  RMAT_COPY(state.ned_to_body_rmat_i, *ned_to_body_rmat);
  /* clear bits for all attitude representations and only set the new one */
  state.att_status = (1 << ATT_RMAT_I);
}

inline void StateSetNedToBodyEulers_i(struct Int32Eulers* ned_to_body_eulers) {
  EULERS_COPY(state.ned_to_body_eulers_i, *ned_to_body_eulers);
  /* clear bits for all attitude representations and only set the new one */
  state.att_status = (1 << ATT_EULER_I);
}

inline void StateSetNedToBodyQuat_f(struct FloatQuat* ned_to_body_quat) {
  QUAT_COPY(state.ned_to_body_quat_f, *ned_to_body_quat);
  /* clear bits for all attitude representations and only set the new one */
  state.att_status = (1 << ATT_QUAT_F);
}

inline void StateSetNedToBodyRMat_f(struct FloatRMat* ned_to_body_rmat) {
  RMAT_COPY(state.ned_to_body_rmat_f, *ned_to_body_rmat);
  /* clear bits for all attitude representations and only set the new one */
  state.att_status = (1 << ATT_RMAT_F);
}

inline void StateSetNedToBodyEulers_f(struct FloatEulers* ned_to_body_eulers) {
  EULERS_COPY(state.ned_to_body_eulers_f, *ned_to_body_eulers);
  /* clear bits for all attitude representations and only set the new one */
  state.att_status = (1 << ATT_EULER_F);
}

/************************ Get functions ****************************/
inline struct Int32Quat StateGetNedToBodyQuat_i(void) {
  if (!bit_is_set(state.att_status, ATT_QUAT_I)) {
    if (bit_is_set(state.att_status, ATT_RMAT_I)) {
      INT32_QUAT_OF_RMAT(state.ned_to_body_quat_i, state.ned_to_body_rmat_i);
    } else if (bit_is_set(state.att_status, ATT_EULER_I)) {
      INT32_QUAT_OF_EULERS(state.ned_to_body_quat_i, state.ned_to_body_eulers_i);
    } else {
      //try floats....
    }
    /* set bit to indicate this representation is computed */
    SetBit(state.att_status, ATT_QUAT_I);
  }
  return state.ned_to_body_quat_i;
}

inline struct Int32RMat StateGetNedToBodyRMat_i(void) {
  if (bit_is_set(state.att_status, ATT_RMAT_I))
    return state.ned_to_body_rmat_i;

  if (bit_is_set(state.att_status, ATT_QUAT_I)) {
    INT32_RMAT_OF_QUAT(state.ned_to_body_rmat_i, state.ned_to_body_quat_i);
  } else if (bit_is_set(state.att_status, ATT_EULER_I)) {
    INT32_RMAT_OF_EULERS(state.ned_to_body_rmat_i, state.ned_to_body_eulers_i);
  } else {
    //try floats....
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.att_status, ATT_RMAT_I);

  return state.ned_to_body_rmat_i;
}

inline struct Int32Eulers StateGetNedToBodyEulers_i(void) {
  if (bit_is_set(state.att_status, ATT_EULER_I))
    return state.ned_to_body_eulers_i;

  if (bit_is_set(state.att_status, ATT_QUAT_I)) {
    INT32_EULERS_OF_QUAT(state.ned_to_body_eulers_i, state.ned_to_body_quat_i);
  } else if (bit_is_set(state.att_status, ATT_RMAT_I)) {
    INT32_EULERS_OF_RMAT(state.ned_to_body_eulers_i, state.ned_to_body_rmat_i);
  } else {
    //try floats....
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.att_status, ATT_EULER_I);

  return state.ned_to_body_eulers_i;
}

inline struct FloatQuat StateGetNedToBodyQuat_f(void) {
  if (bit_is_set(state.att_status, ATT_QUAT_F))
    return state.ned_to_body_quat_f;

  if (bit_is_set(state.att_status, ATT_RMAT_F)) {
    FLOAT_QUAT_OF_RMAT(state.ned_to_body_quat_f, state.ned_to_body_rmat_f);
  } else if (bit_is_set(state.att_status, ATT_EULER_F)) {
    FLOAT_QUAT_OF_EULERS(state.ned_to_body_quat_f, state.ned_to_body_eulers_f);
  } else {
    //try ints....
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.att_status, ATT_QUAT_F);

  return state.ned_to_body_quat_f;
}

inline struct FloatRMat StateGetNedToBodyRMat_f(void) {
  if (bit_is_set(state.att_status, ATT_RMAT_F))
    return state.ned_to_body_rmat_f;

  if (bit_is_set(state.att_status, ATT_QUAT_F)) {
    FLOAT_RMAT_OF_QUAT(state.ned_to_body_rmat_f, state.ned_to_body_quat_f);
  } else if (bit_is_set(state.att_status, ATT_EULER_F)) {
    FLOAT_RMAT_OF_EULERS(state.ned_to_body_rmat_f, state.ned_to_body_eulers_f);
  } else {
    //try floats....
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.att_status, ATT_RMAT_F);

  return state.ned_to_body_rmat_f;
}

inline struct FloatEulers StateGetNedToBodyEulers_f(void) {
  if (bit_is_set(state.att_status, ATT_EULER_F))
    return state.ned_to_body_eulers_f;

  if (bit_is_set(state.att_status, ATT_QUAT_F)) {
    FLOAT_EULERS_OF_QUAT(state.ned_to_body_eulers_f, state.ned_to_body_quat_f);
  } else if (bit_is_set(state.att_status, ATT_RMAT_F)) {
    FLOAT_EULERS_OF_RMAT(state.ned_to_body_eulers_f, state.ned_to_body_rmat_f);
  } else {
    //try floats....
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.att_status, ATT_EULER_F);

  return state.ned_to_body_eulers_f;
}
/** @}*/


/******************************************************************************
 *                                                                            *
 * Set and Get functions for the ANGULAR RATE representations                 *
 *                                                                            *
 *****************************************************************************/
/** @addtogroup RateGroup
 *  @{ */
/************************ Set functions ****************************/
inline void StateSetBodyRates_i(struct Int32Rates* body_rate) {
  RATES_COPY(state.body_rates_i, *body_rate);
  /* clear bits for all attitude representations and only set the new one */
  state.rate_status = (1 << RATE_I);
}

inline void StateSetBodyRates_f(struct FloatRates* body_rate) {
  RATES_COPY(state.body_rates_f, *body_rate);
  /* clear bits for all attitude representations and only set the new one */
  state.rate_status = (1 << RATE_F);
}


/************************ Get functions ****************************/
inline struct Int32Rates StateGetBodyRates_i(void) {
  if (bit_is_set(state.rate_status, RATE_I))
    return state.body_rates_i;

  if (bit_is_set(state.rate_status, RATE_F)) {
    RATES_BFP_OF_REAL(state.body_rates_i, state.body_rates_f);
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.rate_status, RATE_I);

  return state.body_rates_i;
}

inline struct FloatRates StateGetBodyRates_f(void) {
  if (bit_is_set(state.rate_status, RATE_F))
    return state.body_rates_f;

  if (bit_is_set(state.rate_status, RATE_I)) {
    RATES_FLOAT_OF_BFP(state.body_rates_f, state.body_rates_i);
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.rate_status, RATE_F);

  return state.body_rates_f;
}
/** @}*/


/******************************************************************************
 *                                                                            *
 * Set and Get functions for the WIND- AND AIRSPEED representations           *
 *                                                                            *
 *****************************************************************************/
/** @addtogroup WindAirGroup
 *  @{ */
/************************ Set functions ****************************/
inline void StateSetHorizontalWindspeed_i(struct Int32Vect2* h_windspeed) {
  VECT2_COPY(state.h_windspeed_i, *h_windspeed);
  /* clear bits for all windspeed representations and only set the new one */
  ClearBit(state.wind_air_status, WINDSPEED_F);
  SetBit(state.wind_air_status, WINDSPEED_I);
}

inline void StateSetAirspeed_i(int32_t* airspeed) {
  state.airspeed_i = *airspeed;
  /* clear bits for all windspeed representations and only set the new one */
  ClearBit(state.wind_air_status, AIRSPEED_F);
  SetBit(state.wind_air_status, AIRSPEED_I);
}

inline void StateSetHorizontalWindspeed_f(struct FloatVect2* h_windspeed) {
  VECT2_COPY(state.h_windspeed_f, *h_windspeed);
  /* clear bits for all windspeed representations and only set the new one */
  ClearBit(state.wind_air_status, WINDSPEED_I);
  SetBit(state.wind_air_status, WINDSPEED_F);
}

inline void StateSetAirspeed_f(float* airspeed) {
  state.airspeed_f = *airspeed;
  /* clear bits for all windspeed representations and only set the new one */
  ClearBit(state.wind_air_status, AIRSPEED_I);
  SetBit(state.wind_air_status, AIRSPEED_F);
}

/************************ Get functions ****************************/
inline struct Int32Vect2 StateGetHorizontalWindspeed_i(void) {
  if (bit_is_set(state.wind_air_status, WINDSPEED_I))
    return state.h_windspeed_i;

  if (bit_is_set(state.wind_air_status, WINDSPEED_F)) {
    state.h_windspeed_i.x = SPEED_BFP_OF_REAL(state.h_windspeed_f.x);
    state.h_windspeed_i.y = SPEED_BFP_OF_REAL(state.h_windspeed_f.y);
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.rate_status, WINDSPEED_I);

  return state.h_windspeed_i;
}

inline int32_t StateGetAirspeed_i(void) {
  if (bit_is_set(state.wind_air_status, AIRSPEED_I))
    return state.airspeed_i;

  if (bit_is_set(state.wind_air_status, AIRSPEED_F)) {
    state.airspeed_i = SPEED_BFP_OF_REAL(state.airspeed_f);
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.wind_air_status, AIRSPEED_I);

  return state.airspeed_i;
}

inline struct FloatVect2 StateGetHorizontalWindspeed_f(void) {
  if (bit_is_set(state.wind_air_status, WINDSPEED_F))
    return state.h_windspeed_f;

  if (bit_is_set(state.wind_air_status, WINDSPEED_I)) {
    state.h_windspeed_f.x = SPEED_FLOAT_OF_BFP(state.h_windspeed_i.x);
    state.h_windspeed_f.x = SPEED_FLOAT_OF_BFP(state.h_windspeed_i.y);
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.rate_status, WINDSPEED_F);

  return state.h_windspeed_f;
}

inline float StateGetAirspeed_f(void) {
  if (bit_is_set(state.wind_air_status, AIRSPEED_F))
    return state.airspeed_f;

  if (bit_is_set(state.wind_air_status, AIRSPEED_I)) {
    state.airspeed_f = SPEED_FLOAT_OF_BFP(state.airspeed_i);
  }
  /* set bit to indicate this representation is computed */
  SetBit(state.wind_air_status, AIRSPEED_F);

  return state.airspeed_f;
}
/** @}*/
