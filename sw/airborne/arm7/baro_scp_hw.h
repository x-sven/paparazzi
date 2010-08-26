/**
 * \file
 * 
 * Moved from airbone/ to airbone/arm7/ and renamed to _hw
 *
 * This file has been moved to the arm7 hardware directory
 * because we need some modifications to the application layer
 * in airbone/ to use the baro height for controlling altitude.
 *
 * Based on airbone/scp1000.c some month ago
 *
 */

#ifndef BARO_SCP_HW_H
#define BARO_SCP_HW_H

#include "std.h"

#define STA_UNINIT       0
#define STA_INITIALISING 1
#define STA_IDLE         2

extern uint8_t  baro_scp_status;
extern uint32_t baro_scp_pressure;
extern uint16_t baro_scp_temperature;
extern bool_t baro_scp_available;

void baro_scp_init(void);
void baro_scp_periodic(void);

#endif
