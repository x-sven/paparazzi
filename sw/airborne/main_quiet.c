/*
 * $Id$
 *  
 * Copyright (C) 2009  Martin Mueller
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


#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "uart_hw.h"
#include "uart.h"


#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE (!FALSE)
#endif

static inline void main_init( void );
static inline void main_periodic_task( void );
int main_log(void);

int main(void)
{
  while(1);
}

static inline void main_init( void ) {
}

static inline void main_periodic_task( void ) {
}

