/*
 * Paparazzi $Id: ins_xsens.c 3872 2009-08-05 14:42:41Z mmm $
 *
 * Copyright (C) 2010 ENAC
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

/** \file ins_arduimu_spi.h
 *  \brief Driver for the ARDUIMU AHRS
 *  use the binary protocol on the SPI link
*/

extern "C"
{
	//#include "modules/ins/ins_vn100.h"
	#include "mcu_periph/spi.h"
	#include "estimator.h"
	#include "generated/airframe.h"
	#include "ins_arduimu_spi.h"
	#include "messages.h"

	#include "subsystems/gps.h"
}

#ifndef INS_YAW_NEUTRAL_DEFAULT
#define INS_YAW_NEUTRAL_DEFAULT 0.
#endif

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

struct FloatEulers ins_eulers;
struct FloatQuat ins_quat;
struct FloatRates ins_rates;
struct FloatRMat ins_rmat;
struct FloatVect3 ins_accel;
struct FloatVect3 ins_mag;

/* neutrals */
float ins_roll_neutral;
float ins_pitch_neutral;
float ins_yaw_neutral;

typedef struct {
	unsigned char iCmdID;
	unsigned char iRespID;
	uint16_t crc16;
	float   roll;
	float   pitch;
	float   yaw;
	float   omega_x;
	float   omega_y;
	float   omega_z;
} SPI_Message_t;

SPI_Message_t SPI_Message_transmit;
SPI_Message_t SPI_Message_receive;

typedef struct {
	unsigned char iCmdID;
	unsigned char iRespID;
	uint16_t crc16;
	float   speed_3d;
	float   ground_speed;
	float   ground_course;
} GPS_Message_t;

GPS_Message_t GPS_Message_transmit;
GPS_Message_t GPS_Message_receive;

unsigned short crc16(const char *pcBuffer, const int len);


class MyClass
{
  public:
    MyClass();
  private:
    int itemp;
};

MyClass::MyClass() {}

MyClass myclass;


//--------------------------------------------------//

void ArduIMU_SPI_init( void ) {

  ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
  ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
  ins_yaw_neutral = INS_YAW_NEUTRAL_DEFAULT;

  SpiClrCPOL();
  SpiClrCPHA();
}

void ArduIMU_SPI_periodic( void ) {

	if (!SpiCheckAvailable()) {
    SpiOverRun();
    return;
  }

  memset(&SPI_Message_transmit, 0, sizeof(SPI_Message_transmit)); //clear packet
  memset(&SPI_Message_receive, 0, sizeof(SPI_Message_receive)); //clear packet

  SPI_Message_transmit.iCmdID  = ARDUIMU_MESSAGE_REQUEST;
  SPI_Message_transmit.iRespID = 0xff;
  SPI_Message_transmit.crc16 = crc16((const char*)&SPI_Message_transmit, sizeof(SPI_Message_transmit));

  spi_buffer_output = (uint8_t*)&SPI_Message_transmit;
  spi_buffer_input = (uint8_t*)&SPI_Message_receive;
  spi_buffer_length = sizeof(SPI_Message_transmit);
  SpiSelectSlave0();
  SpiStart();
}

void ArduIMU_SPI_periodicGPS( void ) {

	  if (!SpiCheckAvailable()) {
	    SpiOverRun();
	    return;
	  }

	  memset(&GPS_Message_transmit, 0, sizeof(GPS_Message_transmit)); //clear packet
	  memset(&GPS_Message_receive, 0, sizeof(GPS_Message_receive)); //clear packet

	  GPS_Message_transmit.iCmdID  = ARDUIMU_GPS_SUPPLY;
	  GPS_Message_transmit.iRespID = 0xff;
	  GPS_Message_transmit.crc16 = 0;

	  if(gps.fix==3 && gps.gspeed>= 500) { //got a 3d fix and ground speed is more than 0.5 m/s)
	 	  GPS_Message_transmit.speed_3d 		= gps.speed_3d/100.; //m/s
	 	  GPS_Message_transmit.ground_speed 	= gps.gspeed/1000.;	 //m/s
	 	  GPS_Message_transmit.ground_course 	= gps.course/10.;	 //deg
	   }//if [elements are zero otherwise]
	   GPS_Message_transmit.crc16 = crc16((const char*)&GPS_Message_transmit, sizeof(GPS_Message_transmit));

	  spi_buffer_output = (uint8_t*)&GPS_Message_transmit;
	  spi_buffer_input = (uint8_t*)&GPS_Message_receive;
	  spi_buffer_length = sizeof(GPS_Message_transmit);
	  SpiSelectSlave0();
	  SpiStart();
}

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

void ArduIMU_SPI_event( void ) {
  if (spi_message_received) {
    spi_message_received = FALSE;
    ArduIMU_SPI_Parse();
#ifndef INS_ARDUIMU_READ_ONLY
    // Update estimator
    // FIXME Use a proper rotation matrix here
    EstimatorSetAtt((ins_eulers.phi - ins_roll_neutral), ins_eulers.psi, (ins_eulers.theta - ins_pitch_neutral));
#endif
    //uint8_t s = 4+VN100_REG_QMR_SIZE;
    //DOWNLINK_SEND_DEBUG(DefaultChannel,sizeof(last_received_packet),spi_buffer_output);
  }
}

void ArduIMU_SPI_Parse( void ) {

	uint16_t iCRC16 = SPI_Message_receive.crc16;
	SPI_Message_receive.crc16 = 0;

	if( iCRC16 == crc16((const char*)&SPI_Message_receive, sizeof(SPI_Message_receive)) )
	{
		ins_eulers.phi 	 = SPI_Message_receive.roll;
		ins_eulers.theta = SPI_Message_receive.pitch;
		ins_eulers.psi 	 = SPI_Message_receive.yaw;

		ins_rates.p 	= SPI_Message_receive.omega_x;
		ins_rates.q 	= SPI_Message_receive.omega_y;
		ins_rates.r		= SPI_Message_receive.omega_z;
	}


	//send debug
	uint8_t* pmsg = (uint8_t*)&SPI_Message_receive;
	DOWNLINK_SEND_DEBUG(DefaultChannel,sizeof(SPI_Message_receive),pmsg);
}

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

extern void ArduIMU_SPI_report( void ) {
  DOWNLINK_SEND_AHRS_LKF(DefaultChannel,
      &ins_eulers.phi, &ins_eulers.theta, &ins_eulers.psi,
      &ins_quat.qi, &ins_quat.qx, &ins_quat.qy, &ins_quat.qz,
      &ins_rates.p, &ins_rates.q, &ins_rates.r,
      &ins_accel.x, &ins_accel.y, &ins_accel.z,
      &ins_mag.x, &ins_mag.y, &ins_mag.z);
}


unsigned short crc16(const char *pcBuffer, const int len)
{
    const unsigned short crctable[256] =
    {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };
	int i;
	unsigned short crc = 0;

	for (i = 0; i < len; i++)
	{
		crc = (unsigned short)(crctable[(pcBuffer[i]^crc) & 0xff] ^ (crc >> 8));
	}// for all byts in the buffer

	return crc;
}
