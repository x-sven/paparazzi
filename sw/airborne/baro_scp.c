#include <stdio.h>
#include <math.h>
#include "std.h"

#include "baro_scp_hw.h"
#include "baro_scp.h"

// Values for estimator
float baro_scp_r = 20.;
float baro_scp_sigma2 = 1.;
float baro_scp_z = 0.;

// global variables
float baro_scp_ground_height = 0.0;
uint32_t baro_scp_ground_pressure = 10240000;

// local variables
static float Tn = 0.; //!< Temp am Boden in Kelvin zum Zeitpkt der Kalibierung
static float Pn = 0.; //!< Druck am Boden hPa zum Zeitpunkt der Kalibierung
static float Hn = 0.; //!< Hoehe über NN zum Zeitpunkt der Kalibierung

/**
 * Kalibierung des Drucks und der Temperatur am Boden
 *
 * \author Oliver Riesener
 *
 * \param [in] h_über_nn Aktuelle Hoehe ueber NN
 *
 * \return Setzen der Modul lokalen Variablen Tn, Pn, Hn
 */
void baro_scp_calibrate( float h_ueber_nn) {
    // warten auf baro_scp_available
    while( !baro_scp_available );
    // speichern der werte am boden
    Tn = 273.15+baro_scp_temperature;
    Pn = baro_scp_pressure;
    Hn = h_ueber_nn;
    baro_scp_ground_height = h_ueber_nn;
    // global den druck merken
    baro_scp_ground_pressure = baro_scp_pressure;
}

/**
 * Barometrische Hoehenformel mit Temperatur
 *
 * \author Oliver Riesener
 *
 * \param [in] Ph Druck in der Höhe [hP]
 * \param [in] Th Temperatur [gC] in der Höhe
 */
float baro_scp_height( unsigned int Ph, float Th ) {
    // Th in Kelvin umrechnen
    Th += 273.15;
    const float g = 9.80665; //!< Schwerebeschleunigung der Erde [m/s^2]
    float R = 287.; //!< Gaskonstante fuer Luft [J/kgK]
    float Tm = (Tn+Th)/2; //!< Mitteltemperatur (Grad Kelvin) zwischen ph, pn) 
    float h;
    if ( Ph != 0. ) {
	h=R*Tm/g*logf(Pn/Ph); //!< Berechnung der Hoehe in [m]
    } else {
	h=33333; // Sensor kaputt oder im Weltall
    }
    return h;
}
