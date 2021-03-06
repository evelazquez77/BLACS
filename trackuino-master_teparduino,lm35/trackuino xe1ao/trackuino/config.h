/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__


// --------------------------------------------------------------------------
// THIS IS THE TRACKUINO FIRMWARE CONFIGURATION FILE. YOUR CALLSIGN AND
// OTHER SETTINGS GO HERE.
//
// NOTE: all pins are Arduino based, not the Atmega chip. Mapping:
// http://www.arduino.cc/en/Hacking/PinMapping
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Board Config
// --------------------------------------------------------------------------

// You must include a board config file that specifies the pins used by your
// hardware.

//#include "config_board_tiny_trackuino.h"
#include "config_board_trackuino_shield.h"

// --------------------------------------------------------------------------
// APRS config (aprs.c)
// --------------------------------------------------------------------------

// Set your callsign and SSID here. Common values for the SSID are
// (from http://zlhams.wikidot.com/aprs-ssidguide):
//
// - Balloons:  11
// - Cars:       9
// - Home:       0
// - IGate:      5
#define S_CALLSIGN      "XE1GYL"
#define S_CALLSIGN_ID   12

// Destination callsign: APRS (with SSID=0) is usually okay.
#define D_CALLSIGN      "APRS"
#define D_CALLSIGN_ID   0

// Digipeating paths:
// (read more about digipeating paths here: http://wa8lmf.net/DigiPaths/ )
// The recommended digi path for a balloon is WIDE2-1 or pathless. The default
// is pathless. Uncomment the following two lines for WIDE2-1 path:
#define DIGI_PATH1      "WIDE1"
#define DIGI_PATH1_TTL  1
#define DIGI_PATH2      "WIDE2"
#define DIGI_PATH2_TTL  2

// Symbol table and code: '/' for primary table, code 'O' for balloon, '-' for QTH.
#define SYMBOL_TABLE    '/'
#define SYMBOL_CODE     'v'

// APRS comment: this goes in the comment portion of the APRS message. You
// might want to keep this short. The longer the packet, the more vulnerable
// it is to noise. 
#define APRS_COMMENT    "Trackduino de XE1GYL"


// --------------------------------------------------------------------------
// AX.25 config (ax25.cpp)
// --------------------------------------------------------------------------

// TX delay in milliseconds
#define TX_DELAY      500

// --------------------------------------------------------------------------
// Tracker config (trackuino.ino)
// --------------------------------------------------------------------------

// APRS packets are slotted so that multiple trackers can be used without
// them stepping on one another. The transmission times are governed by
// the formula:
//
//         APRS_SLOT (seconds) + n * APRS_PERIOD (seconds)
//
// When launching multiple balloons, use the same APRS_PERIOD in all balloons
// and set APRS_SLOT so that the packets are spaced equally in time.
// Eg. for two balloons and APRS_PERIOD = 60, set APRS_SLOT to 0 and 30, 
// respectively. The first balloon will transmit at 00:00:00, 00:01:00, 
// 00:02:00, etc. and the second balloon will transmit at 00:00:30, 00:01:30,
// 00:02:30, etc.
#define APRS_SLOT     0     // seconds. -1 disables slotted transmissions
#define APRS_PERIOD   2    // seconds //Estaba en 60

// GPS baud rate (in bits per second). This is also the baud rate at which
// debug data will be printed out the serial port.
#define GPS_BAUDRATE  9600

// Disable GPS and just use a predefined latitude and longitude.
// Make sure to use the correct format. You may also define an altitude (in meters).
//#define GPS_DISABLED
//#define LATITUDE "3802.75N"
//#define LONGITUDE "02340.90E"
//#define ALTITUDE 0


// --------------------------------------------------------------------------
// Modem config (afsk.cpp)
// --------------------------------------------------------------------------

// Pre-emphasize the 2200 tone by 6 dB. This is actually done by 
// de-emphasizing the 1200 tone by 6 dB and it might greatly improve
// reception at the expense of poorer FM deviation, which translates
// into an overall lower amplitude of the received signal. 1 = yes, 0 = no.
#define PRE_EMPHASIS    1

// --------------------------------------------------------------------------
// Sensors config (sensors.cpp)
// --------------------------------------------------------------------------

//#define INTERNAL_LM60_DISABLED
//#define EXTERNAL_LM60_DISABLED

// Units for temperature sensors (Added by: Kyle Crockett)
// 1 = Celsius, 2 = Kelvin, 3 = Fahrenheit
#define TEMP_UNIT 1

// Calibration value in the units selected. Use integer only.
#define CALIBRATION_VAL 0

// Resistors divider for the voltage meter (ohms)
#define VMETER_R1       10000
#define VMETER_R2       3300



// --------------------------------------------------------------------------
// Buzzer config (buzzer.cpp)
// --------------------------------------------------------------------------

// Type of buzzer (0=active, 1=passive). An active buzzer is driven by a
// DC voltage. A passive buzzer needs a PWM signal.
#define BUZZER_TYPE             0

// When using a passive buzzer, specify the PWM frequency here. Choose one
// that maximizes the volume according to the buzzer's datasheet. Not all
// the frequencies are valid, check out the buzzer_*.cpp code. On Arduino,
// it must be between L and 65535, where L = F_CPU / 65535 and F_CPU is the
// clock rate in hertzs. For 16 MHz Arduinos, this gives a lower limit of 
// 245 Hz.
#define BUZZER_FREQ             895     // Hz

// These are the number of seconds the buzzer will stay on/off alternately
#define BUZZER_ON_TIME          1       // secs
#define BUZZER_OFF_TIME         2       // secs

// This option disables the buzzer above BUZZER_ALTITUDE meters. This is a
// float value, so make it really high (eg. 1000000.0 = 1 million meters)
// if you want it to never stop buzzing.
#define BUZZER_ALTITUDE         3000.0  // meters (1 ft = 0.3048 m)

// --------------------------------------------------------------------------
// Debug
// --------------------------------------------------------------------------

// Debug info includes printouts from different modules to aid in testing and
// debugging.
//
// Some of the DEBUG modes will cause invalid modulation, so do NOT forget
// to turn them off when you put this to real use.
//
// Particularly the DEBUG_AFSK will print every PWM sample out the serial
// port, causing extreme delays in the actual AFSK transmission.
// 
// 1. To properly receive debug information, only connect the Arduino RX pin 
//    to the GPS TX pin, and leave the Arduino TX pin disconnected. 
//
// 2. On the serial monitor, set the baudrate to GPS_BAUDRATE (above),
//    usually 9600.
//
// 3. When flashing the firmware, disconnect the GPS from the RX pin or you
//    will get errors.

//#define DEBUG_GPS    // GPS sentence dump and checksum validation
// #define DEBUG_AX25   // AX.25 frame dump
// #define DEBUG_MODEM  // Modem ISR overrun and profiling
//#define DEBUG_AFSK   // AFSK (modulation) output
// #define DEBUG_RESET  // AVR reset
// #define DEBUG_SENS   // Sensors

#endif

