/* trackuino copyright (C) 2010  EA5HAV Javi */
#if (ARDUINO + 0) == 0
#error "Oops! We need the real Arduino IDE (version 22 or 23) for Arduino builds."
#error "See trackuino.pde for details on this"

// Refuse to compile on arduino version 21 or lower. 22 includes an
// optimization of the USART code that is critical for real-time operation
// of the AVR code.
#elif (ARDUINO + 0) < 22
#error "Oops! We need Arduino 22 or 23"
#error "See trackuino.pde for details on this"

#endif

// Trackuino custom libs
#include "config.h"
#include "afsk_avr.h"
#include "afsk_avr32u4.h"
#include "aprs.h"
#include "buzzer.h"
#include "gps.h"
#include "pin.h"
#include "power.h"
#include "sensors_avr.h"

#ifdef LCD_ENABLED
// Incluimos libreria de Pantalla LCD
#include <LiquidCrystal.h>
#endif

// Arduino/AVR libs
#if (ARDUINO + 1) >= 100
#  include <Arduino.h>
#else
#  include <WProgram.h>
#endif


// Module constants
static const uint32_t VALID_POS_TIMEOUT = 2000;  // ms

// Module variables
static int32_t next_aprs = 0;

// initialize the library with the numbers of the interface pins
#ifdef LCD_ENABLED
LiquidCrystal lcd(LCD_PINS);
#endif

#define DEBUG_SERIAL Serial

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pin_write(LED_PIN, LOW);

  GPS_SERIAL.begin(GPS_BAUDRATE);

#if defined(__AVR_ATmega32U4__) && (defined(DEBUG_GPS) || defined(DEBUG_RESET) || defined(DEBUG_MODEM) || defined(DEBUG_SENS) || defined(DEBUG_AX25))
  DEBUG_SERIAL.begin(115200);
  // Wait for serial port to connect. Needed for Atmega 32u4 processors only
  while (!DEBUG_SERIAL) {}
#endif

#ifdef DEBUG_RESET
  DEBUG_SERIAL.println("RESET");
#endif

#ifndef GPS_DISABLED
  buzzer_setup();
#endif
  afsk_setup();
#ifndef GPS_DISABLED
  gps_setup();
#endif
  //  sensors_setup();

#ifdef DEBUG_SENS
#ifndef INTERNAL_LM60_DISABLED
  DEBUG_SERIAL.print("i=");
  DEBUG_SERIAL.print(sensors_int_lm60());
  DEBUG_SERIAL.print(", ");
#endif
#ifndef EXTERNAL_LM60_DISABLED
  DEBUG_SERIAL.print("e=");
  DEBUG_SERIAL.print(sensors_ext_lm60());
  DEBUG_SERIAL.print(", ");
#endif
  DEBUG_SERIAL.print("Vin=");
  DEBUG_SERIAL.print(sensors_vin());
#endif
#ifdef LCD_ENABLED
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
#endif

#ifndef GPS_DISABLED
  // Do not start until we get a valid time reference
  // for slotted transmissions.
  if (APRS_SLOT >= 0) {
    do {
      while (! GPS_SERIAL.available())

        power_save();
    } while (! gps_decode(GPS_SERIAL.read()));

    next_aprs = millis() + 1000 *
                (APRS_PERIOD - (gps_seconds + APRS_PERIOD - APRS_SLOT) % APRS_PERIOD);
  }
  else {
    next_aprs = millis();
  }
#else
  next_aprs = millis();
#endif
  // TODO: beep while we get a fix, maybe indicating the number of
  // visible satellites by a series of short beeps?

#ifdef LCD_ENABLED
  // Print a message to the LCD.
  lcd.print(S_CALLSIGN);
  lcd.print("-");
  lcd.print(S_CALLSIGN_ID);
#ifndef GPS_DISABLED
  get_pos();
  lcd.print(" Sats=");
  lcd.print(gps_sats);
#endif
#endif
}


void loop()
{
  // Time for another APRS frame
  if ((int32_t) (millis() - next_aprs) >= 0) {

    ////Añadiendo para mostrar en monitor serial la hora
#ifdef DEBUG_SENS
#ifndef GPS_DISABLED
    get_pos();
    DEBUG_SERIAL.print("Time=");
    DEBUG_SERIAL.println(gps_time);
#endif
#endif
    //// Fin de añadir en monitor serial la hora

#ifndef GPS_DISABLED
    get_pos();
#endif
#ifdef LCD_ENABLED
    lcd.setCursor(0, 0);
    //Añadido para jugar con gps 23/06/15
    lcd.print(gps_time);
#ifndef INTERNAL_LM60_DISABLED
    lcd.print(" Ti=");
    lcd.print(sensors_int_lm60());
#endif
#endif
#ifndef EXTERNAL_LM60_DISABLED
    lcd.setCursor(0, 1);
    lcd.print("Te= ");
    lcd.print(sensors_ext_lm60());
    lcd.print(",Vi=");
    lcd.print(sensors_vin());
#endif
    aprs_send();
    next_aprs += APRS_PERIOD * 1000L;
    while (afsk_flush())
    {
      power_save();
    }

#ifdef DEBUG_MODEM
    // Show modem ISR stats from the previous transmission
    afsk_debug();
#endif
  }

  power_save(); // Incoming GPS data or interrupts will wake us up
}

void get_pos()
{
  // Get a valid position from the GPS
  int valid_pos = 0;
  uint32_t timeout = millis();
  do {
    if (GPS_SERIAL.available())
      valid_pos = gps_decode(GPS_SERIAL.read());
  } while ( (millis() - timeout < VALID_POS_TIMEOUT) && ! valid_pos) ;

#ifndef GPS_DISABLED
  if (valid_pos) {
    if (gps_altitude > BUZZER_ALTITUDE) {
      buzzer_off();   // In space, no one can hear you buzz
    } else {
      buzzer_on();
    }
  }
#endif
}
