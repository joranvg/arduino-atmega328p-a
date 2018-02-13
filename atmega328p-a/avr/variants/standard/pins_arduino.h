/*
  pins_arduino.h - Pin definition functions for Arduino

  Part of 'ATmega328P-A (8 MHz internal clock)' board definition.
	Adds support for the following additional pins:

	D14 (PB6) and D15 (PB7)
	External oscillator pins, available because the internal oscillator is used.

	A6 (ADC6) and A7 (ADC7)
	Analog input pins, only available on the 32-pin variants.
	Cannot be used as digital I/O.

	The definition moves the pin numbers of the analog pins up by 2 (A0 = 16).

	Joran van Gremberghe, Joranalogue Audio Design, http://joranalogue.com/
	2017-01-21, cc-by-sa 2.0
	Based on default Arduino pins-arduino.h by David A. Mellis, 2007.
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            22
#define NUM_ANALOG_INPUTS           8
#define digitalPinHasPWM(p)         ((p) == 3 || (p) == 5 || (p) == 6 || (p) == 9 || (p) == 10 || (p) == 11)

static const uint8_t SS   = 10;
static const uint8_t MOSI = 11;
static const uint8_t MISO = 12;
static const uint8_t SCK  = 13;

#define LED_BUILTIN 13

static const uint8_t A0 = 16;
static const uint8_t A1 = 17;
static const uint8_t A2 = 18;
static const uint8_t A3 = 19;
static const uint8_t A4 = 20;
static const uint8_t A5 = 21;
static const uint8_t A6 = 22;
static const uint8_t A7 = 23;

extern const uint8_t PROGMEM analog_pin_to_channel_PGM[];
#define analogPinToChannel(p)    (pgm_read_byte(analog_pin_to_channel_PGM + ((p) - 16)))

#define digitalPinToPCICR(p)     (((p) >= 0 && (p) <= 21) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p)  (((p) <= 7) ? 2 : (((p) <= 15) ? 0 : 1))
#define digitalPinToPCMSK(p)     (((p) <= 7) ? (&PCMSK2) : (((p) <= 15) ? (&PCMSK0) : (((p) <= 21) ? (&PCMSK1) : ((uint8_t *)0))))
#define digitalPinToPCMSKbit(p)  (((p) <= 7) ? (p) : (((p) <= 15) ? ((p) - 8) : ((p) - 16)))

#define digitalPinToInterrupt(p) ((p) == 2 ? 0 : ((p) == 3 ? 1 : NOT_AN_INTERRUPT))

#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[22] = {
	PD, /* 0 */
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PB, /* 8 */
	PB,
	PB,
	PB,
	PB,
	PB,
	PB,
	PB,
	PC, /* 16 */
	PC,
	PC,
	PC,
	PC,
	PC
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[22] = {
	_BV(0), /* 0, port D */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0), /* 8, port B */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0), /* 16, port C */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5)
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[22] = {
	NOT_ON_TIMER, /* 0 - port D */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	TIMER2B,
	NOT_ON_TIMER,
	TIMER0B,
	TIMER0A,
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 8 - port B */
	TIMER1A,
	TIMER1B,
	TIMER2A,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 16 - port C */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER
};

const uint8_t PROGMEM analog_pin_to_channel_PGM[8] = {
	0, // A0   ADC0   D16   PC0
	1, // A1   ADC1   D17   PC1
	2, // A2   ADC2   D18   PC2
	3, // A3   ADC3   D19   PC3
	4, // A4   ADC4   D20   PC4
	5, // A5   ADC5   D21   PC5
	6, // A6   ADC6
	7  // A7   ADC7
};

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR   Serial
#define SERIAL_PORT_HARDWARE  Serial

#endif
