/*****************************************************************************|
|            MIGHTYCORE WIDE PINOUT         																|
|				https://github.com/mrguen/MightyCore																	|
| Forked from https://github.com/MCUdude/MightyCore 													|
|                                                   													|
| An Arduino core with variants for boards: 																	|
|                                                   													|
| - 644 WIDE 3.3V																													|
| - 644 WIDE 5V																														|
| - 644 POWER 																																|
| - 1284 WIDE 3.3V																													|
| - 1284 WIDE 5V																														|
| - 1284 POWER 																																|
|																																							|
|*****************************************************************************/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#if defined(__AVR_ATmega644A__)
  #define __AVR_ATmega644__
#endif

// We're using the REGULAR pinout
#define REGULAR_PINOUT
#define NUM_DIGITAL_PINS          (32)


// PWM pins
#if defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__)
  #define digitalPinHasPWM(p)       ((p) == 7 || (p) == 10 || (p) == 30 || (p) == 8 || (p) == 9 || (p) == 31)
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)
  #define digitalPinHasPWM(p)       ((p) == 7 || (p) == 10 || (p) == 12 || (p) == 13 || (p) == 30 || (p) == 8 || (p) == 9 || (p) == 31)
#endif

// Builtin LED
#define LED_BUILTIN   (13)
static const uint8_t LED = LED_BUILTIN;

// Analog pins
#define PIN_A0 (16)
#define PIN_A1 (17)
#define PIN_A2 (18)
#define PIN_A3 (19)
#define PIN_A4 (20)
#define PIN_A5 (21)
#define PIN_A6 (22)
#define PIN_A7 (23)
static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;
static const uint8_t A6 = PIN_A6;
static const uint8_t A7 = PIN_A7;
#define NUM_ANALOG_INPUTS           (8)
#define analogInputToDigitalPin(p)  ((p < NUM_ANALOG_INPUTS) ? (p) + 16 : -1)
#define analogPinToChannel(p)       ((p) < NUM_ANALOG_INPUTS ? (p) : (p) >= 16 ? (p) - 16 : -1)

// SPI
#define PIN_SPI_SS    (10)
#define PIN_SPI_MOSI  (11)
#define PIN_SPI_MISO  (12)
#define PIN_SPI_SCK   (13)
static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

// i2c
#define PIN_WIRE_SDA  (14)
#define PIN_WIRE_SCL  (15)
static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

// Interrupts
#define EXTERNAL_NUM_INTERRUPTS     (3)
#define digitalPinToInterrupt(p)    ((p) == 6 ? 2 : ((p) == 2 ? 0 : ((p) == 3 ? 1 : NOT_AN_INTERRUPT)))

// PCINT
#define PORT_NDX_TO_PCMSK(x) ((x) == 0 ? &PCMSK0 : ((x) == 1 ? &PCMSK1 : ((x) == 2 ? &PCMSK2 : ((x) == 3 ? &PCMSK3 : (uint8_t *)0))))
#define ifpin(p,what,ifnot)         (((p) >= 0 && (p) < NUM_DIGITAL_PINS) ? (what) : (ifnot))
#define digitalPinToPCICR(p)    ifpin(p,&PCICR,(uint8_t *)0)
#define digitalPinToPCICRbit(p) ifpin(p,digital_pin_to_pcint[p] >> 3,0)
#define digitalPinToPCMSK(p)    ifpin(p,(uint8_t *)PORT_NDX_TO_PCMSK(digital_pin_to_pcint[p] >> 3),(uint8_t *)0)
#define digitalPinToPCMSKbit(p) ifpin(p,digital_pin_to_pcint[p] & 0x7,0)

// Digital pin numbering

#define PIN_PD0 0
#define PIN_PD1 1
#define PIN_PD2 2
#define PIN_PD3 3
#define PIN_PB0 4
#define PIN_PB1 5
#define PIN_PB2 6
#define PIN_PB3 7
#define PIN_PD5 8
#define PIN_PD6 9
#define PIN_PB4 10
#define PIN_PB5 11
#define PIN_PB6 12
#define PIN_PB7 13
#define PIN_PC1 14
#define PIN_PC0 15

#define PIN_PA0 16
#define PIN_PA1 17
#define PIN_PA2 18
#define PIN_PA3 19
#define PIN_PA4 20
#define PIN_PA5 21
#define PIN_PA6 22
#define PIN_PA7 23

#define PIN_PC2 24
#define PIN_PC3 25
#define PIN_PC4 26
#define PIN_PC5 27
#define PIN_PC6 28
#define PIN_PC7 29
#define PIN_PD4 30
#define PIN_PD7 31

#ifndef ARDUINO_MAIN
extern const uint8_t digital_pin_to_pcint[];
#else
const uint8_t digital_pin_to_pcint[NUM_DIGITAL_PINS] =
{
  24, // D0 - PD0
  25, // D1 - PD1
  26, // D2 - PD2
  27, // D3 - PD3
  8,  // D4 - PB0
  9,  // D5 - PB1
  10, // D6 - PB2
  11, // D7 - PB3
  29, // D8 - PD5
  30, // D9 - PD6
  12, // D10 - PB4
  13, // D11 - PB5
  14, // D12 - PB6
  15, // D13 - PB7
  17, // D23 - PC1
  16, // D22 - PC0
  0,  // D21 - PA0
  1,  // D20 - PA1
  2,  // D19 - PA2
  3,  // D18 - PA3
  4,  // D17 - PA4
  5,  // D16 - PA5
  6,  // D15 - PA6
  7,  // D14 - PA7
  18, // D24 - PC2
  19, // D25 - PC3
  20, // D26 - PC4
  21, // D27 - PC5
  22, // D28 - PC6
  23, // D29 - PC7
  28, // D30 - PD4
  31, // D31 - PD7
};
#endif


#ifdef ARDUINO_MAIN

#define PA 1
#define PB 2
#define PC 3
#define PD 4

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] =
{
  NOT_A_PORT,
  (uint16_t) &DDRA,
  (uint16_t) &DDRB,
  (uint16_t) &DDRC,
  (uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] =
{
  NOT_A_PORT,
  (uint16_t) &PORTA,
  (uint16_t) &PORTB,
  (uint16_t) &PORTC,
  (uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] =
{
  NOT_A_PORT,
  (uint16_t) &PINA,
  (uint16_t) &PINB,
  (uint16_t) &PINC,
  (uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] =
{
  PD, /* PD0 */
  PD, /* PD1 */
  PD, /* PD2 */
  PD, /* PD3 */
  PB, /* PB0 */
  PB, /* PB1 */
  PB, /* PB2 */
  PB, /* PB3 */
  PD, /* PD5 */
  PD, /* PD6 */
  PB, /* PB4 */
  PB, /* PB5 */
  PB, /* PB6 */
  PB, /* PB7 */
  PC, /* PC1 */
  PC, /* PC0 */
  PA, /* PA0 */
  PA, /* PA1 */
  PA, /* PA2 */
  PA, /* PA3 */
  PA, /* PA4 */
  PA, /* PA5 */
  PA, /* PA6 */
  PA, /* PA7 */
  PC, /* PC2 */
  PC, /* PC3 */
  PC, /* PC4 */
  PC, /* PC5 */
  PC, /* PC6 */
  PC, /* PC7 */
  PD, /* PD4 */
  PD, /* PD7 */
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] =
{
  _BV(0),  /* PD0 */
  _BV(1),  /* PD1 */
  _BV(2),  /* PD2 */
  _BV(3),  /* PD3 */
  _BV(0),  /* PB0 */
  _BV(1),  /* PB1 */
  _BV(2),  /* PB2 */
  _BV(3),  /* PB3 */
  _BV(5),  /* PD5 */
  _BV(6),  /* PD6 */
  _BV(4),  /* PB4 */
  _BV(5),  /* PB5 */
  _BV(6),  /* PB6 */
  _BV(7),  /* PB7 */
  _BV(1),  /* PC1 */
  _BV(0),  /* PC0 */
  _BV(0),  /* PA0 */
  _BV(1),  /* PA1 */
  _BV(2),  /* PA2 */
  _BV(3),  /* PA2 */
  _BV(4),  /* PA4 */
  _BV(5),  /* PA5 */
  _BV(6),  /* PA6 */
  _BV(7),  /* PA7 */
  _BV(2),  /* PC2 */
  _BV(3),  /* PC3 */
  _BV(4),  /* PC4 */
  _BV(5),  /* PC5 */
  _BV(6),  /* PC6 */
  _BV(7),  /* PC7 */
  _BV(4),  /* PD4 */
  _BV(7),  /* PD7 */
};


#if defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__)
const uint8_t PROGMEM digital_pin_to_timer_PGM[] =
{
  NOT_ON_TIMER,   /* D0 	- PD0 */
  NOT_ON_TIMER,   /* D1 	- PD1 */
  NOT_ON_TIMER,   /* D2 	- PD2 */
  NOT_ON_TIMER,   /* D3 	- PD3 */
  NOT_ON_TIMER,   /* D4 	- PB0 */
  NOT_ON_TIMER,   /* D5 	- PB1 */
  NOT_ON_TIMER,   /* D6 	- PB2 */
  TIMER0A,        /* D7 	- PB3 */
  TIMER1A,        /* D8 	- PD5 */
  TIMER2B,        /* D9 	- PD6 */
  TIMER0B,        /* D10  - PB4 */
  NOT_ON_TIMER,   /* D11 	- PB5 */
  NOT_ON_TIMER,   /* D12 	- PB6 */
  NOT_ON_TIMER,   /* D13  - PB7 */
  NOT_ON_TIMER,   /* D14 	- PC1 */
  NOT_ON_TIMER,   /* D15 	- PC0 */
  NOT_ON_TIMER,   /* D16 	- PA0 */
  NOT_ON_TIMER,   /* D17 	- PA1 */
  NOT_ON_TIMER,   /* D18 	- PA2 */
  NOT_ON_TIMER,   /* D19 	- PA3 */
  NOT_ON_TIMER,   /* D20 	- PA4 */
  NOT_ON_TIMER,   /* D21 	- PA5 */
  NOT_ON_TIMER,   /* D22 	- PA6 */
  NOT_ON_TIMER,   /* D23 	- PA7 */
  NOT_ON_TIMER,   /* D24 - PC2 */
  NOT_ON_TIMER,   /* D25 - PC3 */
  NOT_ON_TIMER,   /* D26 - PC4 */
  NOT_ON_TIMER,   /* D27 - PC5 */
  NOT_ON_TIMER,   /* D28 - PC6 */
  NOT_ON_TIMER,   /* D29 - PC7 */
  TIMER1B,        /* D30 - PD4 */
  TIMER2A        	/* D31 - PD7 */
};

#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)
const uint8_t PROGMEM digital_pin_to_timer_PGM[] =
{
  NOT_ON_TIMER,   /* D0 	- PD0 */
  NOT_ON_TIMER,   /* D1  	- PD1 */
  NOT_ON_TIMER,   /* D2 	- PD2 */
  NOT_ON_TIMER,   /* D3 	- PD3 */
  NOT_ON_TIMER,   /* D4  	- PB0 */
  NOT_ON_TIMER,   /* D5  	- PB1 */
  NOT_ON_TIMER,   /* D6  	- PB2 */
  TIMER0A,        /* D7  	- PB3 */
  TIMER1A,        /* D8 	- PD5 */
  TIMER2B,        /* D9 	- PD6 */
 	TIMER0B,        /* D10  - PB4 */
  NOT_ON_TIMER,   /* D11  - PB5 */
  TIMER3A,        /* D12  - PB6 */
  TIMER3B,        /* D13  - PB7 */
  NOT_ON_TIMER,   /* D14 	- PC1 */
  NOT_ON_TIMER,   /* D15 	- PC0 */
  NOT_ON_TIMER,   /* D16 	- PA0 */
  NOT_ON_TIMER,   /* D17	- PA1 */
  NOT_ON_TIMER,   /* D18 	- PA2 */
  NOT_ON_TIMER,   /* D19 	- PA3 */
  NOT_ON_TIMER,   /* D20 	- PA4 */
  NOT_ON_TIMER,   /* D21 	- PA5 */
  NOT_ON_TIMER,   /* D22 	- PA6 */
  NOT_ON_TIMER,   /* D23 	- PA7 */
  NOT_ON_TIMER,   /* D24 	- PC2 */
  NOT_ON_TIMER,   /* D25 	- PC3 */
  NOT_ON_TIMER,   /* D26 	- PC4 */
  NOT_ON_TIMER,   /* D27 	- PC5 */
  NOT_ON_TIMER,   /* D28 	- PC6 */
  NOT_ON_TIMER,   /* D29 	- PC7 */
  TIMER1B,        /* D30 	- PD4 */
  TIMER2A        	/* D31 	- PD7 */
};

#endif // Timer defs

#endif // ARDUINO_MAIN

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.

#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE_OPEN   Serial

#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE_OPEN1  Serial1

#endif // Pins_Arduino_h
