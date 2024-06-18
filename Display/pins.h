///////////////////////////////////////////////////////////////////////////////
// PINS.H
//
// This file contains pin definitions only. One-stop shop if you need
// to change the pins you are using
//
// V1.8
//
// Dr J A Gow 2017
//////////////////////////////////////////////////////////////////////////////

#ifndef PINS_H_
#define PINS_H_

//
// DISPLAY CONFIGURATION

#define DISPLAY_I2C_ADDR	0x27
//#define DISPLAY_I2C_ADDR	0x3f

//
// PINS USED

// pin declarations

#define ANALOG_IN			A0		// used for DC and AC ranges
#define MODE_CHANGE_PIN		2		// active low range change input
#define RANGE_CHANGE_PIN	3		// active low range change input
#define RANGE_RELAY_PIN		8		// relay output for range change

#define RANGE_NO_PIN		0		// for ranges that don't require a pin setting

#define MS_KBD_RPT			600		// ms to wait to debounce/key repeat

//
// SCALE FACTORS

#define SCALE_DCV_2VMAX		(5.0 / 1023.0)
#define SCALE_DCV_20VMAX	0.5
#define SCALE_ACV_20VMAX	1.0

//
// Oscillator

// Any lower than 200Hz, and we're below the point where the
// state variables can be accurately rendered in a 1.15 fixed point.

#define	PWMOSC_MIN_FREQ		200		// start frequency
#define PWMOSC_MAX_FREQ		7000	// maximum frequency from PWM generator
#define PWMOSC_FREQ_STEP	100		// frequency step

#endif /* PINS_H_ */
