
/******************************************************************************************
 * Rotary-encoder decoder for Arduino
 ******************************************************************************************
 * Copyright (c) Robert Bakker 2013
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 ******************************************************************************************
 * This library handles polling and decoding of up to 5 rotary encoders.
 * It is non-blocking code that executes in the background while your sketch
 * does it's thing.
 *****************************************************************************************/

#include "EncoderPolling.h"
#include "Arduino.h"	// Has just about everything in it


/*Private #defines************************************************************************/

// Used by state machines
#define STANDBY 0
#define WAITING 1
#define IDLE 2


/*Functions Prototypes********************************************************************/

inline void encoder0_demod(void);
inline void encoder1_demod(void);
inline void encoder2_demod(void);
inline void encoder3_demod(void);
inline void encoder4_demod(void);


/*Declare Variables***********************************************************************/

struct encoderVars
{
	volatile uint8_t state;

	volatile uint8_t pin_A;
	volatile uint8_t pin_B;

	volatile int8_t data;

	volatile uint8_t pinState_A;
	volatile uint8_t pinStateOld_A;
	volatile uint8_t pinState_B;

	volatile uint8_t enable;
};

// Initialize variables (start all state machines in STANDBY state)
struct encoderVars encoder0 = { STANDBY, 0, 0, 0, 0, 0, 0, 0 };
struct encoderVars encoder1 = { STANDBY, 0, 0, 0, 0, 0, 0, 0 };
struct encoderVars encoder2 = { STANDBY, 0, 0, 0, 0, 0, 0, 0 };
struct encoderVars encoder3 = { STANDBY, 0, 0, 0, 0, 0, 0, 0 };
struct encoderVars encoder4 = { STANDBY, 0, 0, 0, 0, 0, 0, 0 };


/*Start of Functions**********************************************************************/

// This function sets up timer2 to trigger an ISR every 300 us.
void encoder_begin(void)
{
	cli();					// Disable global interrupts

	TCCR2A = 0;				// Clear timer2's control registers
	TCCR2B = 0;
	TIMSK2 = 0;				// ...and interrupt mask register (just in case)
	TCNT2 = 0;				// Pre-load the timer to 0
	OCR2A = 149;			// Set output compare register to 149
	TCCR2A |= _BV(WGM21);	// Turn on CTC mode (Clear Timer on Compare match)
	TCCR2B |= 0b011;		// Set prescaler to 32 (starts timer) 
	TIMSK2 |= _BV(OCIE2A);	// Enable timer compare interrupt 

	sei();					// Re-enable global interrupts
}


void attach_encoder(uint8_t _encNum, uint8_t _pin_A, uint8_t _pin_B)
{
	switch(_encNum)
	{
		case 0:
			encoder0.pin_A = _pin_A;
			encoder0.pin_B = _pin_B;

			pinMode(encoder0.pin_A, INPUT);
			pinMode(encoder0.pin_B, INPUT);

			encoder0.enable = 1;
			break;

		case 1:
			encoder1.pin_A = _pin_A;
			encoder1.pin_B = _pin_B;

			pinMode(encoder1.pin_A, INPUT);
			pinMode(encoder1.pin_B, INPUT);

			encoder1.enable = 1;
			break;

		case 2:
			encoder2.pin_A = _pin_A;
			encoder2.pin_B = _pin_B;

			pinMode(encoder2.pin_A, INPUT);
			pinMode(encoder2.pin_B, INPUT);

			encoder2.enable = 1;
			break;

		case 3:
			encoder3.pin_A = _pin_A;
			encoder3.pin_B = _pin_B;

			pinMode(encoder3.pin_A, INPUT);
			pinMode(encoder3.pin_B, INPUT);

			encoder3.enable = 1;
			break;

		case 4:
			encoder4.pin_A = _pin_A;
			encoder4.pin_B = _pin_B;

			pinMode(encoder4.pin_A, INPUT);
			pinMode(encoder4.pin_B, INPUT);

			encoder4.enable = 1;
			break;
	}
}


void dettach_encoder(uint8_t _encNum)
{
	switch(_encNum)
	{
		case 0:
			encoder0.enable = 0;
			encoder0.state = STANDBY;
			break;

		case 1:
			encoder1.enable = 0;
			encoder1.state = STANDBY;
			break;

		case 2:
			encoder2.enable = 0;
			encoder2.state = STANDBY;
			break;

		case 3:
			encoder3.enable = 0;
			encoder3.state = STANDBY;
			break;

		case 4:
			encoder4.enable = 0;
			encoder4.state = STANDBY;
			break;
	}
}


int8_t encoder_data(uint8_t _encNum)
{
	switch(_encNum)
	{
		case 0:
			if(encoder0.state == IDLE)
			{
				int8_t temp = encoder0.data;
				encoder0.state = STANDBY;
				return temp;
			}

			else
				return 0;

			break;

		case 1:
			if(encoder1.state == IDLE)
			{
				int8_t temp = encoder1.data;
				encoder1.state = STANDBY;
				return temp;
			}

			else
				return 0;

			break;

		case 2:
			if(encoder2.state == IDLE)
			{
				int8_t temp = encoder2.data;
				encoder2.state = STANDBY;
				return temp;
			}

			else
				return 0;

			break;

		case 3:
			if(encoder3.state == IDLE)
			{
				int8_t temp = encoder3.data;
				encoder3.state = STANDBY;
				return temp;
			}

			else
				return 0;

			break;

		case 4:
			if(encoder4.state == IDLE)
			{
				int8_t temp = encoder0.data;
				encoder4.state = STANDBY;
				return temp;
			}

			else
				return 0;

			break;
	}
}


/*ISR*************************************************************************************/

ISR(TIMER2_COMPA_vect)
{
	if(encoder0.enable)
		encoder0_demod();

	if(encoder1.enable)
		encoder1_demod();

	if(encoder2.enable)
		encoder2_demod();

	if(encoder3.enable)
		encoder3_demod();

	if(encoder4.enable)
		encoder4_demod();
}


inline void encoder0_demod(void)
{
	switch(encoder0.state)
	{
		case STANDBY:
			encoder0.pinStateOld_A = digitalRead(encoder0.pin_A);
			encoder0.state = WAITING;
			break;

		case WAITING:
			encoder0.pinState_A = digitalRead(encoder0.pin_A);
			if(encoder0.pinState_A != encoder0.pinStateOld_A)
			{
				encoder0.pinState_B = digitalRead(encoder0.pin_B);
				if(encoder0.pinState_A == encoder0.pinState_B)
					encoder0.data = 1;

				else
					encoder0.data = -1;

				encoder0.state = IDLE;
			}
			break;
	}
}

inline void encoder1_demod(void)
{
	switch(encoder1.state)
	{
		case STANDBY:
			encoder1.pinStateOld_A = digitalRead(encoder1.pin_A);
			encoder1.state = WAITING;
			break;

		case WAITING:
			encoder1.pinState_A = digitalRead(encoder1.pin_A);
			if(encoder1.pinState_A != encoder1.pinStateOld_A)
			{
				encoder1.pinState_B = digitalRead(encoder1.pin_B);
				if(encoder1.pinState_A == encoder1.pinState_B)
					encoder1.data = 1;

				else
					encoder1.data = -1;

				encoder1.state = IDLE;
			}
			break;
	}
}

inline void encoder2_demod(void)
{
	switch(encoder2.state)
	{
		case STANDBY:
			encoder2.pinStateOld_A = digitalRead(encoder2.pin_A);
			encoder2.state = WAITING;
			break;

		case WAITING:
			encoder2.pinState_A = digitalRead(encoder2.pin_A);
			if(encoder2.pinState_A != encoder2.pinStateOld_A)
			{
				encoder2.pinState_B = digitalRead(encoder2.pin_B);
				if(encoder2.pinState_A == encoder2.pinState_B)
					encoder2.data = 1;

				else
					encoder2.data = -1;

				encoder2.state = IDLE;
			}
			break;
	}
}

inline void encoder3_demod(void)
{
	switch(encoder3.state)
	{
		case STANDBY:
			encoder3.pinStateOld_A = digitalRead(encoder3.pin_A);
			encoder3.state = WAITING;
			break;

		case WAITING:
			encoder3.pinState_A = digitalRead(encoder3.pin_A);
			if(encoder3.pinState_A != encoder3.pinStateOld_A)
			{
				encoder3.pinState_B = digitalRead(encoder3.pin_B);
				if(encoder3.pinState_A == encoder3.pinState_B)
					encoder3.data = 1;

				else
					encoder3.data = -1;

				encoder3.state = IDLE;
			}
			break;
	}
}

inline void encoder4_demod(void)
{
	switch(encoder4.state)
	{
		case STANDBY:
			encoder4.pinStateOld_A = digitalRead(encoder4.pin_A);
			encoder4.state = WAITING;
			break;

		case WAITING:
			encoder4.pinState_A = digitalRead(encoder4.pin_A);
			if(encoder4.pinState_A != encoder4.pinStateOld_A)
			{
				encoder4.pinState_B = digitalRead(encoder4.pin_B);
				if(encoder4.pinState_A == encoder4.pinState_B)
					encoder4.data = 1;

				else
					encoder4.data = -1;

				encoder4.state = IDLE;
			}
			break;
	}
}