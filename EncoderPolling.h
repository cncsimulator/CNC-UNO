
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

#ifndef Encoder_Polling_V2_H_
#define Encoder_Polling_V2_H_

#include "Arduino.h"	// Has just about everything in it


/*Functions Prototypes********************************************************************/

// Sets up timer2.
void encoder_begin(void);

// Attaches an encoder to pins A and B, and starts polling it.
void attach_encoder(uint8_t _encNum, uint8_t _pin_A, uint8_t _pin_B);
// Stops polling the encoder entered.
void dettach_encoder(uint8_t _encNum);

// Returns a 1 or -1, depending on the direction, if the encoder entered has turned.
// Returns a 0 otherwise.
int8_t encoder_data(uint8_t _encNum);


#endif