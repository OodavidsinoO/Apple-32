/*
 * ps2.h
 *
 *  Created on: 8 Nov 2023
 *      Author: pasha
 */
#ifndef INC_PS2_H_
#define INC_PS2_H_

#include "stm32f1xx_hal.h"
// Scan codes
#define L_CTRL 		0x14
#define RELEASE 	0xF0
// "ascii" codes
#define BACK_SPC	0x7f
#define CTRL_C      0x03
#define CTRL_L		0x0C
#define CTRL_S		0x13
#define ENTER 		0x0A

typedef struct {
	uint8_t defMap[104];
	uint8_t ctrlMap[104];
} scanMaps;

uint8_t getSCode(uint8_t *byte); 	// returns decimal value of little-endian byte.
uint8_t isKbrdReady(); 				// tells whether there's a new key ready to read.
uint8_t getAscii(); 				// returns the keyboard's current ascii value
uint8_t getScanCode(); 				// return the latest scan code
void toggleKeys(); 					// to toggle ctrl and other keys


#endif /* INC_PS2_H_ */
