/*
 * ps2.c
 *
 *  Created on: Nov 2, 2023
 *      Author: pasha
 */


#include "ps2.h"
#include "ps2_scanMaps.h"
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f1xx_it.h"
#include "lcd.h"

// struct for storing keyboard data
struct Ps2 {
	GPIO_TypeDef * clockPort;
	uint16_t clockNumber;
	GPIO_TypeDef * dataPort;
	uint16_t dataNumber;
	uint8_t count;
	uint8_t reading;
	uint8_t scanCode[9];
	uint8_t ctrlStatus;
	uint8_t parity;
	uint8_t sCodeBuffer[2];
	uint8_t readyToRead;
	uint8_t debug;
};

struct Ps2 keyboard = {
		.clockPort = GPIOA,
		.clockNumber = GPIO_PIN_5,
		.dataPort = GPIOA,
		.dataNumber = GPIO_PIN_6,
		.count = 0,
		.reading = 0,
		.ctrlStatus = 0,
		.parity = 0,
		.sCodeBuffer = {RELEASE, RELEASE},
		.readyToRead = 0,
		.debug = 0
};

// convert byte into dec value
uint8_t getSCode(uint8_t *byte){
	double output = 0;
	for(int i=0; i<8; i++){
		if(byte[i]) output += pow(2, i);
	}
	return (uint8_t)output;
}

uint8_t isKbrdReady(){
	return keyboard.readyToRead;
}

uint8_t getAscii(){
	// key read now, no longer new key to read
	keyboard.readyToRead = 0;
	if(keyboard.ctrlStatus){
		return asciiMaps.ctrlMap[keyboard.sCodeBuffer[0]];
	}
	else{
		return asciiMaps.defMap[keyboard.sCodeBuffer[0]];
	}
}

void toggleKeys(){
	// ctrl toggle
	if(keyboard.sCodeBuffer[0] == L_CTRL){
		keyboard.ctrlStatus = 1;
	} // ctrl released
	else if(keyboard.sCodeBuffer[0] == RELEASE){
		keyboard.ctrlStatus = 0;
	}
}

void debug(){
	char outputS[4]; // 3 digits + null terminator
	sprintf(outputS, "%03u", keyboard.sCodeBuffer[0]);
	writeTerminal("|");
	writeTerminal(outputS);
	writeTerminal("|");
}

void EXTI9_5_IRQHandler(void)
{
	GPIO_PinState data_state = HAL_GPIO_ReadPin(keyboard.dataPort, keyboard.dataNumber);
	// check if we are reading during this interrupt
	if(keyboard.reading){
		// do not allow external reading, new key being read.
		keyboard.readyToRead = 0;
		// check if end of string
		if(keyboard.count == 9) {
			// reset read values
			keyboard.count = 0;
			keyboard.reading = 0;
			// update the code buffer
			keyboard.sCodeBuffer[1] = keyboard.sCodeBuffer[0];
			keyboard.sCodeBuffer[0] = getSCode(keyboard.scanCode);
			// check stop bit(should be high)
			if(data_state != GPIO_PIN_SET){
				keyboard.sCodeBuffer[0] = 243;
			}
			else{
				// confirm odd parity
				if((keyboard.parity % 2) != 1){
					keyboard.sCodeBuffer[0] = 244;
				}
			}
			keyboard.parity = 0;
			// check if the new key is readable
			// is in the list (lazy and) then check if it has an action (not 0)
			if(keyboard.sCodeBuffer[0] < 104 && getAscii() != 0){
				// then make sure the key has changed (not just held / let go)
				if(keyboard.sCodeBuffer[0] != (keyboard.sCodeBuffer[1])){
					keyboard.readyToRead = 1;
				}
			}
			else{
				// new scancode not readable!
				keyboard.readyToRead = 0;
			}
			toggleKeys();
			if(keyboard.debug) debug();
		}
		// if not EOS
		else{
			// store either 1 or 0 in the received bits.
			if(data_state == GPIO_PIN_SET) keyboard.scanCode[keyboard.count] = 1;
			else keyboard.scanCode[keyboard.count] = 0;
			// add to parity count as needed & increment counter
			keyboard.parity += keyboard.scanCode[keyboard.count];
			keyboard.count++;
		}
	}
	else{
		// if not reading, check if the bit is a start bit (low) then start reading.
		if(data_state != GPIO_PIN_SET) keyboard.reading = 1;
	}
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
}


