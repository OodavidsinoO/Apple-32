/*
 * esp8266.c
 *
 *  Created on: Nov 23, 2023
 *      Author: pasha
 */

#include "main.h"
#include "lcd.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

// will look for a uart packet for 'timeOut' time
// once found, will read the whole packet in.
// if times out, returns 1.
uint8_t getResponse(char* buffer, uint32_t timeOut){
	uint8_t index = 0;
	HAL_StatusTypeDef result;
	char input[2] = { 0x00 };
	result = HAL_UART_Receive(&huart3, (uint8_t *)input, 1, timeOut);
	// found a packet, now fully read it.
	if(result==HAL_OK){
		while(1){
			if(result==HAL_OK){
				buffer[index] = input[0];
				index++;
			}
			else{
				buffer[index] = '\0';
				return 1;
			}
			result = HAL_UART_Receive(&huart3, (uint8_t *)input, 1, 20);
		}
	}
	// timed out.
	return 0;
}

uint8_t extractIP(char* buffer, uint8_t* ip){
	char stringIp[16] = { 0 };
	for(int i=0; buffer[i]!='\"'; i++){
		stringIp[i] = buffer[i];
	}
	char* ptr = stringIp;
	for(int i=0; i<4; i++){
	    ip[i] = (uint8_t)strtol(ptr, &ptr, 10);
	    ptr++;
	}
	return 0;
}

uint8_t sendAT(char* packet){
	HAL_StatusTypeDef state = HAL_OK;
	state = HAL_UART_Transmit(&huart3, (uint8_t *)packet, strlen(packet), HAL_MAX_DELAY);
	if(state == HAL_OK) return 0;
	else return 1;
}

uint8_t initESP(uint8_t* ip, char* ssid, char* pswd){
	/*
		0 = success, 1 = timeout, 2 = no OK for AT,
		3 = SSID&Pswd don't match a network, 4 = no IP could be fetched
	*/
	char buffer[1024] = { 0x00 };
	uint32_t timeOut = 5000; // timeout value in ms

	// ACK check
	sendAT("AT\r\n");
	if(!getResponse(buffer, timeOut)) return 1;
	if(strcmp(buffer, "AT\r\r\n\r\nOK\r\n")) return 2;

	sendAT("AT+CWMODE=1\r\n");
	if(!getResponse(buffer, timeOut)) return 1;

	// connect to AP
	char packet[16+32+63]; // chars, ssid, pswd
	sprintf(packet, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pswd);
	sendAT(packet);
	if(!getResponse(buffer, timeOut)) return 1;
	// OK ack
	if(!getResponse(buffer, timeOut*4)) return 1;
	if(strstr(buffer, "FAIL")) return 3;
	// get IP address
	sendAT("AT+CIFSR\r\n");
	if(!getResponse(buffer, timeOut)) return 1;
	char* ipStart = strstr(buffer, "STAIP");
	if(ipStart == NULL) return 4;
	extractIP(ipStart+7, ip);
	return 0;
}





















