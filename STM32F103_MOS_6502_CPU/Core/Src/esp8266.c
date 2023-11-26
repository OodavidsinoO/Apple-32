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
	uint8_t fullyRead = 0;
	buffer[0] = '\0';
	while(!fullyRead){
		fullyRead = 1;
		result = HAL_UART_Receive(&huart3, (uint8_t *)input, 1, timeOut);
		// found a packet, now fully read it.
		if(result==HAL_OK){
			fullyRead = 0;
			while(1){
				if(result==HAL_OK){
					buffer[index] = input[0];
					index++;
				}
				else{
					buffer[index] = '\0';
					break;
				}
				result = HAL_UART_Receive(&huart3, (uint8_t *)input, 1, 30);
			}
		}
	}
	// timed out.
	if(buffer[0] == '\0') return 0;
	return 1;
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

void printOutput(char* buffer){
	char tempBuff[2] = { 0x00 };
	tempBuff[1] = '\0';
	for(int i=0; buffer[i]; i++){
		if(buffer[i]=='\r'){
			tempBuff[0] = '\\';
			writeTerminalChar(tempBuff);
			tempBuff[0] = 'r';
			writeTerminalChar(tempBuff);
		}
		else if(buffer[i]=='\n'){
			tempBuff[0] = '\\';
			writeTerminalChar(tempBuff);
			tempBuff[0] = 'n';
			writeTerminalChar(tempBuff);
		}
		else {
			tempBuff[0] = buffer[i];
			writeTerminalChar(tempBuff);
		}
	}
	writelineTerminal("");
}

uint8_t initESP(uint8_t* ip, char* ssid, char* pswd){
	/*
		0 = success, 1 = timeout, 2 = no OK for AT,
		3 = SSID&Pswd don't match a network, 4 = no IP could be fetched
	*/
	char buffer[1024] = { 0x00 };
	uint32_t timeOut = 1000; // timeout value in ms

	//resetting the esp
	writelineTerminal("Resetting");
	sendAT("AT+RST\r\n");
	if(!getResponse(buffer, timeOut)) return 1;
	if(!getResponse(buffer, 4*timeOut)) return 1;

	// ACK check
	writelineTerminal("Sending ACK");
	sendAT("AT\r\n");
	if(!getResponse(buffer, timeOut)) return 1;
	if(strcmp(buffer, "AT\r\r\n\r\nOK\r\n")) return 2;

	// set to client mode
	writelineTerminal("Setting Mode");
	sendAT("AT+CWMODE=1\r\n");
	if(!getResponse(buffer, timeOut)) return 1;

	// connect to AP
	char packet[16+32+63]; // chars, ssid, pswd
	sprintf(packet, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pswd);
	writelineTerminal("Connecting To Network");
	sendAT(packet);
	if(!getResponse(buffer, timeOut)) return 1;

	// OK ack
	if(!getResponse(buffer, timeOut*10)) return 1;
	if(strstr(buffer, "FAIL")) return 3;
	// get IP address
	writelineTerminal("Fetching Own IP");
	sendAT("AT+CIFSR\r\n");
	if(!getResponse(buffer, timeOut)) return 1;
	char* ipStart = strstr(buffer, "STAIP");
	if(ipStart == NULL) return 4;
	extractIP(ipStart+7, ip);
	return 0;
}





















