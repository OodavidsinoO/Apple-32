/*
 * esp8266.h
 *
 *  Created on: Nov 23, 2023
 *      Author: pasha
 */

#ifndef INC_ESP8266_H_
#define INC_ESP8266_H_

uint8_t initESP(uint8_t* ip, char* ssid, char* pswd);
uint32_t getResponse(char* buffer, uint32_t timeOut, uint32_t bounce);
uint8_t extractIP(char* buffer, uint8_t* ip);
uint8_t sendAT(char* packet);

#endif /* INC_ESP8266_H_ */
