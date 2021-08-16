/*
library name:  ESP8266_HAL.h 	
written by: 	 Park	 
Date Written:  Apr 14, 2020	by Controllerstech 
Last Modified: 16 August 2020 by Park	
Description: 	 Indoor Positioning System library.	 
References:    
               - Controllerstech : https://controllerstech.com/esp8266-webserver-using-stm32-hal/ 		

*/

#ifndef INC_ESP8266_HAL_H_
#define INC_ESP8266_HAL_H_

#include "UartRingbuffer_multi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MODE_STA 1
#define MODE_AP  2

#define STATE_READY     0
#define STATE_FORWARD 	1
#define	STATE_BACKWARD 	2
#define	STATE_ROTATION 	3
#define STATE_LEFT 			4
#define STATE_RIGHT 		5
#define STATE_STOP 			6
#define	STATE_ERR 			7

void ESP_Init (UART_HandleTypeDef *huart, char *SSID, char *PASSWD, int MODE);

void ESP_wifi_start(uint8_t *flag, int x1, int y1, int x2, int y2);
	
// Wifi data Rx function for motor control
int Server_start_receive(char *LinkID, char *ServerData, char rxdata);
void Server_id_close (char LinkID);
void Server_control (char ServerData);

// Wifi data Tx function for position visualization
void Server_start_transmit (int x1, int y1, int x2, int y2);
void Server_data_send (char *str, int Link_ID);

// Check ESP transmit response data
int Server_data_wait (char rxdata);

#endif /* INC_ESP8266_HAL_H_ */
