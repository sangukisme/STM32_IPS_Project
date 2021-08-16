/*
library name:  ESP8266_HAL.c 	
written by: 	 Park	 
Date Written:  Apr 14, 2020	by Controllerstech 
Last Modified: 16 August 2020 by Park	
Description: 	 Indoor Positioning System library.	 
References:    
               - Controllerstech : https://controllerstech.com/esp8266-webserver-using-stm32-hal/ 		

*/

#include "ESP8266_HAL.h"

extern UART_HandleTypeDef *pc_uart;
extern UART_HandleTypeDef *wifi_uart;

int sema = 0;
// find_data    : Responses data
// find_complet : Finding Responses Completed
char find_data[40] = {0,};
volatile uint8_t find_complet = 0;

/*****************************************************************************************************************************************/

void ESP_Init (UART_HandleTypeDef *huart, char *SSID, char *PASSWD, int MODE)
{
	char data[80];
  
  // sema Lock	
	sema =1;
	
	Ringbuf_init();
	
	strcpy(find_data, "WIFI CONNECTED\r\n");
	Uart_sendstring("AT+RST\r\n", huart);
	while(!(find_complet));
	find_complet = 0;
	
	/********* AT **********/
  strcpy(find_data, "AT\r\r\n\r\nOK\r\n");
	Uart_sendstring("AT\r\n", huart);
	while(!(find_complet));
	find_complet = 0;
	
	if (MODE == MODE_STA)
	{
		/********* AT+CWMODE=1 (ESP:Client) **********/
		strcpy(find_data, "AT+CWMODE=1\r\r\n\r\nOK\r\n");
		Uart_sendstring("AT+CWMODE=1\r\n", huart);
		while (!(find_complet));
		find_complet = 0;

		/********* AT+CWJAP="SSID","PASSWD" (ESP:Client) **********/
		strcpy(find_data, "WIFI GOT IP\r\n\r\nOK\r\n");
		sprintf (data, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWD);
		Uart_sendstring(data, huart);
		while (!(find_complet));
		find_complet = 0;

		/********* AT+CIFSR (ESP:Client) **********/
		strcpy(find_data, "CIFSR:STAIP,\"");
		Uart_sendstring("AT+CIFSR\r\n", huart);
		while (!(find_complet));
		find_complet = 0;
		strcpy(find_data, "OK\r\n");
		while (!(find_complet));
		find_complet = 0;
	}
	
	else if (MODE == MODE_AP)
	{
	  /********* AT+CWMODE=3 (ESP:Host+Client) **********/
		strcpy(find_data, "AT+CWMODE=3\r\r\n\r\nOK\r\n");
		Uart_sendstring("AT+CWMODE=3\r\n", huart);
		while (!(find_complet));
		find_complet = 0;

	  /********* AT+CWSAP="SSID","PASSWD",CH,ECN (ESP:Host+Client) **********/ 
		strcpy(find_data, "AT+CWSAP=\"ESP-01\",\"1234test\",11,3\r\r\n\r\nOK\r\n");
		Uart_sendstring("AT+CWSAP=\"ESP-01\",\"1234test\",11,3\r\n", huart);
		while (!(find_complet));
		find_complet = 0;

	  /********* AT+CIFSR (ESP:Host+Client) **********/
	  strcpy(find_data, "CIFSR:APIP,\"");
		Uart_sendstring("AT+CIFSR\r\n", huart);
		while (!(find_complet));
		find_complet = 0;
		strcpy(find_data, "OK\r\n");
    while (!(find_complet));
		find_complet = 0;
	}
	
	strcpy(find_data, "AT+CIPMUX=1\r\r\n\r\nOK\r\n");
	Uart_sendstring("AT+CIPMUX=1\r\n", huart);
	while (!(find_complet));
	find_complet = 0;
	
	strcpy(find_data, "AT+CIPSERVER=1,80\r\r\n\r\nOK\r\n");
	Uart_sendstring("AT+CIPSERVER=1,80\r\n", huart);
	while (!(find_complet));
	find_complet = 0;
	
	// sema UnLock	
	sema = 0;
}

void ESP_wifi_start(uint8_t *flag, int x1, int y1, int x2, int y2)
{	
	static uint8_t start_flag = 0;
	static uint32_t dt = 0;
	static uint32_t timeLast = 0;
	uint8_t flag_state = *flag;
	*flag = 0;
	
	if ( flag_state == STATE_ERR )
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
		
		for (int i=0; i<5; i++)
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
			HAL_Delay(100);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			HAL_Delay(100);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
			HAL_Delay(100);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);			
			HAL_Delay(100);
		}
		
		ESP_Init(wifi_uart, "SK_WiFi815A","1501070647", MODE_STA);	 // MODE_STA : ESP Client mode, MODE_AP : ESP Host mode
		
		for (int i=0; i<5; i++)
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
			HAL_Delay(500);
		}
		
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
    
		start_flag = 0;	
	}
	else if ( flag_state == STATE_STOP )
	{
		Server_start_transmit(x1, y1, x2, y2);	
		
		start_flag = 0;	
	}
	else if ( flag_state == STATE_FORWARD || flag_state == STATE_BACKWARD || flag_state == STATE_ROTATION )
	{
		Server_start_transmit(x1, y1, x2, y2);		
		start_flag = 1;	
	}

	if ( start_flag )
	{
		if (timeLast != 0)
		{
			dt += HAL_GetTick() - timeLast;
		}
		timeLast = HAL_GetTick();
		
		if( dt > 200 ) 
		{
			Server_start_transmit(x1, y1, x2, y2);	
			dt = 0;
		}
	}
}


int Server_start_receive(char *LinkID, char *ServerData, char rxdata)
{
	char wifi_data = 0;
	char string[] = "+IPD,"; 	
	uint8_t len = strlen(string);
	
	static uint8_t flag_id = 0;
	static uint8_t flag_IPD = 0;
	static uint8_t flag_Rx = 0; 
	static uint8_t far = 0;
	
	if ( sema == 0 ) wifi_data = rxdata;
	
	if (flag_id)
	{
		*LinkID = wifi_data;
		flag_id = 0;
	}
	
	// ******** Look for a particular string ********
	// @ flag_IPD 1 : if the string is found 
	// @ far 1 : if part of the string is found
	// @ far 0 : if the string is found and not found 	
	if ( wifi_data == string[far] ) 
	{
		far++;
		if( far == len ) 
		{
			flag_id = 1;
			flag_IPD = 1;
			far = 0;
		}
	}
	else far = 0;
	// ******** Look for a particular character ********
	// The Required data is received next to the particular character.
  // @ flag_Rx 1 : if the character is found
	// @ return 1  : if all required data are found ( flag_Rx = 0, flag_IPD = 0 )
	if ( flag_IPD )
	{			
		if ( flag_Rx )
		{
			flag_Rx = 0;
			flag_IPD = 0;
			
			*ServerData	  = wifi_data;	 			
			return 1;
		}
		else
		{				
			if ( wifi_data == '=' ) flag_Rx = 1;										
		}					
	}
	
	return 0;
}

void Server_id_close (char LinkID)
{
	char data[80];
	
	LinkID -= 48;

	sprintf (data, "AT+CIPCLOSE=%d\r\n", LinkID);
	Uart_sendstring(data, wifi_uart);
}

void Server_control (char ServerData)
{
	if (ServerData == 'B')
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);	

	}
	else if (ServerData == 'F')
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
	}
	else if (ServerData == 'L')
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
	}
	else if (ServerData == 'R')
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
	}
	else if (ServerData == 'S')
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
	}
}

void Server_start_transmit (int x1, int y1, int x2, int y2)
{
	uint32_t tick = 0;	
	char data[80];
	char datatosend[80];
	
	// find_data    : Responses data
	// find_complet : Finding Responses Completed
	strcpy(find_data, "OK\r\n");
	sprintf (data, "AT+CIPCLOSE=5\r\n");
	Uart_sendstring(data, wifi_uart);
	tick = HAL_GetTick();
	while (!(find_complet))
	{
		if (HAL_GetTick() - tick > 10) break;                 // timeout = 50ms
	};
//	sprintf(data, "wait time:%d\r\n", HAL_GetTick()-tick);  // wait time max = 25ms
//	Uart_sendstring(data, pc_uart);
	find_complet = 0;
	
  // find_data    : Responses data
	// find_complet : Finding Responses Completed  	
	strcpy(find_data, "AT+CIPSTART=0,\"TCP\",\"192.168.35.242\",80\r\n0,CONNECT\r\n\r\nOK");
	sprintf (data, "AT+CIPSTART=0,\"TCP\",\"192.168.35.242\",80\r\n");
	Uart_sendstring(data, wifi_uart);
	tick = HAL_GetTick();
	while (!(find_complet))
	{
		if (HAL_GetTick() - tick > 50) break;                 // timeout = 50ms
	};
//	sprintf(data, "wait time:%d\r\n", HAL_GetTick()-tick);  // wait time max = 25ms
//	Uart_sendstring(data, pc_uart);
	find_complet = 0;
 
	sprintf (datatosend, "GET /insert_data2.php?num1=%d&num2=%d&num3=%d&num4=%d\r\n", x1, y1, x2, y2);
	Server_data_send(datatosend, 0);	
}

void Server_data_send (char *str, int Link_ID)
{
  uint32_t tick = 0;	
	int len = strlen(str);
	char data[80];
	
  // find_data    : Responses data
	// find_complet : Finding Responses Completed
	strcpy(find_data, ">");
	sprintf (data, "AT+CIPSEND=%d,%d\r\n", Link_ID, len);
	Uart_sendstring(data, wifi_uart);
	tick = HAL_GetTick();
	while (!(find_complet))
	{
		if (HAL_GetTick() - tick > 10) break;                 // timeout = 10ms
	};
//	sprintf(data, "wait time:%d\r\n", HAL_GetTick()-tick);  // wait time max = 5ms
//	Uart_sendstring(data, pc_uart);
	find_complet = 0;
	
  // find_data    : Responses data
	// find_complet : Finding Responses Completed
	strcpy(find_data, "0,CLOSED\r\n");
	Uart_sendstring (str, wifi_uart);
	tick = HAL_GetTick();
	while (!(find_complet))
	{
		if (HAL_GetTick() - tick > 50) break;                 // timeout = 50ms
	};
//	sprintf(data, "wait time:%d\r\n", HAL_GetTick()-tick);  // wait time max = 25ms
//	Uart_sendstring(data, pc_uart);
	find_complet = 0;
	
}	

int Server_data_wait (char rxdata)
{
	/*** Find Data ***/
	uint8_t len = strlen(find_data);
	static uint8_t far = 0;
	/*** Reset Condition ***/
	char string1[] = "busy p"; 	
	char string2[] = "SEND OK";
	uint8_t len1 = strlen(string1);
	uint8_t len2 = strlen(string2);
	static uint8_t far1 = 0;
  static uint8_t far2 = 0;
	static uint8_t flag_connect = 0;

	/******** Find Data ********/
	if( rxdata == find_data[far] )
	{
		far++;
		if( far == len )
		{
			far = 0;
			find_complet = 1;
		}
	}
	else far = 0;
	
	/******** Reset Condition ********/
	if ( !(sema) )
	{
		if( rxdata == string1[far1] )
		{
			far1++;
			if( far1 == len1 )
			{
				far1 = 0;
				flag_connect += 1;
			}
		}
		else far1 = 0;

		if ( flag_connect )
		{
			if ( flag_connect >=10) 
			{
				flag_connect = 0;
				return 1;
			}
			
			if( rxdata == string2[far2] )
			{
				far2++;
				if( far2 == len2 )
				{
					far2 = 0;
					flag_connect = 0;
				}
			}
			else far2 = 0;
		}
	}
	
	return 0;
}

