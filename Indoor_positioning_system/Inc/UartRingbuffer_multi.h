/*
library name:  UartRingbuffer.h 	
written by: 	 Park	 
Date Written:  10-Jul-2019	by Controllerstech 
Last Modified: 16 August 2020 by Park	
Description: 	 Indoor Positioning System library.	 
References:    
               - Controllerstech : https://controllerstech.com/esp8266-webserver-using-stm32-hal/ 		

*/

#ifndef UARTRINGBUFFER_H_
#define UARTRINGBUFFER_H_


#include "stm32f4xx_hal.h"
#include <string.h>


/* change the size of the buffer */
#define UART_BUFFER_SIZE 64

typedef struct
{
  unsigned char buffer[UART_BUFFER_SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;
} ring_buffer;


/* writes the data to the tx_buffer and increment the head count in tx_buffer */
void Uart_write(int c, UART_HandleTypeDef *uart);

/* function to send the string to the uart */
void Uart_sendstring(const char *s, UART_HandleTypeDef *uart);

/* Initialize the ring buffer */
void Ringbuf_init(void);

/* the ISR for the uart. put it in the IRQ handler */
void Uart_isr (UART_HandleTypeDef *huart);

/* Callback function for Receiving Server Data */
void Uart_isr_RxCpltCallback (UART_HandleTypeDef *huart, char data); 

#endif /* UARTRINGBUFFER_H_ */
