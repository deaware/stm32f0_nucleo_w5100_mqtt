/**
  ******************************************************************************
  * File Name          : debug.h
  * Description        : Debug function.
	* Author             : Burin Sapsiri <burin@deaware.com>
  ******************************************************************************
	*/

#ifndef _DEBUG_H
#define _DEBUG_H

#ifdef DEBUG
	#define DEBUG_INIT(...)      debug_init(__VA_ARGS__);
	#define DEBUG_PRINT(...)     printf(__VA_ARGS__); printf("\r\n"); debug_send();
	#define DEBUG_PRINT_RAW(...) printf(__VA_ARGS__); debug_send();
	#define ERR_PRINT(...)       printf("Error %s at %d : ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\r\n"); debug_send();
#else
	#define DEBUG_INIT(...)
	#define DEBUG_PRINT(...)   
	#define DEBUG_PRINT_RAW(...)
	#define ERR_PRINT(...)
#endif

#ifdef DEBUG
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_dma.h>
#include <stm32f0xx_hal_dma_ex.h>
#include <stm32f0xx_hal_uart.h>
#include <stdio.h>

void debug_send(void);
void debug_init(UART_HandleTypeDef *uart_hndl);

#endif

#endif
