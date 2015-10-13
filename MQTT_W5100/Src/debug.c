/**
  ******************************************************************************
  * File Name          : debug.c
  * Description        : Debug function.
	* Author             : Burin Sapsiri <burin@deaware.com>
  ******************************************************************************
	*/

#include "debug.h"

#ifdef DEBUG

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#define DEBUG_BUF_LEN          100
static char debug_buf[2][DEBUG_BUF_LEN];
static uint8_t send_page = 0;
static uint8_t buf_page = 1;
static uint16_t debug_buf_n = 0;
	
static UART_HandleTypeDef *debug_uart_hndl;
	
	
static void debug_insert(char c)
{
	debug_buf[buf_page][debug_buf_n++] = c;
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  
	debug_insert(ch);
  return ch;
}

void debug_send(void)
{
	/* Waiting for previous transmition complete. */
	while (debug_uart_hndl->hdmatx->State != HAL_DMA_STATE_READY) {
		
	}

	buf_page = send_page;
	
	if (send_page == 0) {
		send_page = 1;
	} else {
		send_page = 0;
	}

	HAL_UART_DMAStop(debug_uart_hndl);
	HAL_UART_Transmit_DMA(debug_uart_hndl, (uint8_t*)debug_buf[send_page], debug_buf_n);
	debug_buf_n = 0;
}

void debug_init(UART_HandleTypeDef *uart_hndl)
{
	debug_uart_hndl = uart_hndl;
}

#endif
