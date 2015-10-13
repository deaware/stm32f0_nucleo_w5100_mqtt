/**
  ******************************************************************************
  * File Name          : app.h
  * Description        : Foreground application function.
	* Author             : Burin Sapsiri <burin@deaware.com>
  ******************************************************************************
	*/

#ifndef _APP_H
#define _APP_H

#include <stm32f0xx.h>

#define APP_SOCKET_NO             0

void app_connected(void);
void app_disconnected(void);
void app_received(uint8_t *data, uint16_t len);
void app_sent(void);
void app_init(void);
void app_tick(void);

#endif /*_APP_H*/
