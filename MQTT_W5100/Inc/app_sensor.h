/**
  ******************************************************************************
  * File Name          : app_sensor.h
  * Description        : 
	* Author             : Burin Sapsiri <burin@deaware.com>
  ******************************************************************************
	*/

#ifndef _APP_SENSOR_H
#define _APP_SENSOR_H

#include <stm32f0xx_hal.h>
#include "x_nucleo_iks01a1.h"
#include "x_nucleo_iks01a1_imu_6axes.h"
#include "x_nucleo_iks01a1_magneto.h"
#include "x_nucleo_iks01a1_pressure.h"
#include "x_nucleo_iks01a1_hum_temp.h"

void app_sensor_init(void);
float app_sensor_read_tmp(void);
float app_sensor_read_hum(void);
float app_sensor_read_psr(void);

#endif /*_APP_SENSOR_H*/
