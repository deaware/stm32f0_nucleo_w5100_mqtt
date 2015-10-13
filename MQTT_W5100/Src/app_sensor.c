/**
  ******************************************************************************
  * File Name          : app_sensor.c
  * Description        : 
	* Author             : Burin Sapsiri <burin@deaware.com>
  ******************************************************************************
	*/

#include "app_sensor.h"
#include "debug.h"

#define TMsg_MaxLen             256
/**
 * @brief  Serial message structure definition
 */
typedef struct
{
  uint32_t Len;
  uint8_t Data[TMsg_MaxLen];
} TMsg;

char dataOut[256];                       /*!< DataOut Frame */
volatile Axes_TypeDef ACC_Value;         /*!< Acceleration Value */
volatile Axes_TypeDef GYR_Value;         /*!< Gyroscope Value */
volatile Axes_TypeDef MAG_Value;         /*!< Magnetometer Value */
volatile float PRESSURE_Value;           /*!< Pressure Value */
volatile float HUMIDITY_Value;           /*!< Humidity Value */
volatile float TEMPERATURE_Value;        /*!< Temperature Value */

void app_sensor_init(void)
{
	/* Initialize IMU Sensor */
	/* Init function from x_nucleo_iks01a1_imu_6axes.c file */
	if(BSP_IMU_6AXES_Init() != IMU_6AXES_OK)
		while(1);
	
	/* Initialize Magnetometer Sensor */
	/* Init function from x_nucleo_iks01a1_magneto.c file */
	if(BSP_MAGNETO_Init() != MAGNETO_OK)
		while(1);
	
	/* Initialize Pressure Sensor */
	/* Init function from x_nucleo_iks01a1_pressure.c file */
	if (BSP_PRESSURE_Init() != PRESSURE_OK)
		while(1);
	
	/* Initialize Humidity Sensor */
	/* Init function from x_nucleo_iks01a1_hum_temp.c file */
	if(BSP_HUM_TEMP_Init() != HUM_TEMP_OK)
		while(1);
	
	DEBUG_PRINT("Inialize all sensor successful.")
}

float app_sensor_read_tmp(void)
{
  if(BSP_HUM_TEMP_isInitialized())
  {
		/* Get the Humidity sensor data */
		/* Get Humidity function from x_nucleo_iks01a1_hum_temp.c file */
    BSP_HUM_TEMP_GetHumidity((float *)&HUMIDITY_Value);
		/* Get the Temparture sensor data */
		/* Get Temperature function from x_nucleo_iks01a1_hum_temp.c file */
    BSP_HUM_TEMP_GetTemperature((float *)&TEMPERATURE_Value);
		
		return TEMPERATURE_Value;
  } else {
		return -127.00;
	}
}

float app_sensor_read_hum(void)
{
  if(BSP_HUM_TEMP_isInitialized())
  {
		/* Get the Humidity sensor data */
		/* Get Humidity function from x_nucleo_iks01a1_hum_temp.c file */
    BSP_HUM_TEMP_GetHumidity((float *)&HUMIDITY_Value);
		
		return HUMIDITY_Value;
  } else {
		return -127.00;
	}
}

float app_sensor_read_psr(void)
{
	 if(BSP_PRESSURE_isInitialized())
  {
		/* Get the pressure sensor data */
		/* Get Pressure function from x_nucleo_iks01a1_pressure.c file */
    BSP_PRESSURE_GetPressure((float *)&PRESSURE_Value);
    
		return PRESSURE_Value;
  } else {
		return -127.00;
	}
}
