/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
	
/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main function and background process;
	* Author             : Burin Sapsiri <burin@deaware.com>
  ******************************************************************************
	*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#define IPADDR0              192
#define IPADDR1              168
#define IPADDR2               10
#define IPADDR3               50

#define NMASK0               255
#define NMASK1               255
#define NMASK2               255
#define NMASK3                 0

#define GW0                    0
#define GW1                    0
#define GW2                    0
#define GW3                    0


#include "wizchip_conf.h"
#include "app.h"
#include "debug.h"
#include "socket.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static const int8_t WZN_ERR = -1;
static const uint8_t wzn_memsize[2][4] = {{2, 2, 2, 2}, {2, 2, 2, 2}};
static wiz_NetInfo wzn_netif;
static uint8_t wzn_rx_buf[256];
static volatile uint8_t wzn_interrupted = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/********************************************
 * BEGIN
 * Low-level function for wiznet.
 ********************************************/
void wzn_select(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
}

void wzn_deselect(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

void wzn_write_byte(uint8_t wb)
{
	HAL_SPI_Transmit(&hspi1, &wb, 1, 0xffff);
}

uint8_t wzn_read_byte(void)
{
	uint8_t rd_byte = 0;

	HAL_SPI_Receive(&hspi1, &rd_byte, 1, 0xffff);
	
	return rd_byte;
}

void wzn_write(uint8_t* pBuf, uint16_t len)
{
	HAL_SPI_Transmit(&hspi1, pBuf, len, 0xffff);
}

void wzn_read(uint8_t* pBuf, uint16_t len)
{
	HAL_SPI_Receive(&hspi1, pBuf, len, 0xffff);
}

/********************************************
 * END
 * Low-level function for wiznet.
 ********************************************/

/* Wiznet low-level config. */
void wzn_config(void)
{
	uint16_t imr = 0;
	uint16_t imr_rd = 0;
	
	if(ctlwizchip(CW_INIT_WIZCHIP,(void*)wzn_memsize) == WZN_ERR) {
		ERR_PRINT("Initialize chip fail...\r\n");
		while (1);
  }
	
	HAL_Delay(1000);
	
	/* Set interrupt mask to S0, S1, S2, S3. */
	imr = IK_SOCK_0 | IK_SOCK_1 | IK_SOCK_2 | IK_SOCK_3;
	if (ctlwizchip(CW_SET_INTRMASK, &imr) == WZN_ERR) {
		ERR_PRINT("Cannot set imr...")
	}
	
	if (ctlwizchip(CW_GET_INTRMASK, &imr_rd) == WZN_ERR) {
		ERR_PRINT("Cannot set imr...")
	}
	
	if ((imr & imr_rd) != imr) {
		ERR_PRINT("IMR set unsuccessful...")
	}
}

void wzn_event_handle(void)
{
	uint16_t ir = 0;
	uint8_t sir = 0;
	uint16_t len = 0;
	
	if (ctlwizchip(CW_GET_INTERRUPT, &ir) == WZN_ERR) {
		ERR_PRINT("Cannot get ir...")
	}
	
	if (ir & IK_SOCK_0) {
		sir = getSn_IR(APP_SOCKET_NO);
		
		if ((sir & Sn_IR_CON) > 0) {
			/* Clear Sn_IR_CON flag. */
			setSn_IR(APP_SOCKET_NO, Sn_IR_CON);
			
			app_connected();
		}
		
		if ((sir & Sn_IR_DISCON) > 0) {
			/* Clear Sn_IR_DISCON flag. */
			setSn_IR(APP_SOCKET_NO, Sn_IR_DISCON);
			
			app_disconnected();
		}
		
		if ((sir & Sn_IR_SENDOK) > 0) {
			/* Clear Sn_IR_SENDOK flag. */
			setSn_IR(APP_SOCKET_NO, Sn_IR_SENDOK);
			
			app_sent();
		}
		
		if ((sir & Sn_IR_RECV) > 0) {
			len = getSn_RX_RSR(APP_SOCKET_NO);
			recv(APP_SOCKET_NO, wzn_rx_buf, len);
			
			/* Clear Sn_IR_RECV flag. */
			setSn_IR(APP_SOCKET_NO, Sn_IR_RECV);
			
			app_received(wzn_rx_buf, len);
		}
	}
}

/* Ethernet interface config. */
void netif_config(void)
{
	if (ctlnetwork(CN_GET_NETINFO, (void*)&wzn_netif) == WZN_ERR) {
		ERR_PRINT("Get initialized netif fail...\r\n");
		while (1);
	}
	
	wzn_netif.ip[0] = IPADDR0;
	wzn_netif.ip[1] = IPADDR1;
	wzn_netif.ip[2] = IPADDR2;
	wzn_netif.ip[3] = IPADDR3;
	
	wzn_netif.sn[0] = NMASK0;
	wzn_netif.sn[1] = NMASK1;
	wzn_netif.sn[2] = NMASK2;
	wzn_netif.sn[3] = NMASK3;
	
	wzn_netif.gw[0] = GW0;
	wzn_netif.gw[1] = GW1;
	wzn_netif.gw[2] = GW2;
	wzn_netif.gw[3] = GW3;
	
	if (ctlnetwork(CN_SET_NETINFO, (void*)&wzn_netif) == WZN_ERR) {
		ERR_PRINT("Set initialized netif fail...\r\n");
		while (1);
	}
	
	if (ctlnetwork(CN_GET_NETINFO, (void*)&wzn_netif) == WZN_ERR) {
		ERR_PRINT("Get initialized netif fail...\r\n");
		while (1);
	}

	DEBUG_PRINT("IP Address : %d.%d.%d.%d", wzn_netif.ip[0], wzn_netif.ip[1], wzn_netif.ip[2], wzn_netif.ip[3])
	DEBUG_PRINT("Netmask : %d.%d.%d.%d", wzn_netif.sn[0], wzn_netif.sn[1], wzn_netif.sn[2], wzn_netif.sn[3])
	DEBUG_PRINT("Gateway : %d.%d.%d.%d", wzn_netif.gw[0], wzn_netif.gw[1], wzn_netif.gw[2], wzn_netif.gw[3])
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	
	DEBUG_INIT(&huart2)
	DEBUG_PRINT("==========MQTT over ETH demonstration start==========")

	/* Set initial state of W5100_CS. */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	
	/* Register select & deselect function for wiznet. */
	reg_wizchip_cs_cbfunc(wzn_select, wzn_deselect);
	
	/* Register read & write function for wiznet. */
	reg_wizchip_spi_cbfunc(wzn_read_byte, wzn_write_byte);
	
	/* Register read brust * write brust funtion for wiznet. */
	reg_wizchip_spiburst_cbfunc(wzn_read, wzn_write);
	
	wzn_config();
	DEBUG_PRINT("Wiznet chip low-level is already config.")
	
	netif_config();
	DEBUG_PRINT("Ethernet interface already config.")
	
	app_init();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		app_tick();
		
		if (wzn_interrupted) {
			wzn_interrupted = 0;
			wzn_event_handle();
		}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00301D29;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLED;
  HAL_SPI_Init(&hspi1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_10) {
		wzn_interrupted = 1;
	}
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
