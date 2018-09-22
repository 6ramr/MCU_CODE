
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "sensor.h"
#include "spi.h"
#include "helperFunctions.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int VerifySensorConfig(SPI_HandleTypeDef*, uint8_t[], int len);
int WriteVerifySensorRegister (SPI_HandleTypeDef*, uint8_t, uint8_t);
void ConfigureSensor(SPI_HandleTypeDef*,uint8_t[]);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	static int configurationCorrect=0;
	static float sensorOutputX=0;
	static float sensorOutputY=0;
	static float sensorOutputZ=0;
	static uint8_t abc[6]={0};
	static uint8_t abd[6]={0};

//	uint8_t sensorData[4]= {SENSOR_OUT_X_H, SENSOR_OUT_X_L,0,0};
//  uint8_t sensorData[4]= {SENSOR_WHO_AM_I, SENSOR_WHO_AM_I,0,0};
	uint8_t sensorData[6]={0};
	uint8_t transmitData[6];
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//#############################
//#  Delay necessary for clocks to spread through chip. Otherwise MX_CAN_Init() fails
//#############################
  HAL_Delay(1000);
  CanTxMsgTypeDef CanTxMsg;
  CanTxMsg.StdId = 0x00;
  CanTxMsg.ExtId = 0x00;
  CanTxMsg.IDE = CAN_ID_STD;
  CanTxMsg.RTR = CAN_RTR_DATA;
  CanTxMsg.DLC = 6;

  CanRxMsgTypeDef CanRxMsg;
  /*CanRxMsg.StdId = 0;
  CanRxMsg.ExtId = 0;
  CanRxMsg.IDE = CAN_ID_STD;
  CanRxMsg.RTR = CAN_RTR_DATA;
  CanRxMsg.DLC = 6;
  CanRxMsg.FMI = 0;
  CanRxMsg.FIFONumber = CAN_FIFO0;*/

  CAN_FilterConfTypeDef CanFilterConf;
  CanFilterConf.FilterIdHigh = 0x000;
  CanFilterConf.FilterIdLow = 0x000;
  CanFilterConf.FilterMaskIdHigh = 0x000;
  CanFilterConf.FilterMaskIdLow = 0x000;
  CanFilterConf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  CanFilterConf.BankNumber = 0;
  CanFilterConf.FilterNumber = 0;
  CanFilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
  CanFilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
  CanFilterConf.FilterActivation = ENABLE;
  CanFilterConf.BankNumber = 0;
  hcan.pTxMsg = &CanTxMsg;
  hcan.pRxMsg = &CanRxMsg;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  // CUBEMX-RESISTENT RECONFIG BEGIN
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  if(HAL_CAN_ConfigFilter(&hcan,&CanFilterConf) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }
    // END

  /* Sensor Config */
//  SPI_TxSensor(&hspi1, sensorConf, (uint16_t)LEN(sensorConf));
//  configurationCorrect=VerifySensorConfig(&hspi1, sensorConf, LEN(sensorConf));

//  for (int i=0; i<6;i++) abc[i] = WriteVerifySensorRegister(&hspi1,sensorConf[i*2],sensorConf[i*2+1]);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
  	  HAL_SPI_Transmit(&hspi1,&sensorConf[0],2,50);
  	while( hspi1.State == HAL_SPI_STATE_BUSY );
  	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
  	  //SPI_TxSensor(&hspi1, &sensorConf[2*i], 2);
//  	  HAL_Delay(1000);

  for (int i=1; i<7;i++){
//	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
//	  HAL_SPI_Transmit(&hspi1,&sensorConf[(2*i)],2,50);
//	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	  SPI_TxSensor(&hspi1, &sensorConf[2*i], 2);
//	  HAL_Delay(0);
//	  abd[i-1]=sensorConf[(2*i)] + 0x80;
//	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
//	  HAL_SPI_Transmit(&hspi1,&abd[i-1],1,50);
//	  HAL_SPI_Receive(&hspi1,&abd[i-1],1,50);
//	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	  abd[i-1] = SPI_RxSensor(&hspi1,sensorConf[2*i]);
}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);

//	sensorData[2]=SPI_RxSensor(&hspi1,sensorData[0]);
//	sensorData[3]=SPI_RxSensor(&hspi1,sensorData[1]);

	sensorData[0]=0xA8;

	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&sensorData[0],1,5);
	HAL_SPI_Receive(&hspi1,&sensorData[0],6,5);
	while( hspi1.State == HAL_SPI_STATE_BUSY );
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);

	for (int i=0; i<6; i++) {
		CanTxMsg.Data[i] = sensorData[i] ;
		sensorData[i]=0;
	}

	if(HAL_CAN_Transmit(&hcan,10) != HAL_OK)
	    {
	      /* CAN_Transmit_Error */
	      Error_Handler();
	    }

	if(HAL_CAN_GetState(&hcan) != HAL_CAN_STATE_READY)
	  {
	    return HAL_ERROR;
	  }

	if(HAL_CAN_Receive(&hcan,CAN_FIFO0,10) != HAL_OK)
	    {
	      /* CAN_Receive_Error */
	      Error_Handler();
	    }

	if(HAL_CAN_GetState(&hcan) != HAL_CAN_STATE_READY)
	  {
	    return HAL_ERROR;
	  }
	for (int i=0; i<6; i++) {
			sensorData[i] = CanRxMsg.Data[i] ;
	}
	sensorOutputX = ResolveAcceleration(sensorData[1],sensorData[0]);
	sensorOutputY = ResolveAcceleration(sensorData[3],sensorData[2]);
	sensorOutputZ = ResolveAcceleration(sensorData[5],sensorData[4]);


	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_SILENT_LOOPBACK;
  hcan.Init.SJW = CAN_SJW_2TQ;
  hcan.Init.BS1 = CAN_BS1_4TQ;
  hcan.Init.BS2 = CAN_BS2_2TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  HAL_Delay(100);
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Ch2_3_DMA2_Ch1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch2_3_DMA2_Ch1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch2_3_DMA2_Ch1_2_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int VerifySensorConfig(SPI_HandleTypeDef *hspi1, uint8_t sensorConf[], int len) {
	for (int i=0; i<len;i++) {
		if (!(i % 2)) {
			if (! (sensorConf[i+1] == SPI_RxSensor(hspi1,sensorConf[i]))) {
				return 0;
			}
		}
	}
	return 1;
}
int WriteVerifySensorRegister (SPI_HandleTypeDef *hspi1, uint8_t adress, uint8_t value) {
	uint8_t txData[2] = {adress, value};
	SPI_TxSensor(hspi1,txData,2);
	if (value == SPI_RxSensor(hspi1,adress)){
		return 1;
	} else {
		return 0;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
