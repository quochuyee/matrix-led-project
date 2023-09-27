#include "main.h"
#include <stdbool.h>


#define CS 	GPIO_PIN_8
#define CLK GPIO_PIN_6
#define DIN GPIO_PIN_1

/*
 * The Function that'll help us to write the row and also the setting of The 7219
 * @param byte the data of each row
 */
void write_byte (uint8_t byte)
{
	for (int i =0; i<8; i++) {
		HAL_GPIO_WritePin (GPIOA, CLK, 0);  			// Pull the clock pin low
		HAL_GPIO_WritePin (GPIOA, DIN, byte&0x80);  	// Write the MSB bit to the data pin
		byte = byte<<1;  								// Shift left
		HAL_GPIO_WritePin (GPIOA, CLK, 1);  			// Pull the clock pin HIGH
	}
}

/*
 * The Function that'll help us to write the row and also the setting of The MAX 7219
 * @param address the number of column
 * @param data the data of the row
 */
void write_max (uint8_t address, uint8_t data)
{
	HAL_GPIO_WritePin (GPIOA, CS, 0);  // Set the CS pin to LOW
	write_byte (address);
	write_byte (data);
	HAL_GPIO_WritePin (GPIOA, CS, 1);  // Set the CS pin to HIGH
}

/*
 * The Function that is used for initializing led Matrix
 */
void max_init(void)
{
	 write_max(0x09, 0x00);       //  No decoding
	 write_max(0x0a, 0x03);       //  Brightness intensity
	 write_max(0x0b, 0x07);       //  Scan limit = 8 LEDs
	 write_max(0x0c, 0x01);       //  Power down =0,normal mode = 1
	 write_max(0x0f, 0x00);       //  No test display
}

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

bool flag 	= false;
bool flag_2 = false;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);



int main(void)
{
	uint8_t MSG_OFF[40] = {'\0'};

	uint8_t A_letter[5][8] = {
	  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	  {0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00},
	  {0x00,0x00,0x3C,0x3C,0x3C,0x3C,0x00,0x00},
	  {0x00,0x7E,0x7E,0x7E,0x7E,0x7E,0x7E,0x00},
	  {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},
	};

	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM3_Init();

	// Initialize the Matrix LED
	max_init();

	// Transmit message to the monitor (turn off)
	sprintf(MSG_OFF, "The LED has been turned off !\r\n");
	HAL_UART_Transmit(&huart2, MSG_OFF, sizeof(MSG_OFF), 100);

	while (1)
	{
	  if ((flag == true) && (flag_2 == false)) {
		 // Zoom out animation
		 for (int i = 0; i < 5; i++) {
			 for (int j = 0; j < 8; j++) {
				 write_max(j+1, A_letter[i][j]);
			 }
			 HAL_Delay(500);
		 }
		 flag_2 = true;	 ///> Capture flag 2
	  } else if((flag == false) && (flag_2 == true)) {
		 // Zoom in animation
		 for (int i = 4; i >= 0; i--) {
			 for (int j = 0; j < 8; j++) {
				 write_max(j+1, A_letter[i][j]);
			 }
			 HAL_Delay(500);
		 }
		flag_2 = false;  ///> Release flag 2
	  } else if ((flag == false) && (flag_2 == false)) {
		  	// Completely turn off every pieces of the matrix LED
		  	for (int j = 0; j < 8; j++) {
				write_max(j+1, A_letter[0][j]);
			}
	  }
	}
}

/**
 * The Interrupt callback function to flip the value of the flag
 * when an EXTI7 is triggered
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_7) {
		// Start the timer3
		HAL_TIM_Base_Start_IT(&htim3);
	}
}

/**
 * The Timer interrupt callback function occurs when timer3 reach the predefined number
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	// Declare the message buffer
	uint8_t MSG_ON[40] = {'\0'};
	uint8_t MSG_OFF[40] = {'\0'};

	if (htim == &htim3) {
		// Stop the timer3
		HAL_TIM_Base_Stop(&htim3);

		// Check whether the pin is low
		if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)) {
			flag = !flag;
		}

		// Send the message to the monitor
		if (flag) {
		    // Transmit message to the monitor (turn on)
		    sprintf(MSG_ON, "The matrix LED has been turned on !\r\n");
		    HAL_UART_Transmit(&huart2, MSG_ON, sizeof(MSG_ON), 100);
		} else {
			// Transmit message to the monitor (turn off)
			sprintf(MSG_OFF, "The matrix LED has been turned off !\r\n");
			HAL_UART_Transmit(&huart2, MSG_OFF, sizeof(MSG_OFF), 100);
		}

	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin|GPIO_PIN_6|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 LD2_Pin PA6 PA8
                           PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD2_Pin|GPIO_PIN_6|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
