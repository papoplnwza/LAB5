/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//papoplnwza
uint8_t RxBuffer[5];
uint8_t TxBuffer[40];
uint8_t st = 0, Blink_st = 1;
uint8_t Button = 0, Button_L = 0;
uint8_t CLK_F = 0, B_F = 0, Hz_F = 0;
uint8_t text1[] = "\nMENU\r\n"
		"Please Select Mode :\r\n"
		"0 = LED Control\r\n"
		"1 = Button Status\r\n";
uint8_t text2[] = "\n\nLED Control Mode\r\n"
		"Please Select\r\n"
		"a = Speed up\r\n"
		"s = Speed down\r\n"
		"d = On/Off\r\n"
		"x = Back\r\n";
uint8_t text3[] = "\n\nButton Status Mode\r\n"
		"x = Back\r\n";
uint8_t text4[10];
uint32_t Hz = 1, Stamp = 500;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UARTInterruptConfig();
void Blink();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  UARTInterruptConfig();
  HAL_UART_Transmit_IT(&huart2, text1, strlen((char*)text1));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Blink();
	  Button = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	  if (Button != Button_L) {
	  			B_F = 1;
	  		} else {
	  			B_F = 0;
	  		}
	  Button_L = Button;
		switch (st) {
		case (0):
			if (CLK_F == 1) {
				if (RxBuffer[0] == 48) {
					HAL_UART_Transmit_IT(&huart2, text2, strlen((char*)text2));
					CLK_F = 0;
					st = 1;
				} else if (RxBuffer[0] == 49) {
					HAL_UART_Transmit_IT(&huart2, text3, strlen((char*)text3));
					CLK_F = 0;
					st = 2;
				} else {
					HAL_UART_Transmit(&huart2, "Error\r\n", 7,10);
					HAL_UART_Transmit_IT(&huart2, text1, strlen((char*)text1));
					CLK_F = 0;
					st = 0;
				}
			}
			break;
		case (1):
			if (CLK_F == 1) {
				if (RxBuffer[0] == 97) {
					CLK_F = 0;
					Hz += 1;
					HAL_UART_Transmit(&huart2, "Speed +1\r\n", 10,10);
					sprintf((char*)text4,"Hz = %d\r\n",Hz);
					HAL_UART_Transmit_IT(&huart2, text4, strlen((char*)text4));
					st = 1;
				} else if (RxBuffer[0] == 115) {
					CLK_F = 0;
					Hz -= 1;
					HAL_UART_Transmit(&huart2, "Speed -1\r\n", 10,10);
					sprintf((char*)text4,"Hz = %d\r\n",Hz);
					HAL_UART_Transmit_IT(&huart2, text4, strlen((char*)text4));
					st = 1;
				} else if (RxBuffer[0] == 100) {
					if (Blink_st == 1) {
						Blink_st = 0;
						HAL_UART_Transmit_IT(&huart2, "Off\r\n", 5);
					} else if (Blink_st == 0) {
						HAL_UART_Transmit_IT(&huart2, "On\r\n", 4);
						Blink_st = 1;
					}
					CLK_F = 0;
//					Blink_st = !Blink_st;
					st = 1;
				}else if (RxBuffer[0] == 120) {
					HAL_UART_Transmit_IT(&huart2, text1, strlen((char*)text1));
					CLK_F = 0;
					st = 0;
				}
				else {
					HAL_UART_Transmit(&huart2, "Error\r\n", 7,10);
					HAL_UART_Transmit_IT(&huart2, text2, strlen((char*)text2));
					CLK_F = 0;
					st = 1;
				}
			}
			break;
		case (2):
			if (CLK_F == 1) {
				if (RxBuffer[0] == 120) {
					HAL_UART_Transmit_IT(&huart2, text1, strlen((char*)text1));
					CLK_F = 0;
					st = 0;
				} else {
					HAL_UART_Transmit(&huart2, "Error\r\n", 7,10);
					HAL_UART_Transmit_IT(&huart2, text3, strlen((char*)text3));
					CLK_F = 0;
					st = 2;
				}
			}
			if (CLK_F == 0 && B_F == 1) {
				if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0){
					HAL_UART_Transmit(&huart2, "Press\r\nx = Back\r\n\n", 19,10);

				}
				if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1){
					HAL_UART_Transmit(&huart2, "Release\r\nx = Back\r\n\n", 21,10);
				}
			}
			break;
		}
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 230400;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Blink(){
	if (Blink_st == 1) {
		if (Hz < 1) {
			Hz_F = 1;
		} else {
			Stamp = 500 / Hz;
		}
		static uint32_t timestamp = 0;
		if ((HAL_GetTick() >= timestamp)) {
			if (Hz_F == 0) {
				timestamp = HAL_GetTick() + Stamp;
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			} else {
				timestamp = HAL_GetTick() + 1;
				Hz_F = 0;
			}
		}
	}
	if (Blink_st == 0){
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
	}
}
void UARTInterruptConfig(){
	HAL_UART_Receive_IT(&huart2, RxBuffer, 1);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2){
		CLK_F = 1;
		RxBuffer[10] = '\0';
		sprintf((char*)TxBuffer,"\nReceived : %s\r\n",RxBuffer);
		HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer),10);

		HAL_UART_Receive_IT(&huart2, RxBuffer, 1);
	}
}
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
