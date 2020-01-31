#include "main.h"
#include <math.h>

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);

void AX12_MovingSpeed(uint8_t ID, uint16_t Speed)
{
	uint8_t Data[9] = {0xFF, 0xFF, ID, 0x05, 0x03, 0x20, 0x00, 0x00, 0x00};
	uint8_t checksum = 0;

	Data[6] = Speed & 0xFF;
	Data[7] = (Speed >> 8) & 0xFF;

	for(uint8_t i = 2; i<8; i++)
	{
		checksum += Data[i];
	}

	checksum = ~checksum;

	Data[8] = checksum;

	while(huart1.gState != HAL_UART_STATE_READY);
	HAL_UART_Transmit(&huart1, Data, 9, 10);

}
void AX12_GoalPosition(uint8_t ID, uint32_t Position)
{
	uint8_t Data[9] = {0xFF, 0xFF, ID, 0x05, 0x03, 0x1E, 0x00, 0x00, 0x00};
	uint8_t checksum = 0;

	Data[6] = Position & 0xFF;
	Data[7] = (Position >> 8) & 0xFF;

	for(uint8_t i = 2; i<8; i++)
	{
		checksum += Data[i];
	}

	checksum = ~checksum;

	Data[8] = checksum;

	while(huart1.gState != HAL_UART_STATE_READY);
	HAL_UART_Transmit(&huart1, Data, 9, 10);
}
void AX12_ChangeID(uint8_t ID, uint8_t New_ID)
{
	uint8_t Data[8] = {0xFF, 0xFF, ID, 0x04, 0x03, 0x03, New_ID, 0x00};
	uint8_t checksum = 0;

	for(uint8_t i = 2; i<7; i++)
	{
		checksum += Data[i];
	}

	checksum = ~checksum;

	Data[7] = checksum;

	while(huart1.gState != HAL_UART_STATE_READY);
	HAL_UART_Transmit(&huart1, Data, 8, 10);
}
void AX12_Change_Baud_Rate(uint8_t ID, uint8_t Baud_Rate)
{
	uint8_t Data[8] = {0xFF, 0xFF, ID, 0x04, 0x03, 0x04, Baud_Rate, 0x00};
	uint8_t checksum = 0;

	for(uint8_t i = 2; i<7; i++)
	{
		checksum += Data[i];
	}

	checksum = ~checksum;

	Data[7] = checksum;

	while(huart1.gState != HAL_UART_STATE_READY);
	HAL_UART_Transmit(&huart1, Data, 8, 10);
}
void TFmini_Lidar_init()
{
	uint8_t Data1[8] = {0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x06};//reset
	while(huart2.gState != HAL_UART_STATE_READY);
	HAL_UART_Transmit(&huart2, Data1, 8, 10);

	HAL_Delay(1000);
}

extern uint8_t Lidar_Dataa[9];

int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();

  //uint8_t Lidar_Data[9];
  uint16_t Lidar_Distance = 0;
  uint16_t Dynamixel_degree = 205;
  uint16_t theta = 205;

  float pi = 3.14159265;
  float deg2rad = pi / 180.0;
  double azimuth = 0;
  double elevation = 0;

  char str[20] = {0,};
  int x = 0;
  int y = 0;
  int z = 0;

  TFmini_Lidar_init();
  HAL_Delay(1000);

  AX12_MovingSpeed(2, 128);

  AX12_MovingSpeed(1, 128);
  HAL_Delay(2000);

  AX12_GoalPosition(2, 205);

  AX12_GoalPosition(1, 205);
  HAL_Delay(2000);

  while(1)
  {
	  elevation = (theta - 205) * 0.29296875 * deg2rad;
	  AX12_GoalPosition(1, theta);

	  for(int i = 205; i<=819; i+=3){

		  while(Lidar_Dataa[0]!=0x59 || Lidar_Dataa[1]!=0x59);

		  //USART2->CR1 &= ~(0x20);
		  if(i>819) i=819;
		  Dynamixel_degree = i;

		  azimuth = (Dynamixel_degree - 205) * 0.29296875 * deg2rad;

		  AX12_GoalPosition(2, Dynamixel_degree);

		  HAL_Delay(100);

		  Lidar_Distance = (Lidar_Dataa[3]<<8) + (Lidar_Dataa[2] & 0xFF);

		  x = (int)Lidar_Distance * sin(azimuth) * cos(elevation);
		  y = (int)Lidar_Distance * cos(azimuth) * cos(elevation);
		  z = (int)Lidar_Distance * sin(elevation);

		  sprintf(str, "%d %d %d\n", -x, y, z);

		  HAL_UART_Transmit(&huart6, (uint8_t*)str, (uint16_t)sizeof(str), 10);
	  }

	  theta+=3;
	  if(theta>510) theta=510;
	  elevation = (theta - 205) * 0.29296875 * deg2rad;
	  AX12_GoalPosition(2, theta);

	  for(int i = 819; i>=205; i-=3){

		  while(Lidar_Dataa[0]!=0x59 || Lidar_Dataa[1]!=0x59);

		  if(i<205) i=205;
	  	  Dynamixel_degree = i;

	  	  azimuth = (Dynamixel_degree - 205) * 0.29296875 * deg2rad;

	  	  AX12_GoalPosition(2, Dynamixel_degree);

	  	  HAL_Delay(100);

	  	  Lidar_Distance = (Lidar_Dataa[3]<<8) + (Lidar_Dataa[2] & 0xFF);

	  	  x = (int)Lidar_Distance * sin(azimuth) * cos(elevation);
	  	  y = (int)Lidar_Distance * cos(azimuth) * cos(elevation);
	  	  z = (int)Lidar_Distance * sin(elevation);

	 	  sprintf(str, "%d %d %d\n", -x, y, z);

		  HAL_UART_Transmit(&huart6, (uint8_t*)str, (uint16_t)sizeof(str), 10);
	   }

	  theta+=3;
	  if(theta>510) theta=510;
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

  /**Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  USART2->CR1 |= 1<<5;
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
