/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_ILI9341.h"
#include "TSC2046.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "math.h"
#include "sawtable.h"
#include "triangletable.h"
#include "sinetable.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define radian_per_degree 0.0174532925
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi5;

/* USER CODE BEGIN PV */
uint8_t DataSelector();
uint8_t FrequencySelector();
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI5_Init(void);
static void MX_DAC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TS_TOUCH_DATA_Def myTS_Handle;
float SineRegion = 60;
float TriangleRegion = 120;
float RampRegion = 180;
float SquareRegion = 240;
float FrequencyDivider = 10;
float FrequencyInterval = 32;
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
  MX_SPI2_Init();
  MX_SPI5_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */

HAL_DAC_Init(&hdac);
HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

uint8_t middley = 110;
uint8_t middlex = 100;
//uint8_t Q = 0;
//FrequencyInterval = 320/FrequencyDivider;

char welcome1[] = "Hello there.";
char welcome2[] = "Let's go ahead and";
char welcome3[] = "configure the screen so ";
char welcome4[] = "it's more accurate";
char welcome5[] = "Thank you.";
char welcome6[] = "Now, let's begin...";

  char wave1[] = "sine";
  char wave2[] = "triangle";
  char wave3[] = "saw";
  char wave4[] = "square";

  ILI9341_Init(&hspi5, LCD_CS_GPIO_Port, LCD_CS_Pin, LCD_DC_GPIO_Port, LCD_DC_Pin, LCD_RST_GPIO_Port, LCD_RST_Pin);
  ILI9341_setRotation(2);
  ILI9341_Fill(COLOR_NAVY);

  TSC2046_Begin(&hspi2, TS_CS_GPIO_Port, TS_CS_Pin);
  ILI9341_Fill(COLOR_RED);
  ILI9341_printText(welcome1, middlex, middley, COLOR_WHITE, COLOR_RED, 2);
  HAL_Delay(1000);
  ILI9341_Fill(COLOR_NAVY);
  ILI9341_printText(welcome2, middlex -75, middley - 20, COLOR_WHITE, COLOR_NAVY, 2);
  ILI9341_printText(welcome3, middlex -75, middley, COLOR_WHITE, COLOR_NAVY, 2);
  ILI9341_printText(welcome4, middlex -75, middley + 20, COLOR_WHITE, COLOR_NAVY, 2);

  TSC2046_Calibrate();
  ILI9341_Fill(COLOR_BLACK);

  ILI9341_printText(welcome5, middlex, middley -10, COLOR_WHITE, COLOR_BLACK, 2);
  HAL_Delay(1000);
  ILI9341_printText(welcome6, middlex - 50, middley +10, COLOR_WHITE, COLOR_BLACK, 2);

  void mainbg() {
  ILI9341_Fill_Rect(0, 0, 320, 60, COLOR_RED);
  ILI9341_Fill_Rect(0, 60, 320, 120, COLOR_ORANGE);
  ILI9341_Fill_Rect(0, 120, 320, 180, COLOR_PURPLE);
  ILI9341_Fill_Rect(0, 180, 320, 240, COLOR_BLUE);

  ILI9341_printText(wave1, 5, 20, COLOR_WHITE, COLOR_RED, 2);
  ILI9341_printText(wave2, 5, 80, COLOR_WHITE, COLOR_ORANGE, 2);
  ILI9341_printText(wave3, 5, 140, COLOR_WHITE, COLOR_PURPLE, 2);
  ILI9341_printText(wave4, 5, 200, COLOR_WHITE, COLOR_BLUE, 2);

  };

uint32_t degree = 0;
float wave_out = 0;
uint8_t testpos = 1;
uint8_t k = 0;

mainbg();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t P = DataSelector();
	  switch(P){
	  //square wave lol
	  case 4:
		  degree = 0;
		  wave_out = 0;
	  	  if (degree == testpos*360){
	  		  degree = 0;
	  	  }

	  	  for (degree = 0; degree < (testpos)*180; degree++) {
	  		  wave_out = 3700;
	  		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, wave_out);
	  	  } for (degree = (testpos)*180; degree < (testpos)*360; degree++){
	  		  wave_out = 0;
	  		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, wave_out);
	  	  }
	  	  break;

	  //saw wave lol
	  case 3:
		  degree = 0;
		  wave_out = 0;
		  for (degree = 0; degree <= 512; degree++){
			  wave_out = sawtable[degree];
	  		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, wave_out);
		  }
		  if(degree == 512){
		  	  		  		degree = 0;
		  	  		  	}

	  	  break;
	  case 2:
		  degree = 0;
		  wave_out = 0;
		  for (degree = 0; degree <= 512; degree++)
		  {
			  wave_out = triangletable[degree]-1850;
	  		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, wave_out);
		  }
		  if(degree == 512){
		  	  		  		degree = 0;
		  }
		  break;

	  case 1:
		  degree = 0;
		  wave_out = 0;
		  for (degree = 0; degree <= 512; degree++)
		  {
			  wave_out = sinetable[degree];
	  		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, wave_out);
		  }
		  if(degree == 512){
		  	  degree = 0;
		  }
		  break;

	  //triangle wave looolllll
	  }
	  myTS_Handle = TSC2046_GetTouchData();
	  if (myTS_Handle.isPressed) {
		 // ILI9341_fillCircle(myTS_Handle.X, myTS_Handle.Y, 2, COLOR_RED);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 15;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 15;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TS_CS_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TS_CS_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = TS_CS_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint8_t DataSelector()
{
  uint8_t j = 0;
  if(myTS_Handle.Y > 0.0 && myTS_Handle.Y <= SineRegion)
  {
    j = 1;
  }
  if(myTS_Handle.Y > SineRegion && myTS_Handle.Y <= TriangleRegion)
  {
    j = 2;
  }
  if(myTS_Handle.Y > TriangleRegion && myTS_Handle.Y <= RampRegion)
  {
    j = 3;
  }
  if(myTS_Handle.Y > RampRegion && myTS_Handle.Y <= SquareRegion)
  {
    j = 4;
  }
  return j;
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
