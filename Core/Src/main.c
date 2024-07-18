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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ssd1306.h"
#include "fonts.h"
#include "test.h"
#include "bitmap.h"
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
I2C_HandleTypeDef hi2c3;
char buffer[10]={'P','A','V','A','N','\0'};
char symbol[10]={0};
uint16_t CO_2=10;
uint16_t PM_2_5=10;
uint16_t PM_10=10;
uint16_t Humidity=10;
uint16_t Temp=10;
uint16_t Light_intensity=10;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  //sprintf(buffer,"%d",CO_2);
  SSD1306_Init (); // initialize the display


  SSD1306_DrawBitmap(0,0,ganesha,128,64,1);
  SSD1306_UpdateScreen();
 HAL_Delay(2000);
 SSD1306_Clear ();

 SSD1306_DrawBitmap(0,0,Temperature,128,64,1);
 SSD1306_DrawBitmap(0,0,degree_centigrade,128,64,1);
 HAL_Delay(2000);
 SSD1306_GotoXY (70,10); // goto 10, 10
 SSD1306_Puts ("80", &Font_16x26, 1);
//
//  SSD1306_UpdateScreen();
//  HAL_Delay(2000);
//  SSD1306_Clear ();
//  SSD1306_DrawBitmap(0,0,humidity,128,64,1);
//  SSD1306_DrawBitmap(0,0,percentage,128,64,1);
//  SSD1306_GotoXY (50,30); // goto 10, 10
//  SSD1306_Puts ("40", &Font_16x26, 1);
//
//  SSD1306_UpdateScreen();
//  HAL_Delay(2000);
//  SSD1306_Clear ();
//
//  SSD1306_DrawBitmap(0,0,PM2,128,64,1);
//  SSD1306_DrawBitmap(0,0,micro_gram_per_cubic_meter,128,64,1);
//  SSD1306_GotoXY (82,10); // goto 10, 10
//  SSD1306_Puts ("50", &Font_16x26, 1);
//  SSD1306_GotoXY (120,30);
//  //SSD1306_Puts ("µg/m³", &Font_7x10, 1);
//  SSD1306_UpdateScreen();
//  HAL_Delay(2000);
//  SSD1306_Clear ();
//  SSD1306_DrawBitmap(0,0,PM10,128,64,1);
//  SSD1306_DrawBitmap(0,0,micro_gram_per_cubic_meter,128,64,1);
//  SSD1306_GotoXY (82,10); // goto 10, 10
//  SSD1306_Puts ("60", &Font_16x26, 1);
//  SSD1306_GotoXY (120,30);
//  //SSD1306_Puts ("µg/m³", &Font_7x10, 1);
//  SSD1306_UpdateScreen();
//  HAL_Delay(2000);
//  SSD1306_Clear ();
//  SSD1306_DrawBitmap(0,0,CO2,128,64,1);
//  SSD1306_DrawBitmap(0,0,ppm,128,64,1);
//  SSD1306_GotoXY (82,10); // goto 10, 10
//  SSD1306_Puts (buffer, &Font_16x26, 1);
//  SSD1306_GotoXY (120,30);
//  //SSD1306_Puts ("ppm", &Font_7x10, 1);
//  SSD1306_UpdateScreen();
//  HAL_Delay(2000);
//  SSD1306_Clear ();
//  SSD1306_DrawBitmap(0,0,Light_Intensity,128,64,1);
//  SSD1306_GotoXY (82,10); // goto 10, 10
//  SSD1306_Puts (buffer, &Font_16x26, 1);
//  SSD1306_GotoXY (120,30);
//  //SSD1306_Puts ("ppm", &Font_7x10, 1);
//  SSD1306_UpdateScreen();
//  HAL_Delay(2000);










  //SSD1306_DrawLine(64, 0, 64 , 64, 0x01);
  //SSD1306_DrawLine(0, 32, 128 , 32, 0x01);
  //SSD1306_DrawRectangle(0, 0, 128, 64, 0x01);
  //SSD1306_UpdateScreen();
  //HAL_Delay(2000);
  //SSD1306_GotoXY (5,5); // goto 10, 10
  //SSD1306_Puts ("CO2=0.22", &Font_7x10, 1);
  //SSD1306_GotoXY (5, 37);
  //SSD1306_Puts ("CO=0.34", &Font_7x10, 1);
  //SSD1306_GotoXY (75,5); // goto 10, 10
  //SSD1306_Puts ("CO2=0.22", &Font_7x10, 1);
  //SSD1306_GotoXY (75, 37);
  //SSD1306_Puts ("CO=0.34", &Font_7x10, 1); //Font_7x10 Font_11x18
  //SSD1306_UpdateScreen(); // update scree
  //SSD1306_Clear ();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x0010061A;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
