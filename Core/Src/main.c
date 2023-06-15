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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../App/inc/servo_driver.h"
#include "../../App/inc/MCP3008.h"
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

/* USER CODE BEGIN PV */
volatile uint16_t g_adcVal[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
MCP3008_SPI spi_mpc3008;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
//  HAL_GPIO_Init(TEMP_GPIO_POTENTIOMETER_GPIO_Port, )
//  HAL_GPIO_WritePin(TEMP_GPIO_POTENTIOMETER_GPIO_Port, TEMP_GPIO_POTENTIOMETER_Pin, 1);
//  HAL_Delay(2000);
  Servo servo;
  GRP_InitialiseServo(&servo, &SERVO_0_TIMER, SERVO_0_CHANNEL, 90);
  GRP_SetServo(&servo, 10);
  GRP_EnableServo(&servo);

//  HAL_ADC_Start_DMA(&hadc1, &g_adcVal, 3);

	MCP3008_Init(&spi_mpc3008, &hspi1, SPI_CS_GPIO_Port, SPI_CS_Pin);
	uint16_t adc0 = MCP3008_Read_Channel(&spi_mpc3008, 0); // Channel 0
	uint16_t adc1 = MCP3008_Read_Channel(&spi_mpc3008, 1); // Channel 1
	uint16_t adc2 = MCP3008_Read_Channel(&spi_mpc3008, 2); // Channel 2
	uint16_t adc3 = MCP3008_Read_Channel(&spi_mpc3008, 3); // Channel 3
	uint16_t adc4 = MCP3008_Read_Channel(&spi_mpc3008, 4); // Channel 4
	uint16_t adc5 = MCP3008_Read_Channel(&spi_mpc3008, 5); // Channel 5
	uint16_t adc6 = MCP3008_Read_Channel(&spi_mpc3008, 6); // Channel 6
	uint16_t adc7 = MCP3008_Read_Channel(&spi_mpc3008, 7); // Channel 7

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_GPIO_TogglePin(TEMP_GPIO_POTENTIOMETER_GPIO_Port, TEMP_GPIO_POTENTIOMETER_Pin);
	  if(servo.currentPosition <= 80){
		  servo.targetPosition = 110;
	  }
	  else if(servo.currentPosition >= 110){
		  servo.targetPosition = 80;
	  }
	  GRP_GoToPositionServo(&servo);
	  HAL_Delay(servo.stepTime);
//	  HAL_GPIO_WritePin(spi_mpc3008.CS_PORT, spi_mpc3008.CS_PIN, GPIO_PIN_RESET);
//	  HAL_Delay(500);
//	  HAL_GPIO_WritePin(spi_mpc3008.CS_PORT, spi_mpc3008.CS_PIN, GPIO_PIN_SET);
	  adc0 = MCP3008_Read_Channel(&spi_mpc3008, 0); // Channel 0
	  adc1 = MCP3008_Read_Channel(&spi_mpc3008, 1); // Channel 1
	  adc2 = MCP3008_Read_Channel(&spi_mpc3008, 2); // Channel 2
	  adc3 = MCP3008_Read_Channel(&spi_mpc3008, 3); // Channel 3
	  adc4 = MCP3008_Read_Channel(&spi_mpc3008, 4); // Channel 4
	  adc5 = MCP3008_Read_Channel(&spi_mpc3008, 5); // Channel 5
	  adc6 = MCP3008_Read_Channel(&spi_mpc3008, 6); // Channel 6
	  adc7 = MCP3008_Read_Channel(&spi_mpc3008, 7); // Channel 7
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
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
