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
#include "math.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RMS_FRAME_LEN 100
typedef struct AdcRMS_t {
	uint8_t lastAdcIndex;
	uint16_t rawADC[RMS_FRAME_LEN];
	float adcRMS;
}AdcRMS_t;
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
MCP3008_SPI spi_mpc3008;
//volatile uint16_t adc[3] = { 0 };
uint16_t result[3];
uint16_t cnt = 0;
volatile AdcRMS_t g_adcRMS[3];
Servo servo;
bool g_updateAdcRMS = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float CalculateRMS(volatile AdcRMS_t *adc);
void ResetAdcRMSCalculation(volatile AdcRMS_t *adc);
void WriteNewAdcValue(volatile AdcRMS_t *adc, volatile uint16_t adc_val[1]);
void UpdateAdcRMS();
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
//  HAL_GPIO_Init(TEMP_GPIO_POTENTIOMETER_GPIO_Port, )
//  HAL_GPIO_WritePin(TEMP_GPIO_POTENTIOMETER_GPIO_Port, TEMP_GPIO_POTENTIOMETER_Pin, 1);
//  HAL_Delay(2000);
  SRV_Initialise(&servo, &SERVO_0_TIMER, SERVO_0_CHANNEL, 90);
  SRV_Set(&servo, 10);
  SRV_Enable(&servo);
  SRV_GoToPosition(&servo);
  for(int i = 0; i < 3; i++){
	  g_adcRMS[i].lastAdcIndex = 0;
	  ResetAdcRMSCalculation(&g_adcRMS[i]);
  }
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_ADC_Start_DMA(&hadc1, g_adcVal, 3);

//  for(int i = 0 ; i < 3; i++){
//	  HAL_ADC_Start(&hadc1);
//	  HAL_ADC_PollForConversion(&hadc1, 100);
//	  g_adcVal[i] = HAL_ADC_GetValue(&hadc1);
//  }

//  MCP_Init(&spi_mpc3008, &hspi1, SPI_CS_GPIO_Port, SPI_CS_Pin);
//  for (int i = 0; i < 8; i++) {
//    adc[i] = MCP_Read_Channel(&spi_mpc3008, i);
//  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    //	  HAL_GPIO_TogglePin(TEMP_GPIO_POTENTIOMETER_GPIO_Port, TEMP_GPIO_POTENTIOMETER_Pin);
//    if (servo.currentPosition <= 40) {
//      servo.targetPosition = 140;
//    }
//    else if (servo.currentPosition >= 140) {
//      servo.targetPosition = 40;
//    }

//    HAL_Delay(servo.stepTime);
    if(g_updateAdcRMS){
    	UpdateAdcRMS();
    }
//    result[1] -= 2122;

//    for(int i = 0 ; i < 3; i++){
//		  HAL_ADC_Start(&hadc1);
//    	  HAL_ADC_PollForConversion(&hadc1, 100);
//    	  g_adcVal = hadc1->Instance->DR;
//    	  g_adcVal[0] = HAL_ADC_GetValue(&hadc1);
//    	  g_adcVal[1] = HAL_ADC_GetValue(&hadc1);
//    	  g_adcVal[2] = HAL_ADC_GetValue(&hadc1);
//      }
    //	  HAL_GPIO_WritePin(spi_mpc3008.CS_PORT, spi_mpc3008.CS_PIN, GPIO_PIN_RESET);
    //	  HAL_Delay(500);
    //	  HAL_GPIO_WritePin(spi_mpc3008.CS_PORT, spi_mpc3008.CS_PIN, GPIO_PIN_SET);
//    for (int i = 0; i < 8; i++) {
//      adc[i] = MCP_Read_Channel(&spi_mpc3008, i);
//    }
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
float CalculateRMS(volatile AdcRMS_t *adc) {
	/**
	 * @brief Calculate Root Mean Square with moving frame value
	 * @retval Root Mean Square value
	 */

	uint32_t square = 0;
	float mean = 0.0;
//	float square_root = 0.0;

	for (uint8_t i = 0; i < RMS_FRAME_LEN; i++) {
		square += powf(adc->rawADC[i], 2);
	}

	mean = (square / (float) RMS_FRAME_LEN);
	adc->adcRMS = sqrtf(mean);
	return adc->adcRMS;
//	square_root = sqrtf(mean);
//	return square_root;
}

void ResetAdcRMSCalculation(volatile AdcRMS_t* adc) {
	/* initialize array with zeros */
	for (uint8_t i = 0; i < RMS_FRAME_LEN - 1; i++) {
		adc->rawADC[i] = 0;
	}
}

void WriteNewAdcValue(volatile AdcRMS_t *adc, volatile uint16_t *adc_val) {
	adc->rawADC[adc->lastAdcIndex] = *adc_val;
	if(adc->lastAdcIndex >= RMS_FRAME_LEN){
		adc->lastAdcIndex = 0;
	}
//	CalculateRMS(adc);
	adc->lastAdcIndex++;
}

void UpdateAdcRMS(){
	 for(int i = 0; i < 3; i++){
//		  WriteNewAdcValue(&g_adcRMS[i], &g_adcVal[i]);
		  CalculateRMS(&g_adcRMS[i]);
	  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim3 ) {
	  for(int i = 0; i < 3; i++){
	  		  WriteNewAdcValue(&g_adcRMS[i], &g_adcVal[i]);
	  	  }
	 g_updateAdcRMS = true;
  }
  if (htim == &htim14 ) {
//	  SRV_GoToPosition(&servo);
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
  while (1) {
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
