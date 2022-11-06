/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "TMAG5170.h"

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

uint16_t count = 2000;
uint8_t data = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(data == 49)
  {
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2500);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  }
  else
  {
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  }
  HAL_UART_Receive_IT(&huart2, &data, 1);
}


// void HallSensorSettings(void)
// {
//   TMAG5170_dev_conf_settings(TMAG5170_DEVICE_CONFIG_settings_t *new_settings);

  
// }

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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  DisableCRC();
  //uint8_t RxData;

  //TMAG5170_return_code_t ret = TMAG5170_RET_OK;


  TMAG5170_DEVICE_CONFIG_settings_t DevSettings;
  DevSettings.ConvAvg = TMAG5170_CONV_AVG_1x;
  DevSettings.MagTempCoef = TMAG5170_MAG_TEMPCO_0;
  DevSettings.OperatingMode = TMAG5170_OPERATING_MODE_ConfigurationMode;
  DevSettings.TempChannelEnabled = TMAG5170_T_CH_EN_TempChannelEnabled; 
  DevSettings.TempRate = TMAG5170_T_RATE_sameAsOtherSensors;
  DevSettings.TempChannelEnabled = TMAG5170_T_HLT_EN_tempLimitCheckOff;

  

  TMAG5170_SENSOR_CONFIG_settings_t SensSettings;
  SensSettings.AngleEnabled = TMAG5170_ANGLE_EN_X_Y;
  SensSettings.SleepTime = TMAG5170_SLEEPTIME_1ms;
  SensSettings.MagChannelEnabled = TMAG5170_MAG_CH_EN_XY;
  SensSettings.Zrange = TMAG5170_Z_RANGE_50mT;
  SensSettings.Yrange = TMAG5170_Y_RANGE_50mT;
  SensSettings.Xrange = TMAG5170_X_RANGE_50mT;


  TMAG5170_SYSTEM_CONFIG_settings_t SystemSettings;
  SystemSettings.DiagnosticSelect = TMAG5170_DIAG_SEL_AllDataPathTogether;
  SystemSettings.TriggerMode = TMAG5170_TRIGGER_MODE_nCSpulse;
  SystemSettings.DataType = TMAG5170_DATA_TYPE_32bit;
  SystemSettings.DiagnosticEnabled = TMAG5170_DIAG_EN_AFEdiagnosticsDisabled;
  SystemSettings.ZlimitCheck = TMAG5170_Z_HLT_EN_ZaxisLimitCheckOff;
  SystemSettings.YlimitCheck = TMAG5170_Y_HLT_EN_YaxisLimitCheckOff;
  SystemSettings.XlimitCheck = TMAG5170_X_HLT_EN_XaxisLimitCheckOff;


  TMAG5170_ALERT_CONFIG_settings_t AlertSettings;
  AlertSettings.AlertStatus = TMAG5170_STATUS_ALRT_IsAsserted;


  // TMAG5170 configuration
  // DisableCRC();
  // TMAG5170_dev_conf_settings(&DevSettings);
  // TMAG5170_sens_conf_settings(&SensSettings);
  // TMAG5170_sys_conf_settings(&SystemSettings);
  // TMAG5170_al_conf_settings(&AlertSettings);

  //uint8_t bity = 0x01;
  char kupa[7] = "kupa: ";
  // float temperature = 0.0;
  // uint8_t UART_buff;
  // uint16_t UART_buff_length;
  HAL_UART_Receive_IT(&huart2, &data, 1);
  HAL_UART_Transmit(&huart2, (uint8_t*)"sterownik silnikami\r\n", strlen("sterownik silnikami\r\n"), 200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    DisableCRC();
    TMAG5170_dev_conf_settings(&DevSettings);
    TMAG5170_sens_conf_settings(&SensSettings);
    TMAG5170_sys_conf_settings(&SystemSettings);
    TMAG5170_al_conf_settings(&AlertSettings);
    //ReadRegister();
    //ReadRegister1();
    
    //GetTemp();

    //HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);

    GetAngle();
    // temperature = GetTemp();
    // UART_buff_length = sprintf(UART_buff, "%f\r\n", temperature);
    // HAL_UART_Transmit(&huart2, UART_buff, UART_buff_length, HAL_MAX_DELAY );
    
    //HAL_UART_Transmit(&huart2, &asdkna, strlen(asdkna), HAL_MAX_DELAY);
    //HAL_UART_Transmit(&huart2, &Xvalue, strlen(Xvalue), HAL_MAX_DELAY);
    //HAL_UART_Transmit(&huart2, (uint8_t*)kupa, strlen(kupa), HAL_MAX_DELAY);

    //GetAngle();
    //HAL_UART_Transmit(&huart2, bit, strlen(bit), HAL_MAX_DELAY);
    //HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", strlen("\r\n"), HAL_MAX_DELAY);
    
    HAL_UART_Transmit(&huart2, (uint8_t*)" hal transmit\r\n", strlen(" hal transmit\r\n"), HAL_MAX_DELAY);
    HAL_Delay(1000);

    //HAL_UART_Transmit(&huart2, "\r\n", strlen("\r\n"), HAL_MAX_DELAY);
    
    
    HAL_Delay(100);


    // HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    // HAL_Delay(500);

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
