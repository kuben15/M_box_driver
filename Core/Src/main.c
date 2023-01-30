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
#include <stdbool.h>
#include "tmag5170.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ENCODER_CS ((uint8_t) 0x00)
#define ENCODER_CP ((uint8_t) 0x01)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t data[6];
//uint32_t ilosc_krokow = 1600;

char motorCS[2] = {'C', 'S'};
char motorCP[2] = {'C', 'P'};


float AbsoluteAngleCS = 0;
float AbsoluteAngleCP = 0;


bool isInterrupt = false;
bool wasInterrupt = false;

uint8_t directionOfRotation = 0;
char kat_obrotu[3]; 

char motor[2];

float FirstAngle = 0.0;
float SecondAngle = 0.0;
float AngleDifference = 0.0;

uint8_t WhichRotation = 0;

uint8_t IsInRange = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void PWM_steering_period(float kroki_funkcja)
{
  //================================================DISCLAIMER================================================
  // tą funkcje bym przerobił w taki sposob, zeby nie mierzyla czasu tylko liczyla konkretne okresy PWM
  // i po wykonaniu okreslonej wartosci okresow zatrzymuje timer
  //==========================================================================================================
  
  float okres = 0.0;
  float czas_delay = 0.0;
  //T = (PRS + 1)(ARR +1) / f        PRS- preskaler, ARR- period

  okres = ((float)htim1.Init.Period + 1) * ((float)htim1.Init.Prescaler + 1)/(float)16e6;
  //okres = 1/(float)1600;

  czas_delay = okres * kroki_funkcja * 1000;
  ///delay(czas_delay);
  HAL_Delay(czas_delay);
}

float angleToSteps(uint16_t angle)
{ 
  //----------------------
  // tutaj zwracana wartość powinna być całkowita
  //----------------------
  
  float steps = 0;
 
  // TMC2209 is set to 8 microsteps
  // motor has 1.8 degree for 1 step
  // for 360 degree:
  // (360/1.8)*8 = 1600 steps
  // 360/1600 = 0,225 =>
  // each microstep is 0,225 degree

  steps = (float)angle/0.225;

  return steps;
} 

void directionRead(uint8_t data[6])
{
  
  char direction = (char)data[2];
  
  // Left
  if(direction == 'L')
  {
    directionOfRotation = 0;
  }
  // Right
  else if ((char*)data[2] == 'R')
  {
    directionOfRotation = 1;
  }
}

void motorRotation(char whichMotor[2], uint8_t direction, uint16_t angle)
{

  uint16_t FirstAngle;
  uint16_t SecondAngle;

  
  if(strncmp(whichMotor, motorCS, 2) == 0)
  {
    HAL_GPIO_WritePin(DIR_TMC1_GPIO_Port, DIR_TMC1_Pin, direction);
    
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2500);
    
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    PWM_steering_period(angleToSteps(angle));

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);

  }
  else if (strncmp(whichMotor, motorCP, 2) == 0)
  {
    if (direction == 0)
    direction = 1;
    else
    direction = 0;

    HAL_GPIO_WritePin(DIR_TMC2_GPIO_Port, DIR_TMC2_Pin, direction);
    
    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_1, 2500);
    
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_1);
  
    PWM_steering_period(angleToSteps(angle));

    HAL_TIM_PWM_Stop(&htim20, TIM_CHANNEL_1);
  }
  
}

float CheckAngle(float firstAngle, float secondAngle, uint16_t Angle, uint8_t direction)
{
  float angleDifference = 0;
  if (direction == 0)
  {
    angleDifference = secondAngle - firstAngle;
  }
  else if (direction == 1)
  {
    angleDifference = firstAngle - secondAngle;
  }

  HAL_Delay(50);
  return angleDifference;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t Data[40];
  uint16_t size = 0;

  isInterrupt = !isInterrupt;

  // dalsze nasluchiwanie UART
  HAL_UART_Receive_IT(&huart2, data, 6);
}

void HallSensorSettings(void)
{
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

  DisableCRC();
  TMAG5170_dev_conf_settings(&DevSettings);
  TMAG5170_sens_conf_settings(&SensSettings);
  TMAG5170_sys_conf_settings(&SystemSettings);
  TMAG5170_al_conf_settings(&AlertSettings);

  
}

void DisplayAngle(void)
{
  float Angle;
  uint16_t AngleInt = 0;
  uint16_t AngleInt100 = 0;

  Angle = Angle + GetAngle();

  
  AngleInt = (uint16_t)Angle;

  Angle = Angle*100 - AngleInt*100;

  AngleInt100 = (uint16_t)Angle;
  
  HAL_UART_Transmit(&huart2, "Angle: ", strlen("Angle: "), HAL_MAX_DELAY);
  char str1[8];
  sprintf(str1, "%d", AngleInt);
  HAL_UART_Transmit(&huart2, str1, strlen(str1), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, ",", strlen(","), HAL_MAX_DELAY);
  char str2[8];
  sprintf(str2, "%d", AngleInt100);
  HAL_UART_Transmit(&huart2, str2, strlen(str2), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", strlen("\r\n"), HAL_MAX_DELAY);

}

void CheckAbsoluteAngle(uint16_t Angle, uint8_t Direction, char motorSel[2])
{
  uint16_t NewAngleCS = 0;
  uint16_t NewAngleCP = 0;
  
  
  // at the beginning absolute angle is set to 0 degree
  if(strncmp(motorSel, motorCS, 2) == 0)
  {
    if(Direction == 0)
    {
      if(AbsoluteAngleCS + Angle < 3600)
      {
        AbsoluteAngleCS = AbsoluteAngleCS + Angle;
        IsInRange = 1;
      }
      else
      IsInRange = 0;
    }
    else
    {
      if(AbsoluteAngleCS - Angle > 0)
      {
        AbsoluteAngleCS = AbsoluteAngleCS - Angle;
        IsInRange = 1;
      }
      else
      IsInRange = 0;
    }
  }
  else if(strncmp(motorSel, motorCP, 2) == 0)
  {
    
    if(Direction == 0)
    {
      if(AbsoluteAngleCP + Angle < 3600)
      {
        AbsoluteAngleCP = AbsoluteAngleCP + Angle;
        IsInRange = 1;
      }
      else
      IsInRange = 0;
    }
    else
    {
      if(AbsoluteAngleCP - Angle > 0)
      {
        AbsoluteAngleCP = AbsoluteAngleCP - Angle;
        IsInRange = 1;
      }
      else
        IsInRange = 0;
    }
  }
}


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
  MX_TIM20_Init();
  /* USER CODE BEGIN 2 */

  // TMAG5170 configuration
  HallSensorSettings();


  HAL_UART_Receive_IT(&huart2, data, 6);
  HAL_UART_Transmit(&huart2, (uint8_t*)"sterownik silnikami\r\n", strlen("sterownik silnikami\r\n"), 200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  
  if(isInterrupt != wasInterrupt)
  {

    
    wasInterrupt = !wasInterrupt;
    directionRead(data);

    
    // save to "silnik" from data 'CS' or 'CP'
    strncpy(motor, data, 2);
    // save to "kat_obrotu" from data 
    strncpy(kat_obrotu, data + 3, 3);
    uint16_t kat_obrotu_zmienna = (uint16_t)atoi(kat_obrotu);

    CheckAbsoluteAngle(kat_obrotu_zmienna, directionOfRotation, motor);

    if(IsInRange == 1)
      motorRotation(motor, directionOfRotation, (uint16_t)kat_obrotu_zmienna);
    else
      HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nwrong angle!!!\r\nrange has been exceeded\r\n", strlen("\r\nwrong angle!!!\r\nrange has been exceeded\r\n"), 200);
    
    
    
    // AngleDifference = CheckAngle(FirstAngle, SecondAngle, kat_obrotu_zmienna, directionOfRotation);

    // uint16_t kat_dupa = 0;
    // kat_dupa = (uint16_t)AngleDifference;
    
    // char str1[8];
    // sprintf(str1, "%d", kat_dupa);
    // HAL_UART_Transmit(&huart2, str1, strlen(str1), HAL_MAX_DELAY);


  }

    //DisplayAngle();
    //HAL_Delay(1000);


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
