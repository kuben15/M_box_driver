/*
 *   TMAG51170.c
 *
 *   Created on: 21.10.2022
 *       Author: JS
 *
 */

#include "tmag5170.h"
#include <inttypes.h>
#include <string.h>
#include <spi.h>
#include <usart.h>
#include <stdio.h>
#include <stdlib.h>

static TMAG5170_DEVICE_CONFIG_settings_t DeviceConfigSettings;
// static TMAG5170_DEVICE_CONFIG_settings_t results;
static TMAG5170_reg_addr_t register_address;

static TMAG5170_SENSOR_CONFIG_settings_t SensorConfigSettings;

static TMAG5170_SYSTEM_CONFIG_settings_t SystemConfigSettings;

static TMAG5170_ALERT_CONFIG_settings_t AlertConfigSettings;






void TransmitData(uint8_t address, uint8_t dataTx[2], uint8_t crc)
{
  // zapisanie '0' na pierwszym miejscu aby dane były zapisane w sensorze
  address |= 0b0 << 7;
  
  // zmiana stanu pinow CS na 0 w celu zainicjiowania urzadzen, do obu sensorow sa wprowadzane te same dane
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 0);
  //HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, 0);

  // rozpoczynamy transmisje od adresu, nastepnie 2 bajty danych oraz crc
  HAL_SPI_Transmit(&hspi1, &address, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi1, dataTx, 2, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi1, &crc, 1, HAL_MAX_DELAY);
  
  // konczymy inicjalizacje po przez zmiane stanow pinow CS na 1
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);
  //HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, 1);

}



TMAG5170_return_code_t TMAG5170_dev_conf_settings(TMAG5170_DEVICE_CONFIG_settings_t *new_settings)
{
  
  uint8_t register_address = TMAG5170_DEVICE_CONFIG_REG;
  TMAG5170_return_code_t ret = TMAG5170_RET_OK;

  // copy bytes from settings to new_settings
  memcpy(&DeviceConfigSettings, new_settings, sizeof(DeviceConfigSettings));

  // saving the bytes to the appropriate places in the variable
  uint16_t register_data = ((uint16_t)(DeviceConfigSettings.ConvAvg) << 12u); // tak sie powinno robić, ten sam typ, przy przesunięciu gubi się znak+-
  register_data |= ((uint16_t)(DeviceConfigSettings.MagTempCoef) << 8u);
  register_data |= ((uint16_t)(DeviceConfigSettings.OperatingMode) << 4u);
  register_data |= ((uint16_t)(DeviceConfigSettings.TempChannelEnabled) << 3u);
  register_data |= ((uint16_t)(DeviceConfigSettings.TempRate) << 2u);
  register_data |= ((uint16_t)(DeviceConfigSettings.TempChannelEnabled) << 1u);

  uint8_t register_data_8[2];
  register_data_8[0] = register_data >> 8;
  register_data_8[1] = register_data;

  // last 8 bits:
  // 0b12345678
  // 3 and 4 bits are cmd status
  // 3 bit: CMD1
  // CMD1 = 0 --> Display SET_COUNT[2:0] in STAT[2:0] bits at SDO next frame
  // CMD1 = 1 --> Display DATA_TYPE[2:0] in STAT[2:0] bits at SDO next frame
  // 4 bit: CMD0
  // CMD0 = 0 --> no conversion start through command bits
  // CMD0 = 1 --> start of conversion at the CS going high
  // last 4 bits are CRC

  uint8_t StatusCRC = 0b00111100;

  TransmitData(register_address, register_data_8, StatusCRC);
  
  return ret;
}

TMAG5170_return_code_t TMAG5170_sens_conf_settings(TMAG5170_SENSOR_CONFIG_settings_t *new_settings)
{
  
  register_address = TMAG5170_SENSOR_CONFIG_REG;
  TMAG5170_return_code_t ret = TMAG5170_RET_OK;

  // copy bytes from settings to new_settings
  memcpy(&SensorConfigSettings, new_settings, sizeof(SensorConfigSettings));

  // saving the bytes to the appropriate places in the variable
  uint16_t register_data = (SensorConfigSettings.AngleEnabled << 14);
  register_data |= (SensorConfigSettings.SleepTime << 10);
  register_data |= (SensorConfigSettings.MagChannelEnabled << 6);
  register_data |= (SensorConfigSettings.Zrange << 4);
  register_data |= (SensorConfigSettings.Yrange << 2);
  register_data |= (SensorConfigSettings.Xrange << 0);

  uint8_t register_data_8[2];
  register_data_8[0] = register_data >> 8;
  register_data_8[1] = register_data;

  uint8_t StatusCRC = 0b00111100;
  TransmitData(register_address, register_data_8, StatusCRC);
  
  return ret;
}

TMAG5170_return_code_t TMAG5170_sys_conf_settings(TMAG5170_SYSTEM_CONFIG_settings_t *new_settings)
{
  
  register_address = TMAG5170_SYSTEM_CONFIG_REG;
  TMAG5170_return_code_t ret = TMAG5170_RET_OK;

  // copy bytes from settings to new_settings
  memcpy(&SystemConfigSettings, new_settings, sizeof(SystemConfigSettings));

  // saving the bytes to the appropriate places in the variable
  uint16_t register_data = (SystemConfigSettings.DiagnosticSelect << 12);
  register_data |= (SystemConfigSettings.TriggerMode << 9);
  register_data |= (SystemConfigSettings.DataType << 6);
  register_data |= (SystemConfigSettings.DiagnosticEnabled << 5);
  register_data |= (SystemConfigSettings.ZlimitCheck << 2);
  register_data |= (SystemConfigSettings.YlimitCheck << 1);
  register_data |= (SystemConfigSettings.XlimitCheck << 0);

  uint8_t register_data_8[2];
  register_data_8[0] = register_data >> 8;
  register_data_8[1] = register_data;
  
  uint8_t StatusCRC = 0b00111100;

  // wysłanie
  TransmitData(register_address, register_data_8, StatusCRC);
  
  return ret;
}

TMAG5170_return_code_t TMAG5170_al_conf_settings(TMAG5170_ALERT_CONFIG_settings_t *new_settings)
{
  
  register_address = TMAG5170_ALERT_CONFIG_REG;
  TMAG5170_return_code_t ret = TMAG5170_RET_OK;

  // copy bytes from settings to new_settings
  memcpy(&AlertConfigSettings, new_settings, sizeof(AlertConfigSettings));

  // saving the bytes to the appropriate places in the variable
  uint16_t register_data = (AlertConfigSettings.AlertLatch << 13);
  register_data |= (AlertConfigSettings.AlertMode << 12);
  register_data |= (AlertConfigSettings.AlertStatus << 11);
  register_data |= (AlertConfigSettings.AlertResponse << 8);
  register_data |= (AlertConfigSettings.ThresholdCount << 4);
  register_data |= (AlertConfigSettings.TempThrAlert << 3);
  register_data |= (AlertConfigSettings.ZthrAlert << 2);
  register_data |= (AlertConfigSettings.YthrAlert << 1);
  register_data |= (AlertConfigSettings.XthrAlert << 0);

  
  uint8_t register_data_8[2];
  register_data_8[0] = register_data >> 8;
  register_data_8[1] = register_data;
  
  
  uint8_t StatusCRC = 0b00111100;

  TransmitData(register_address, register_data_8, StatusCRC);
  
  return ret;
}

void DisableCRC(void)
{

  uint8_t DisCRC[4];
  DisCRC[0] = 0x0F;
  DisCRC[1] = 0x00;
  DisCRC[2] = 0x04;
  DisCRC[3] = 0x07;
  
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 0);
  HAL_SPI_Transmit(&hspi1, DisCRC, 4, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);
}

void ReadRegister(void)
{
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 0);


  //register_address = TMAG5170_X_CH_RESULT_REG;
  // switch (address)
  // {
  // case 1:
  //   register_address = TMAG5170_Z_THRX_CONFIG_REG;
  //   break;
  // case 2:
  //   register_address = TMAG5170_SYS_STATUS_REG;
  // default:
  //   break;
  // }
  
  register_address = TMAG5170_ANGLE_RESULT_REG;
  uint8_t FirstFrame = register_address;
  FirstFrame |= 0b1 << 7;

  //uint16_t zeros = 0x0000;
  uint8_t crc = 0b00110000;
  // wysłanie

  HAL_SPI_Transmit(&hspi1, &FirstFrame, 3, HAL_MAX_DELAY);
  //HAL_SPI_Transmit(&hspi1, &zeros, 2, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi1, &crc, 1, HAL_MAX_DELAY);
  //HAL_SPI_Receive(&hspi1, &RxData, 2, HAL_MAX_DELAY);

  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);

  //return RxData;
}

void ReadRegister1(void)
{
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 0);
  //register_address = TMAG5170_X_CH_RESULT_REG;
  // switch (address)
  // {
  // case 1:
  //   register_address = TMAG5170_Z_THRX_CONFIG_REG;
  //   break;
  // case 2:
  //   register_address = TMAG5170_SYS_STATUS_REG;
  // default:
  //   break;
  // }
  
  //uint16_t RxData;

  register_address = TMAG5170_X_CH_RESULT_REG;
  uint8_t FirstFrame = register_address;
  FirstFrame |= 0b1 << 7;

  //uint16_t zeros = 0x0000;
  uint8_t crc = 0b00110000;
  // wysłanie

  HAL_SPI_Transmit(&hspi1, &FirstFrame, 3, HAL_MAX_DELAY);
  //HAL_SPI_Transmit(&hspi1, &zeros, 2, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi1, &crc, 1, HAL_MAX_DELAY);
  //HAL_SPI_Receive(&hspi1, &RxData, 2, HAL_MAX_DELAY);


  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);
  //return RxData;
}

void GetmT(void)
{
  // Received data
  uint8_t RxData[2];
  uint16_t data_reg;

  // wartosc pola magnetycznego w mT
  float B = 0.0;

  // ustawiona czulosc sensora
  uint8_t Br = TMAG5170_X_RANGE_50mT;

  // sum of bits 0-14
  uint16_t sum;


  // Send address
  register_address = TMAG5170_X_CH_RESULT_REG;
  uint8_t FirstFrame = register_address;
  FirstFrame |= 0b1 << 7;

  // CS LOW
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 0);
  // Transmit address
  HAL_SPI_Transmit(&hspi1, &FirstFrame, 1, HAL_MAX_DELAY);
  // Receive data
  HAL_SPI_Receive(&hspi1, RxData, 2, HAL_MAX_DELAY);
  // CS HIGH
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);

  // uint8_t to uint16_t
  data_reg = RxData[0];
  data_reg = data_reg << 8;
  data_reg |= RxData[1];

  // read MSB
  uint16_t and = data_reg;
  uint16_t D15 = 0x8000 & and;

  // counting sum of bits 0-14
  for(int i=0; i<=14; i++)
  {
      sum += data_reg & (0b1 << i);
  }

  // put into the formula
  B = (((float)D15 + (float)sum)/65536)*2*50;

  // calculate numbers after dot
  uint8_t t;
  t = (B - (int8_t)B)*100;

  char str1[8];
  char str2[8];

  // converting to char
  sprintf(str1, "%d", (int8_t)B);
  sprintf(str2, "%d", t); 

  // sending data to console
  HAL_UART_Transmit(&huart2, str1, strlen(str1), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, ".", strlen("."), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, str2, strlen(str2), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, " mT", strlen(" mT"), HAL_MAX_DELAY);
}

uint16_t ReadData(uint8_t which_motor, uint8_t address)
{
  
  uint16_t dataRx = 0;
  

  // decydujemy ktory czujnik bedzie aktywny(aktywacja nastepuje stanem niskim)
  switch (which_motor)
  {
  case 1:
    HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 0);
    HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, 1);
    HAL_Delay(50);
    break;
  
  case 2:
    HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, 0);
    HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);
    break;
  default:
    break;
  }

  //wysylamy adres 
  HAL_SPI_Transmit(&hspi1, address, 1, HAL_MAX_DELAY);
  // odbieramy 2 ramki danych
  HAL_SPI_Receive(&hspi1, dataRx, 2, HAL_MAX_DELAY);



  //po skonczonej transmisji  dezaktywujemy urzadzenia
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);
  HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, 1);

  return dataRx;
}


// void GetAngle(uint8_t which_motor)
// {

//   uint8_t address = 0;
//   uint16_t raw_angle = 0;

//   // wysylamy na poczatek ramki czy chcemy odczytac czy zapisac oraz adres rejestru
//   // 1 dla odczytu
//   // 0 dla zapisu
//   address = TMAG5170_ANGLE_RESULT_REG;
//   address = address | 0b1 << 7;


//   raw_angle = ReadData(which_motor, address);

// }



float GetAngle(void)
{
  
  uint8_t data[2] = { 0 };
  uint16_t data_reg = 0;
  uint16_t data_reg_LSB = 0;
  uint16_t data_reg_MSB = 0;

  
  // Angle in degree
  float Angle = 0.0;
  

  //wysyłamy adres
  register_address = TMAG5170_ANGLE_RESULT_REG;
  // register_address = TMAG5170_ANGLE_RESULT_REG;
  uint8_t FirstFrame = register_address;
  FirstFrame |= 0b1 << 7;


  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 0);
  HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, 1);
  HAL_SPI_Transmit(&hspi1, &FirstFrame, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, data, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);
  HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, 1);
    
  data_reg = data[0];
  data_reg = data_reg << 8;
  data_reg |= data[1];


  data_reg_LSB = data[1];
  //zostawimy tylko ostatnie 4 bity
  data_reg_LSB = data_reg_LSB & 0x0F;
  // LSB = (float)data_reg_LSB;
  // LSB = LSB/8;
  data_reg_LSB = data_reg_LSB * 100;
  data_reg_LSB = data_reg_LSB/16;
  


  data_reg_MSB = data_reg & 0x1FF0;
  data_reg_MSB = data_reg >> 4;

  Angle = data_reg_MSB + data_reg_LSB/(float)100;

  // MSB = (float)data_reg_MSB;
  // Angle_uint = (data_reg_MSB * 1000) + data_reg_LSB;
  // Angle = Angle_uint/1000;

  // Angle = MSB + LSB;

  return Angle;
}


float GetTemp()
{
  float temp = 0.0;
  uint8_t buff[2];
  uint16_t TADCt;

  // float temperature = 0.0;
  // uint8_t UART_buff;
  // uint16_t UART_buff_length;

  
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 0);

  register_address = TMAG5170_TEMP_RESULT_REG;
  uint8_t FirstFrame = register_address;
  FirstFrame |= 0b1 << 7;
  HAL_SPI_Transmit(&hspi1, &FirstFrame, 1, HAL_MAX_DELAY);

  HAL_SPI_Receive(&hspi1, buff[0], 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, buff[1], 1, HAL_MAX_DELAY);

  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);

  HAL_UART_Transmit(&huart2, buff[1], strlen(buff[1]), HAL_MAX_DELAY);


  TADCt = ((uint16_t)buff[0] << 4)|buff[1]>>4;

  temp = 25.0 + ((TADCt + 17522.0)/60.0);

  return temp;

  

  //HAL_UART_Transmit(&huart2, TADCt, strlen(TADCt), HAL_MAX_DELAY);

  //UART_buff_length = sprintf(UART_buff, "%f\r\n", temperature);
  //HAL_UART_Transmit(&huart2, UART_buff, UART_buff_length, HAL_MAX_DELAY );

  // char buffff[100];
  // gcvt(temp, 4, buffff);
  // HAL_UART_Transmit(&huart2, (uint8_t)buffff, strlen(buffff), HAL_MAX_DELAY );

}

void GetMagnitude(void)
{
  // Received data
  uint8_t RxData[2];
  uint16_t data_reg;

  // Magnitude
  uint16_t M;


  // Send address
  register_address = TMAG5170_MAGNITUDE_RESULT_REG;
  uint8_t FirstFrame = register_address;
  FirstFrame |= 0b1 << 7;

  // CS LOW
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 0);
  // Transmit address
  HAL_SPI_Transmit(&hspi1, &FirstFrame, 1, HAL_MAX_DELAY);
  // Receive data
  HAL_SPI_Receive(&hspi1, RxData, 2, HAL_MAX_DELAY);
  // CS HIGH
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);

  // uint8_t to uint16_t
  data_reg = RxData[0];
  data_reg = data_reg << 8;
  data_reg |= RxData[1];

  // read Magnitude 13 LSB
  uint16_t and = data_reg;
  M = and & 0x1FFF;


  char str[8];

  // converting to char
  sprintf(str, "%d", M); 

  // sending data to console
  HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
}