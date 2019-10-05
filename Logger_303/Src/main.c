/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "sd_hal_mpu6050.h"
#include "bmp280.h"
#include <string.h>
#include <stdarg.h>
#include "sdcard.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
SD_MPU6050 mpu1;

BMP280_HandleTypedef bmp280;
int32_t temperature;
uint32_t pressure, humidity;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_Printf(const char* fmt, ...) {
    char buff[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
    HAL_UART_Transmit(&huart2, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
    va_end(args);
}

void init() {
    int code;
    UART_Printf("Ready!\r\n");

    code = SDCARD_Init();
    if(code < 0) {
        UART_Printf("SDCARD_Init() failed: code = %d\r\n", code);
        return;
    }

    UART_Printf("SDCARD_Init() done!\r\n");

    uint32_t blocksNum;
    code = SDCARD_GetBlocksNumber(&blocksNum);
    if(code < 0) {
        UART_Printf("SDCARD_GetBlocksNumber() failed: code = %d\r\n", code);
        return;
    }

    UART_Printf("SDCARD_GetBlocksNumber() done! blocksNum = %u (or %u Mb)\r\n",
        blocksNum, blocksNum/2000 /* same as * 512 / 1000 / 1000 */);

    uint32_t startBlockAddr = 0x00ABCD;
    uint32_t blockAddr = startBlockAddr;
    uint8_t block[512];

    snprintf((char*)block, sizeof(block), "0x%08X", (int)blockAddr);

    code = SDCARD_WriteSingleBlock(blockAddr, block);
    if(code < 0) {
        UART_Printf("SDCARD_WriteSingleBlock() failed: code = %d\r\n", code);
        return;
    }
    UART_Printf("SDCARD_WriteSingleBlock(0x%08X, ...) done!\r\n", blockAddr);

    memset(block, 0, sizeof(block));

    code = SDCARD_ReadSingleBlock(blockAddr, block);
    if(code < 0) {
        UART_Printf("SDCARD_ReadSingleBlock() failed: code = %d\r\n", code);
        return;
    }

    UART_Printf("SDCARD_ReadSingleBlock(0x%08X, ...) done! block = \"%c%c%c%c%c%c%c%c%c%c...\"\r\n",
        blockAddr, block[0], block[1], block[2], block[3], block[4], block[5], block[6], block[7], block[8], block[9]);

    blockAddr = startBlockAddr + 1;
    code = SDCARD_WriteBegin(blockAddr);
    if(code < 0) {
        UART_Printf("SDCARD_WriteBegin() failed: code = %d\r\n", code);
        return;
    }
    UART_Printf("SDCARD_WriteBegin(0x%08X, ...) done!\r\n", blockAddr);

    for(int i = 0; i < 3; i++) {
        snprintf((char*)block, sizeof(block), "0x%08X", (int)blockAddr);

        code = SDCARD_WriteData(block);
        if(code < 0) {
            UART_Printf("SDCARD_WriteData() failed: code = %d\r\n", code);
            return;
        }

        UART_Printf("SDCARD_WriteData() done! blockAddr = %08X\r\n", blockAddr);
        blockAddr++;
    }

    code = SDCARD_WriteEnd();
    if(code < 0) {
        UART_Printf("SDCARD_WriteEnd() failed: code = %d\r\n", code);
        return;
    }
    UART_Printf("SDCARD_WriteEnd() done!\r\n");

    blockAddr = startBlockAddr + 1;
    code = SDCARD_ReadBegin(blockAddr);
    if(code < 0) {
        UART_Printf("SDCARD_ReadBegin() failed: code = %d\r\n", code);
        return;
    }
    UART_Printf("SDCARD_ReadBegin(0x%08X, ...) done!\r\n", blockAddr);

    for(int i = 0; i < 3; i++) {
        code = SDCARD_ReadData(block);
        if(code < 0) {
            UART_Printf("SDCARD_ReadData() failed: code = %d\r\n", code);
            return;
        }

        UART_Printf("SDCARD_ReadData() done! block = \"%c%c%c%c%c%c%c%c%c%c...\"\r\n",
            block[0], block[1], block[2], block[3], block[4], block[5], block[6], block[7], block[8], block[9]);
    }

    code = SDCARD_ReadEnd();
    if(code < 0) {
        UART_Printf("SDCARD_ReadEnd() failed: code = %d\r\n", code);
        return;
    }
    UART_Printf("SDCARD_ReadEnd() done!\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	SD_MPU6050_Result result ;
	uint8_t mpu_ok[15] = {"MPU WORK FINE\n"};
	uint8_t mpu_not[17] = {"MPU NOT WORKING\n"};
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;

	while (!bmp280_init(&bmp280, &bmp280.params)) {
		UART_Printf("BMP280: initialization failed. \r\n");
		HAL_Delay(2000);
	}
	bool bme280p = bmp280.id == BME280_CHIP_ID;
	if(bme280p){
		UART_Printf("BME280 detected... \r\n");
	}

  UART_Printf("Init status started. \r\n");
  init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  result = SD_MPU6050_Init(&hi2c1,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_250s );
	  HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(result == SD_MPU6050_Result_Ok)
	  {
		  HAL_UART_Transmit(&huart2, mpu_ok, sizeof(mpu_ok), 1000);
	  }
	  else
	  {
		  HAL_UART_Transmit(&huart2, mpu_not, sizeof(mpu_not), 1000);
	  }
	  HAL_Delay(500);
	  SD_MPU6050_ReadTemperature(&hi2c1,&mpu1);
	  SD_MPU6050_ReadGyroscope(&hi2c1,&mpu1);
	  int16_t g_x = mpu1.Gyroscope_X;
	  int16_t g_y = mpu1.Gyroscope_Y;
	  int16_t g_z = mpu1.Gyroscope_Z;
	  //HAL_UART_Transmit(&huart2, (uint8_t *)&g_x, sizeof((uint8_t *)&g_x), 1000);
	  UART_Printf("g_x: %X , g_y: %X, g_z: %X \r\n",g_x,g_y,g_z);

	  SD_MPU6050_ReadAccelerometer(&hi2c1,&mpu1);
	  int16_t a_x = mpu1.Accelerometer_X;
	  int16_t a_y = mpu1.Accelerometer_Y;
	  int16_t a_z = mpu1.Accelerometer_Z;
	  //HAL_UART_Transmit(&huart2, (uint8_t *)&a_x, sizeof((uint8_t *)&a_x), 1000);
	  UART_Printf("a_x: %X , a_y: %X , a_z: %X \r\n",a_x, a_y, a_z);

	HAL_Delay(100);
	while (!bmp280_read_fixed(&bmp280, &temperature, &pressure, &humidity)) {
		UART_Printf("Temperature/pressure reading failed\r\n");
		HAL_Delay(2000);
	}

	if (bme280p) {
		UART_Printf("[Pressure]: %X, [Temperature]: %X, [Humidity]: %X \r\n", pressure, temperature, humidity);
	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 800;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_SPI_CS_GPIO_Port, SD_SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_SPI_CS_Pin */
  GPIO_InitStruct.Pin = SD_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_SPI_CS_GPIO_Port, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
