// mpu-9250을 이용하여 센서를 읽어보자

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU9250_I2C_ADDR        (0x68 << 1)  // HAL은 8-bit 주소(<<1) 사용

/* MPU-9250 Register map */
#define MPU9250_REG_WHO_AM_I    0x75   // Expected 0x71
#define MPU9250_REG_PWR_MGMT_1  0x6B
#define MPU9250_REG_SMPLRT_DIV  0x19
#define MPU9250_REG_CONFIG      0x1A
#define MPU9250_REG_GYRO_CONFIG 0x1B
#define MPU9250_REG_ACCEL_CONFIG 0x1C
#define MPU9250_REG_ACCEL_XOUT_H 0x3B  // 0x3B~0x48 (Accel X/Y/Z, Temp, Gyro X/Y/Z)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
static float accel_lsb_per_g = 16384.0f;
static float gyro_lsb_per_dps = 131.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
static void Error_Handler_Blink(void);
static HAL_StatusTypeDef mpu9250_write_u8(uint8_t reg, uint8_t data);
static HAL_StatusTypeDef mpu9250_read_u8(uint8_t reg, uint8_t *data);
static HAL_StatusTypeDef mpu9250_read_bytes(uint8_t start_reg, uint8_t *buf, uint16_t len);
static int16_t be16_to_s16(uint8_t hi, uint8_t lo);
static HAL_StatusTypeDef mpu9250_init(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  if (mpu9250_init() != HAL_OK) {
      Error_Handler_Blink();
  }

  uint8_t raw[14];
  int16_t ax, ay, az, gx, gy, gz, temp_raw;
  float ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_c;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* 14바이트 일괄 읽기 (ACCEL[6] + TEMP[2] + GYRO[6]) */
    if (mpu9250_read_bytes(MPU9250_REG_ACCEL_XOUT_H, raw, 14) == HAL_OK)
    {
    	ax = be16_to_s16(raw[0], raw[1]);
	    ay = be16_to_s16(raw[2], raw[3]);
	    az = be16_to_s16(raw[4], raw[5]);
	    temp_raw = be16_to_s16(raw[6], raw[7]);
	    gx = be16_to_s16(raw[8], raw[9]);
	    gy = be16_to_s16(raw[10], raw[11]);
	    gz = be16_to_s16(raw[12], raw[13]);

	    // 물리 단위 변환
	    ax_g = ax / accel_lsb_per_g;
	    ay_g = ay / accel_lsb_per_g;
	    az_g = az / accel_lsb_per_g;

	    gx_dps = gx / gyro_lsb_per_dps;
	    gy_dps = gy / gyro_lsb_per_dps;
	    gz_dps = gz / gyro_lsb_per_dps;

	    // 온도: Temp in °C = (TEMP_OUT Register / 333.87) + 21  (MPU-9250 기준)
		temp_c = ((float)temp_raw) / 333.87f + 21.0f;
    }
    else {
	    // NACK/버스 에러 등: 버스 리커버리 or 재시도
	    HAL_Delay(2);
	}

	HAL_Delay(5); // 폴링 주기 예: 200 Hz(=5 ms)


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
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10800D28;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void Error_Handler_Blink(void)
{
  // 디버깅용: 에러 시 멈춤
  __disable_irq();
  while (1) { ; }
}

/* I2C1 init function (CubeMX가 생성한 함수 사용) ---------------------------*/
/* 참고: CubeMX에서 PB8/PB9, Fast Mode 400kHz, Analog filter enabled,
         Digital filter 0, Rise=200ns, Fall=100ns 설정 후 생성하세요. */

/* MPU-9250 basic write (8-bit) */
static HAL_StatusTypeDef mpu9250_write_u8(uint8_t reg, uint8_t data)
{
  return HAL_I2C_Mem_Write(&hi2c2, MPU9250_I2C_ADDR, reg,
                           I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}

/* MPU-9250 basic read (8-bit) */
static HAL_StatusTypeDef mpu9250_read_u8(uint8_t reg, uint8_t *data)
{
  return HAL_I2C_Mem_Read(&hi2c2, MPU9250_I2C_ADDR, reg,
                          I2C_MEMADD_SIZE_8BIT, data, 1, 10);
}

/* Read multiple bytes starting from reg (Polling, blocking) */
static HAL_StatusTypeDef mpu9250_read_bytes(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c2, MPU9250_I2C_ADDR, start_reg,
                          I2C_MEMADD_SIZE_8BIT, buf, len, 20);
}

/* Two's complement 16-bit to int16 */
static int16_t be16_to_s16(uint8_t hi, uint8_t lo)
{
  return (int16_t)((hi << 8) | lo);
}

/* Sensor init (power up + basic config) ------------------------------------*/
static HAL_StatusTypeDef mpu9250_init(void)
{
  HAL_StatusTypeDef ret;
  uint8_t who = 0;

  // WHO_AM_I 읽기 (0x71 예상)
  ret = mpu9250_read_u8(MPU9250_REG_WHO_AM_I, &who);
  if (ret != HAL_OK) return ret;

  if (who != 0x71) {
    // 주소(0x69)일 수 있으니 한번 더 시도해도 됨.
    return HAL_ERROR;
  }

  // 슬립 해제: PWR_MGMT_1 = 0x00 (클럭소스 Internal 8MHz)
  ret = mpu9250_write_u8(MPU9250_REG_PWR_MGMT_1, 0x00);
  if (ret != HAL_OK) return ret;
  HAL_Delay(100);

  // LPF 설정: CONFIG[DLPF_CFG]=3 (≈44Hz for Gyro, 1kHz sample) 예시
  ret = mpu9250_write_u8(MPU9250_REG_CONFIG, 0x03);
  if (ret != HAL_OK) return ret;

  // Gyro full scale: FS_SEL=0 (±250 dps)
  ret = mpu9250_write_u8(MPU9250_REG_GYRO_CONFIG, 0x00);
  if (ret != HAL_OK) return ret;

  // Accel full scale: AFS_SEL=0 (±2g)
  ret = mpu9250_write_u8(MPU9250_REG_ACCEL_CONFIG, 0x00);
  if (ret != HAL_OK) return ret;

  // 샘플 속도(내부 1kHz 가정): SMPLRT_DIV=4 → 200 Hz (예시)
  ret = mpu9250_write_u8(MPU9250_REG_SMPLRT_DIV, 4);
  if (ret != HAL_OK) return ret;

  return HAL_OK;
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
#ifdef USE_FULL_ASSERT
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
