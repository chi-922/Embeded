// Nucleo-F746ZG 보드와 LCD1602 LCD로 병렬통신을 통해 화면에 실시간으로 변하는 cnt 입력하기
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
#include <stdarg.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_RS_GPIO      GPIOB
#define LCD_RS_PIN       GPIO_PIN_11     // PB11

#define LCD_E_GPIO       GPIOB
#define LCD_E_PIN        GPIO_PIN_10     // PB10

#define LCD_D4_GPIO      GPIOE
#define LCD_D4_PIN       GPIO_PIN_15     // PE15

#define LCD_D5_GPIO      GPIOE
#define LCD_D5_PIN       GPIO_PIN_14     // PE14

#define LCD_D6_GPIO      GPIOE
#define LCD_D6_PIN       GPIO_PIN_12     // PE12

#define LCD_D7_GPIO      GPIOE
#define LCD_D7_PIN       GPIO_PIN_10     // PE10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---- 짧은 지연: us 단위 (DWT 사용) ---- */
/*
static void delay_us_init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
static void delay_us(uint32_t us) {
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = (HAL_RCC_GetHCLKFreq() / 1000000U) * us;
  while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
}
*/

/* ---- E 펄스 ---- */
static void lcd_pulse_enable(void) {
  HAL_GPIO_WritePin(LCD_E_GPIO, LCD_E_PIN, GPIO_PIN_SET);
  //delay_us(1);                  // Enable High 최소 ~450ns
  HAL_Delay(1);
  HAL_GPIO_WritePin(LCD_E_GPIO, LCD_E_PIN, GPIO_PIN_RESET);
  //delay_us(50);                 // 데이터 셋업 시간 (여유있게)
  HAL_Delay(1);
}

/* ---- 상위 4비트(또는 하위 4비트) 내보내기 ---- */
static void lcd_write4(uint8_t nibble) {
	HAL_GPIO_WritePin(LCD_D4_GPIO, LCD_D4_PIN, (nibble & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D5_GPIO, LCD_D5_PIN, (nibble & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D6_GPIO, LCD_D6_PIN, (nibble & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D7_GPIO, LCD_D7_PIN, (nibble & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	lcd_pulse_enable();
}


/* ---- 바이트 전송 (4비트 모드: 상위→하위) ---- */
static void lcd_send(uint8_t rs, uint8_t value) {
  HAL_GPIO_WritePin(LCD_RS_GPIO, LCD_RS_PIN, rs ? GPIO_PIN_SET : GPIO_PIN_RESET);
  lcd_write4(value >> 4);       // 상위 4비트
  lcd_write4(value & 0x0F);     // 하위 4비트
}

/* ---- 명령/데이터 편의 함수 ---- */
static void lcd_cmd(uint8_t cmd) { lcd_send(0, cmd); }
static void lcd_data(uint8_t data){ lcd_send(1, data); }

/* ---- 초기화 시퀀스 (HD44780 4비트) ---- */
void lcd_init(void) {
  //delay_us_init();

  HAL_Delay(50);                // 전원 인가 후 15ms 이상 대기

  // 8bit 모드 진입 시퀀스 (데이터라인 D7~D4만 사용)
  lcd_write4(0x03); HAL_Delay(5);
  lcd_write4(0x03); HAL_Delay(5);
  lcd_write4(0x03); HAL_Delay(1);

  // 4bit 모드 전환
  lcd_write4(0x02); HAL_Delay(1);

  // Function set: 4bit, 2라인, 5x8 도트
  lcd_cmd(0x28); HAL_Delay(1);

  // Display off
  lcd_cmd(0x08); HAL_Delay(1);

  // Clear display
  lcd_cmd(0x01); HAL_Delay(2);

  // Entry mode set: 커서 증가, 쉬프트 없음
  lcd_cmd(0x06); HAL_Delay(1);

  // Display on, 커서 off, 블링크 off
  lcd_cmd(0x0C); HAL_Delay(1);
}

/* ---- 커서 위치 (col:0~15, row:0~1) ---- */
void lcd_set_cursor(uint8_t col, uint8_t row) {
  static const uint8_t row_addr[2] = {0x00, 0x40};
  lcd_cmd(0x80 | (row_addr[row & 0x01] + (col & 0x0F)));
}

/* ---- 문자열 출력 ---- */
void lcd_print(const char *s) {
  while (*s) lcd_data((uint8_t)*s++);
}

/* ---- printf 스타일 간단 지원 ---- */
void lcd_printf(uint8_t col, uint8_t row, const char *fmt, ...) {
  char buf[32];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  lcd_set_cursor(col, row);
  lcd_print(buf);
}
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
  /* USER CODE BEGIN 2 */
  lcd_init();              // LCD 초기화
  lcd_set_cursor(0,0);
  lcd_print("Hello, LCD!");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  lcd_printf(0,1, "Count:%5lu", cnt++);
	  HAL_Delay(200);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, D7_Pin|D6_Pin|D5_Pin|D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_E_Pin|LCE_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D7_Pin D6_Pin D5_Pin D4_Pin */
  GPIO_InitStruct.Pin = D7_Pin|D6_Pin|D5_Pin|D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_E_Pin LCE_RS_Pin */
  GPIO_InitStruct.Pin = LCD_E_Pin|LCE_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
