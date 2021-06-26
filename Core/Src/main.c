/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stdint.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint32_t CRL;
	uint32_t CRH;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t BSRR;
	uint32_t BRR;
	uint32_t LCKR;
} GPIO;
typedef struct
{
	uint32_t SR;
	uint32_t DR;
	uint32_t BRR;
	uint32_t CR1;
	uint32_t CR2;
	uint32_t CR3;
	uint32_t GTPR;
} USART;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
GPIO *GPIO_A = 0x40010800;
USART *USART_1 = 0x40013800;
char receive_firmware[4096] = {0};
__attribute__((section(".vector_table")))char vector_table[1024] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void Error_Handler(void);
void erase_FLASH(uint32_t address, uint8_t sectors)__attribute__((section(".run_in_ram")));
void write_FLASH(uint32_t address, uint16_t data)__attribute__((section(".run_in_ram")));
int wait_operation()__attribute__((section(".run_in_ram")));
char find_ok()__attribute__((section(".run_in_ram")));
void update_firmware()__attribute__((section(".run_in_ram")));
static void move_vector_table_to_RAM(void)__attribute__((section("run_in_ram")));
static void MX_USART_Init(void)__attribute__((section("run_in_ram")));
static void setup_GPIOA(void)__attribute__((section("run_in_ram")));
static void setup_USART1(void)__attribute__((section("run_in_ram")));
static void setup_DMA1(void)__attribute__((section("run_in_ram")));
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MX_USART_Init()
{
	// Enable USART clock
	__HAL_RCC_USART1_CLK_ENABLE();
}
void setup_GPIOA()
{
	GPIO_A->CRH &= ~(0xFF << 4); 					// Reset config of PORTA9,11
	GPIO_A->CRH |= (0b1011 << 4) | (0b0100 << 8);	// Set PORTA9 to output mode AF push-pull and PORTA10 to floating input
}
void setup_USART1()
{
	USART_1->BRR = (52 << 4) | 1;			// Set USART1 Baud rate
	USART_1->CR1 |= (1 << 12);				// Set Data length of 1 start bit, 9 data bits, n stop bits
	USART_1->CR1 |= (1 << 10);				// Enable parity control
	USART_1->CR1 &= ~(1 << 9);				// Select even parity
	USART_1->CR3 |= (1 << 6);				// Enable DMA receiver
	USART_1->CR1 |= (1 << 13);				// Enable USART1
	USART_1->CR1 |= (1 << 2) | (1 << 3);	// Enable USART1 transmitter and receiver
}
void setup_DMA1()
{
	uint32_t* RCC_AHBENR = (uint32_t*)0x40021014;
	*RCC_AHBENR |= (1 << 0);
	// Set buffer memory address
	uint32_t* DMA1_CMAR5 = (uint32_t*)0x40020064;
	*DMA1_CMAR5 = (uint32_t)receive_firmware;
	// Set size of buffer memory
	uint32_t* DMA1_CNDTR5 = (uint32_t*)0x4002005C;
	*DMA1_CNDTR5 = sizeof(receive_firmware);
	// Set peripheral memory address
	uint32_t* DMA1_CPAR5 = (uint32_t*)0x40020060;
	*DMA1_CPAR5 = (uint32_t)0x40013804;
	// Enable circular, increment mode and DMA1 channel
	uint32_t* DMA1_CCR5 = (uint32_t*)0x40020058;
	*DMA1_CCR5 |= (1 << 7) | (1 << 5) | 1;
}
void move_vector_table_to_RAM()
{
	memcpy(&vector_table, 0x08000000, 1024);
	uint32_t *VTOR = 0xE000ED08;
	*VTOR = 0x20000000;
}
void erase_FLASH(uint32_t address, uint8_t numSector)
{
	uint32_t* FLASH_SR = (uint32_t*)0x4002200C;
	uint32_t* FLASH_CR = (uint32_t*)0x40022010;
	uint32_t* FLASH_AR = (uint32_t*)0x40022014;

	if(((*FLASH_CR >> 1) & 1 )!= 1)
	{
		*FLASH_CR |= (1 << 1);		//Page Erase chosen
	}
	if((*FLASH_CR & 1) == 1)
	{
		*FLASH_CR &= ~(1UL);
	}
	for(int i = 0; i < numSector; i++)
	{
		*FLASH_CR &= ~1UL;
		while(((*FLASH_SR) & 1) == 1);
		*FLASH_AR = address + 0x400 * i;
		*FLASH_CR |=  (1 << 6);		//Start

		while(((*FLASH_SR >> 5) & 1) != 1);
		*FLASH_SR |= (1 << 5);
	}

    *FLASH_CR &= ~(1u << 1);
}

void write_FLASH(uint32_t address, uint16_t data2Write)
{
	uint32_t* FLASH_CR = (uint32_t*)0x40022010;
	volatile uint16_t* pData;
	*FLASH_CR |= 1;
	pData = address;
	wait_operation();

	*pData = data2Write;
	wait_operation();
	*FLASH_CR &= ~1u;
}

int wait_operation()
{
	uint32_t* FLASH_SR = (uint32_t*)0x4002200C;
	uint32_t* FLASH_WRPR = (uint32_t*)0x4002201C;
	while(*FLASH_SR & 1);

	if((*FLASH_SR << 5) & 1)
	{
		*FLASH_SR |= (1 << 5);
	}

	if(((*FLASH_WRPR >> 1) &1 )  || ((*FLASH_WRPR) & 1) || ((*FLASH_SR >> 2) & 1))
	{
		while(1);
	}

	return HAL_OK;
}

char find_ok()
{
	int size = sizeof(receive_firmware) - 1;
	for(int i = 0; i < size; i++)
	{
		if ((receive_firmware[i] == 'O') && (receive_firmware[i+1] == 'K'))
			return 1;
	}
	return 0;
}

void update_firmware()
{
	while(find_ok() == 0);
	__HAL_RCC_FLITF_CLK_ENABLE();
	HAL_FLASH_Unlock();
	int size = sizeof(receive_firmware);
	erase_FLASH(0x08000000, 4);
	for(int i = 0; i < size; i+=2)
	{
		write_FLASH(0x08000000 + i, *(uint16_t*)(receive_firmware + i));
	}
	uint32_t* AIRCR = (uint32_t*)0xE000ED0C;
	*AIRCR = (0x5FA << 16) | (1 << 2);
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  move_vector_table_to_RAM();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  MX_USART_Init();
  setup_GPIOA();
  setup_USART1();
  setup_DMA1();
  uint32_t *system_tick = (uint32_t*)0xe000e010;
  *system_tick &= ~(1 << 1);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
