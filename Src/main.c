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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct  report_desc_s
{
	uint8_t hat      : 4; // (hat format, 0x08 is released, 0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW)
	uint8_t square   : 1; // west
	uint8_t cross    : 1; // south
	uint8_t circle   : 1; // east
	uint8_t triangle : 1; // north
    
	uint8_t L1     : 1;
	uint8_t R1     : 1;
	uint8_t L2     : 1;
	uint8_t R2     : 1;
	uint8_t share  : 1;
	uint8_t option : 1;
	uint8_t L3     : 1;
	uint8_t R3     : 1;
} report_desc;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void _delay_loops(uint32_t loops) 
{
	DWT->CYCCNT = 0; 
	while(DWT->CYCCNT < loops) {}
}

#define delay_us( US ) _delay_loops(US*72)
#define delay_ms( MS ) _delay_loops(MS*72000)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

uint8_t key_Z, key_Y, key_X, key_R;
uint8_t key_B, key_C, key_A, key_St;
uint8_t key_Up, key_Dn, key_Lt, key_Rt;
uint8_t key_L;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void (*update_input)(void) = NULL;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void update_SS(void)
{
	uint8_t t[4];
	
	TH_HIGH();
	TR_HIGH();
	delay_us(20);
	t[0] = SS_Port->IDR >> 10;
	
	TH_LOW();
	TR_HIGH();
	delay_us(20);
	t[1] = SS_Port->IDR >> 10;
	
	TH_HIGH();
	TR_LOW();
	delay_us(20);
	t[2] = SS_Port->IDR >> 10;
	
	TH_LOW();
	TR_LOW();
	delay_us(20);
	t[3] = SS_Port->IDR >> 10;
	
	TH_HIGH();
	TR_HIGH();
	
	key_L  = !(t[0] & 8);
	
	key_B  = !(t[1] & 1);
    key_C  = !(t[1] & 2);
    key_A  = !(t[1] & 4);
    key_St = !(t[1] & 8);
    
    key_Up = !(t[2] & 1);
    key_Dn = !(t[2] & 2);
    key_Lt = !(t[2] & 4);
    key_Rt = !(t[2] & 8);
    
    key_Z  = !(t[3] & 1);
	key_Y  = !(t[3] & 2);
	key_X  = !(t[3] & 4);
	key_R  = !(t[3] & 8);
}

static void update_MD(void)
{
	uint8_t t[8];
	
	TH_HIGH();
	delay_us(20);
	
	t[0] = SS_Port->IDR >> 10;
	
	TH_LOW();
	delay_us(20);
	
	t[1] = SS_Port->IDR >> 10;
	
	TH_HIGH();
	delay_us(20);
	
	t[2] = SS_Port->IDR >> 10;
	
	TH_LOW();
	delay_us(20);
	
	t[3] = SS_Port->IDR >> 10;
	
	TH_HIGH();
	delay_us(20);
	
	t[4] = SS_Port->IDR >> 10;
	
	TH_LOW();
	delay_us(20);
	
	t[5] = SS_Port->IDR >> 10;
	
	TH_HIGH();
	delay_us(20);
	
	t[6] = SS_Port->IDR >> 10;
	
	TH_LOW();
	delay_us(20);
	
	t[7] = SS_Port->IDR >> 10;
	
	TH_HIGH();
	
	key_Z  = !(t[5] & 1);
	key_Y  = !(t[5] & 2);
	key_X  = !(t[5] & 4);
	key_R  = !(t[5] & 8);
	
	key_B  = !(t[1] & 16);
    key_C  = !(t[1] & 32);
    key_A  = !(t[0] & 16);
    key_St = !(t[0] & 32);
    
    key_Up = !(t[1] & 1);
    key_Dn = !(t[1] & 2);
    key_Lt = !(t[1] & 4);
    key_Rt = !(t[1] & 8);
    
    key_L  = 0;
}

__STATIC_INLINE void DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;   // запускаем счётчик
}

void detect_joy()
{
	uint8_t ID = 0;
	
	// Detect Saturn/MegaDrive (0x0B - Saturn std pad; 0x0D - MegaDrive 3 button; 0x0C - MegaDrive 6 button)
	TH_HIGH();
	delay_us(2);
	
	ID  = ((SS_Port->IDR >> 10) & 8) | ((SS_Port->IDR >> 9) & 8);
	ID |= ((SS_Port->IDR >> 9 ) & 4) | ((SS_Port->IDR >> 8) & 4);
	
	TH_LOW();
	delay_us(2);
	
	ID |= ((SS_Port->IDR >> 12) & 2) | ((SS_Port->IDR >> 11) & 2);
	ID |= ((SS_Port->IDR >> 11) & 1) | ((SS_Port->IDR >> 10) & 1);
	
	TH_HIGH();
#ifdef DEBUG
	printf("SS/MD ID = %X\n", ID);
#endif
	switch (ID)
	{
		case 0x0B:
			update_input = &update_SS;
			
			LL_GPIO_SetPinMode(SS_Port, SS_S0, LL_GPIO_MODE_OUTPUT);
			LL_GPIO_SetPinOutputType(SS_Port, SS_S0, LL_GPIO_OUTPUT_PUSHPULL);
			LL_GPIO_SetPinSpeed(SS_Port, SS_S0, LL_GPIO_SPEED_FREQ_HIGH);
			TR_HIGH();
			LL_GPIO_ResetOutputPin(LED_Port, LED_GREEN);
			LL_GPIO_SetOutputPin(GPIOB, LED_BLUE);
			return;
			break;
		
		case 0x0C:
		case 0x0D:
			update_input = &update_MD;
			LL_GPIO_ResetOutputPin(LED_Port, LED_GREEN);
			LL_GPIO_SetOutputPin(GPIOB, LED_BLUE);
			return;
			break;
		
		case 0x0F:
			break;
		
		default:
			while(1)
			{
				LL_GPIO_TogglePin(LED_Port, LED_GREEN);
				LL_GPIO_TogglePin(GPIOB, LED_BLUE);
				delay_ms(500);
			}
	}
	
	update_input = &update_SS;
	//LL_GPIO_ResetOutputPin(LED_Port, LED_GREEN);
	//LL_GPIO_TogglePin(LED_Port, LED_GREEN);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	report_desc report_data;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	DWT_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	detect_joy();
	//LL_GPIO_ResetOutputPin(LED_Port, LED_GREEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
	{
		update_input();
		
		if (key_Up)
		{
			if (key_Rt)
			{
				report_data.hat = 1;
			}
			else if (key_Lt)
			{
				report_data.hat = 7;
			}
			else
			{
				report_data.hat = 0;
			}
		}
		else if (key_Dn)
		{
			if (key_Rt)
			{
				report_data.hat = 3;
			}
			else if (key_Lt)
			{
				report_data.hat = 5;
			}
			else
			{
				report_data.hat = 4;
			}
		}
		else if (key_Rt)
		{
			report_data.hat = 2;
		}
		else if (key_Lt)
		{
			report_data.hat = 6;
		}
		else
		{
			report_data.hat = 8;
		}
		
		report_data.square   = key_X;
		report_data.cross    = key_A;
		report_data.circle   = key_B;
		report_data.triangle = key_Y;
		report_data.L1		 = key_Z;
		report_data.R1		 = key_C;
		report_data.L2		 = key_L;
		report_data.R2		 = key_R;
		report_data.share	 = (BUTTON_Port->IDR & 1);
		report_data.option	 = key_St;
		report_data.L3		 = !(SS_Port->IDR & 1);
		report_data.R3		 = !((SS_Port->IDR >> 1) & 1);

		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *) &report_data, sizeof(uint16_t));
		delay_ms(16);
	}
	
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(72000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL_DIV_1_5);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	/* GPIO Ports Clock Enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
	
	/**/
	GPIO_InitStruct.Pin = USR_BUTTON;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(BUTTON_Port, &GPIO_InitStruct);
	
	/**/
	TH_HIGH();
	
	/**/
	GPIO_InitStruct.Pin = SS_S1 | LED_BLUE;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(SS_Port, &GPIO_InitStruct);
	
	/**/
	GPIO_InitStruct.Pin = SS_S0 | SS_D0 | SS_D1 | SS_D2 | SS_D3 | SS_D4 | L3BTN | R3BTN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(SS_Port, &GPIO_InitStruct);
	
	/**/
	GPIO_InitStruct.Pin = LED_GREEN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	LL_GPIO_Init(LED_Port, &GPIO_InitStruct);
	LL_GPIO_ResetOutputPin(LED_Port, LED_GREEN);
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
