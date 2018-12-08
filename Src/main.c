
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */


#include "fonts.h"
#include "nokia5110.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


typedef enum ButState
{							/**/
	RELEASED,				/**/
	PRESSED
} ButtonState;
ButtonState but1state = RELEASED;	/* i can't use it in file stm32f1xx_it.c 	_      _	*/
enum ButState but2state = RELEASED;										/*		 \_:[_/		*/


uint8_t x_adr, y_adr, dataDHT[5];
uint8_t but1, but2;	/*but_=1 means button pressed, 0 - released*/
uint8_t screen_state;	/* =0 - showing temp and rh, =1 - menu */
uint8_t menu_state;
uint16_t rh, tmpr, timeouts;
char dataBME[20], bufTx[100];

int32_t t_fine, adc_TTx;
int32_t adc_HTx;
//uint16_t dig_T1, dig_T2, dig_T3;
//uint16_t dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/*HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);*/


uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);

void PinAOut(uint16_t GPIO_Pin);
void PinAIn(uint16_t GPIO_Pin);

void StartDHT(uint16_t GPIO_Pin);
void ReadDHT(uint16_t GPIO_Pin);

void PrintRHTmprFont(uint16_t rh_or_tmpr);
void PrintBig(uint8_t num[][14]);
void PrintToBig(uint8_t number);
void PrintRHTmprBig(uint16_t rh_or_tmpr);
void PrintString(uint8_t *strng);
void myputc(uint8_t symbol);
void PrintVariable(uint16_t var);
void Screen0(void);
void Screen1(void);
void Switch0_1(void);
void Switch1_0(void);
void Switch1_2(void);
void Switch2_0(void);

int32_t BME280_tmpr();//(int32_t adc_T);
uint32_t BME280_h();
void BME280_reset();




/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */



  LCD_Init();


  HAL_TIM_Base_Start_IT(&htim3);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  LCD_RAM_Clr();
  SetXY(24,2);
  PrintString("Привет");
  HAL_Delay(200);
  PrintFrame();

  HAL_Delay(900);	/*DHT2_ NEEDS 2S DELAY FOR INITIALIZATION*/


  HAL_Delay(4000);


  //BME280_reset();


  //set filter and stby duration
  HAL_Delay(5);
  dataBME[0] = 0b10001000;	//set filter coefficient 4 and stby duration 500ms
  if( HAL_I2C_Mem_Write(&hi2c1, 0xEC, 0xF5, I2C_MEMADD_SIZE_8BIT, dataBME, 1, 0x10000)==HAL_OK )
	  CDC_Transmit_FS("\nset filter 4\nset stdby duration 500ms", strlen);

  //normal mode launch
  HAL_Delay(5);//
  dataBME[0] = 0b00000001;	//set humidity oversampling *1
  if( HAL_I2C_Mem_Write(&hi2c1, 0xEC, 0xF2, I2C_MEMADD_SIZE_8BIT, dataBME, 1, 0x10000)==HAL_OK )
	  CDC_Transmit_FS("\nset humidity oversampling *1", strlen);
  HAL_Delay(5);//
  dataBME[0] = 0b00100011;	//set temperature oversampling *1
  if( HAL_I2C_Mem_Write(&hi2c1, 0xEC, 0xF4, I2C_MEMADD_SIZE_8BIT, dataBME, 1, 0x10000)==HAL_OK )
	  CDC_Transmit_FS("\nset temperature oversampling *1\nnormal mode launch", strlen);



  while(1)
  {



	  if( HAL_I2C_Mem_Read(&hi2c1, 0xEC, 0xF4, I2C_MEMADD_SIZE_8BIT, dataBME, 1, 0x10000)==HAL_OK )
		  /*CDC_Transmit_FS("\nctrl reading", strlen)*/;
	  HAL_Delay(5);
	  sprintf(bufTx, "\nctrl = 0x%X", dataBME[0]);
	  CDC_Transmit_FS(bufTx, strlen(bufTx));

	  HAL_Delay(5);
	  sprintf(bufTx, "\nadc_T = 0x%X", adc_TTx);
	  CDC_Transmit_FS(bufTx, strlen(bufTx));

	  HAL_Delay(5);
	  sprintf(bufTx, "\nadc_H = 0x%X", adc_HTx);
	  CDC_Transmit_FS(bufTx, strlen(bufTx));

	  //temperature reading
	  sprintf(bufTx, "\nBME280tmpr = %ld", BME280_tmpr());
	  HAL_Delay(5);
	  CDC_Transmit_FS(bufTx, strlen(bufTx));

	  //humidity reading
	  sprintf(bufTx, "\nBME280hum = %ld", BME280_h());
	  HAL_Delay(5);
	  CDC_Transmit_FS(bufTx, strlen(bufTx));



	  switch(screen_state)
	  {
	  case 0:
		  Screen0();
		  break;
	  case 1:
		  HAL_Delay(500);	/* without this doesn't work 		 */
		  break;			/* switch to Screen0 (optimization?) */
	  case 2:
		  break;
	  default:
		  /*PinReset(LEDB12_GPIO_Port, LEDB12_Pin);*/	/* error */
		  HAL_GPIO_WritePin(LEDB12_GPIO_Port, LEDB12_Pin, GPIO_PIN_RESET);
		  break;
	  }

  }


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 11;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 11999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 29;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GND_Pin|LED_Pin|CLK_Pin|DIN_Pin 
                          |DC_Pin|CE_Pin|RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VCC_GPIO_Port, VCC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDB12_GPIO_Port, LEDB12_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : GND_Pin LED_Pin VCC_Pin CLK_Pin 
                           DIN_Pin DC_Pin CE_Pin RST_Pin */
  GPIO_InitStruct.Pin = GND_Pin|LED_Pin|VCC_Pin|CLK_Pin 
                          |DIN_Pin|DC_Pin|CE_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON1_Pin BUTTON2_Pin */
  GPIO_InitStruct.Pin = BUTTON1_Pin|BUTTON2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LEDB12_Pin */
  GPIO_InitStruct.Pin = LEDB12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDB12_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDA1_Pin SDA2_Pin */
  GPIO_InitStruct.Pin = SDA1_Pin|SDA2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



void PinAOut(uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}
void PinAIn(uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void StartDHT(uint16_t GPIO_Pin)
{
	PinAOut(GPIO_Pin);
	/*PinReset(GPIOA, GPIO_Pin);*/
	HAL_GPIO_WritePin(GPIOA, GPIO_Pin, GPIO_PIN_RESET);
	HAL_Delay(2);				/*delay 2ms*/
	PinAIn(GPIO_Pin);

	while( HAL_GPIO_ReadPin(GPIOA, GPIO_Pin)==GPIO_PIN_SET );
	while( HAL_GPIO_ReadPin(GPIOA, GPIO_Pin)==GPIO_PIN_RESET );
	while( HAL_GPIO_ReadPin(GPIOA, GPIO_Pin)==GPIO_PIN_SET );


}
void ReadDHT(uint16_t GPIO_Pin)
{		/* ReadDHT function executing ~10ms */
	uint8_t i, j;

	HAL_NVIC_DisableIRQ(TIM3_IRQn);	/* try later to disable interrupts after StartDHT() */

	StartDHT(GPIO_Pin);

	for(i=0; i<5; i++)
	{
		for(j=7; j<=7; j--)
		{
			TIM2->CNT = 0;
			while( HAL_GPIO_ReadPin(GPIOA, GPIO_Pin)==GPIO_PIN_RESET );
			TIM2->CR1 = 1;
			while( HAL_GPIO_ReadPin(GPIOA, GPIO_Pin)==GPIO_PIN_SET )
				{
					if( TIM2->CNT > 200 )
					{
						TIM2->CNT = 0;
						TIM2->CR1 = 0;
						timeouts++;
						return;
					}
				}
			TIM2->CR1 = 0;

			if( (TIM2->CNT)>MICROSECONDS50 )	/*75us: data bit=1*/
				dataDHT[i] |= 1<<j;				/*25us: data bit=0*/
			else
				dataDHT[i] &= ~(1<<j);
		}
	}

	HAL_NVIC_EnableIRQ(TIM3_IRQn);

	/*MAYBE LATER SHOULD  ADD CHECKING DATA FROM DHT BY CRC*/

}
/*void PrintRHTmprFont(uint16_t rh_or_tmpr)
{
	uint8_t tens, units, tenth;

	tenth = rh_or_tmpr%10;
	rh_or_tmpr /= 10;
	units = rh_or_tmpr%10;
	tens = rh_or_tmpr/10;

	PrintFont(tens);
	PrintFont(units);
	PrintFont(tenth);
}*/
void PrintBig(uint8_t num[][14])
{
	uint8_t i, j;

	for(i=0; i<3; i++)
	{
		XadressLCD();
		if( y_adr<3 )
			y_adr = i;
		else
			y_adr = i+3;
		YadressLCD();

		for(j=0; j<14; j++)
			SendData( num[i][j] );
	}

}
void PrintToBig(uint8_t number)
{
	switch(number)
	{
	case 0:
		PrintBig(num0);
		break;
	case 1:
		PrintBig(num1);
		break;
	case 2:
		PrintBig(num2);
		break;
	case 3:
		PrintBig(num3);
		break;
	case 4:
		PrintBig(num4);
		break;
	case 5:
		PrintBig(num5);
		break;
	case 6:
		PrintBig(num6);
		break;
	case 7:
		PrintBig(num7);
		break;
	case 8:
		PrintBig(num8);
		break;
	case 9:
		PrintBig(num9);
		break;
	default:
		PrintBig(num0);
		break;
	}
}
void PrintRHTmprBig(uint16_t rh_or_tmpr)
{
	uint8_t tens, units, tenth, temp;


	if( rh_or_tmpr==tmpr )
	{
		temp = 0;
//		CDC_Transmit_FS("\ntmp=", 5);
	}
	else
	{
		temp = 3;
//		CDC_Transmit_FS("\trh=", 4);
	}

	tenth = rh_or_tmpr%10;
	rh_or_tmpr /= 10;
	units = rh_or_tmpr%10;
	tens = rh_or_tmpr/10;


		x_adr = 1;
		y_adr = temp;
		PrintToBig(tens);
		x_adr = 16;
		y_adr = temp;
		PrintToBig(units);
		x_adr = 35;
		y_adr = temp;
		PrintToBig(tenth);


//		sprintf(bufTx, "%d%d,%d", tens, units, tenth);
//		CDC_Transmit_FS(bufTx, strlen(bufTx));

}
void PrintString(uint8_t *strng)
{
	while(*strng)
	{
		myputc(*strng++);

	}
}
void myputc(uint8_t symbol)
{
	uint8_t i;

	for(i=0; i<5; i++)
		SendData( FontTable [symbol][i] );

	x_adr += 6;
	XadressLCD();
}
void PrintVariable(uint16_t var)
{
	uint8_t hundreds, tens, units;

	hundreds = var/100;
	var %= 100;
	tens = var/10;
	units = var%10;

/*	tenth = var%10;
	var /= 10;
	units = var%10;
	tens = var/10;*/

	myputc(hundreds+48);
	myputc(tens+48);
	myputc(units+48);
}

void Screen0(void)
{
	ReadDHT(SDA1_Pin);				//reading DHT21
	rh = dataDHT[0]*256 + dataDHT[1];
	tmpr = dataDHT[2]*256 + dataDHT[3];
	PrintRHTmprBig(tmpr);
	PrintRHTmprBig(rh);
	HAL_Delay(1000);

	if( !screen_state )
	{
		ReadDHT(SDA2_Pin);				//reading DHT22
		rh = dataDHT[0]*256 + dataDHT[1];
		tmpr = dataDHT[2]*256 + dataDHT[3];
		PrintRHTmprBig(tmpr);
		PrintRHTmprBig(rh);
		HAL_Delay(1000);
	}
}
void Screen1(void)
{
	LCD_RAM_Clr();
	PrintString("     Меню");
	SetXY(0,1);
	PrintString("о 0 пункт");
	SetXY(0,2);
	PrintString("  1 пункт");
	SetXY(0,3);
	PrintString("  2 выход");
	SetXY(0,4);
	PrintString("таймауты ");
	PrintVariable(timeouts);

}
void Switch0_1(void)
{
	SetXY(0,1);
	PrintString(" ");
	SetXY(0,2);
	PrintString("о");
}
void Switch1_0(void)
{
	SetXY(0,1);
	PrintString("о");
	SetXY(0,2);
	PrintString(" ");
}
void Switch1_2(void)
{
	SetXY(0,2);
	PrintString(" ");
	SetXY(0,3);
	PrintString("о");
}
void Switch2_0(void)
{
	SetXY(0,3);
	PrintString(" ");
	SetXY(0,1);
	PrintString("о");
}

int32_t BME280_tmpr()	//(int32_t adc_T)
{
	int32_t adc_T, var1, var2, T;
	uint16_t dig_T1;
	int16_t dig_T2, dig_T3;


	if( HAL_I2C_Mem_Read(&hi2c1, 0xEC, 0xFA, I2C_MEMADD_SIZE_8BIT, dataBME, 3, 0x10000)==HAL_OK )
		/*CDC_Transmit_FS("\ntemperature raw received ", strlen)*/;
	adc_T = (((int32_t)dataBME[0])<<16) + (((int32_t)dataBME[1])<<8) + (int32_t)dataBME[2];
	adc_T &= 0x00FFFFFF;
	adc_T >>= 4;

	adc_TTx = adc_T;


	if( HAL_I2C_Mem_Read(&hi2c1, 0xEC, 0x88, I2C_MEMADD_SIZE_8BIT, dataBME, 6, 0x10000)==HAL_OK )
		/*CDC_Transmit_FS("\ndig_T1-T3 received ", strlen)*/;
	dig_T1 = (uint16_t)dataBME[0] + (((uint16_t)dataBME[1])<<8);
	dig_T2 = (uint16_t)dataBME[2] + (((uint16_t)dataBME[3])<<8);
	dig_T3 = (uint16_t)dataBME[4] + (((uint16_t)dataBME[5])<<8);
	HAL_Delay(5);
	sprintf(bufTx, "\ndig_T1 = %d \ndig_T2 = %d \ndig_T3 = %d", dig_T1, dig_T2, dig_T3);
	CDC_Transmit_FS(bufTx, strlen(bufTx));

	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *
			((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t BME280_h()	//(int32_t adc_H)
{
	int32_t v_x1_u32r, adc_H;
	uint8_t dig_H1, dig_H3;
	int16_t dig_H2, dig_H4, dig_H5;
	int8_t dig_H6;

	if( HAL_I2C_Mem_Read(&hi2c1, 0xEC, 0xFD, I2C_MEMADD_SIZE_8BIT, dataBME, 2, 0x10000)==HAL_OK )
		/*CDC_Transmit_FS("\nhumidity raw received ", strlen)*/;
	adc_H = ((int32_t)dataBME[0]<<8) + (int32_t)dataBME[1];
	adc_HTx = adc_H;


	if( HAL_I2C_Mem_Read(&hi2c1, 0xEC, 0xA1, I2C_MEMADD_SIZE_8BIT, dataBME, 1, 0x10000)==HAL_OK )
		;
	dig_H1 = dataBME[0];
	if( HAL_I2C_Mem_Read(&hi2c1, 0xEC, 0xE1, I2C_MEMADD_SIZE_8BIT, dataBME, 2, 0x10000)==HAL_OK )
		;
	dig_H2 = (int16_t)dataBME[0] + (((int16_t)dataBME[1])<<8);
	if( HAL_I2C_Mem_Read(&hi2c1, 0xEC, 0xE3, I2C_MEMADD_SIZE_8BIT, dataBME, 1, 0x10000)==HAL_OK )
		;
	dig_H3 = dataBME[0];
	if( HAL_I2C_Mem_Read(&hi2c1, 0xEC, 0xE4, I2C_MEMADD_SIZE_8BIT, dataBME, 2, 0x10000)==HAL_OK )
			;
	dig_H4 = (((int16_t)dataBME[0])<<4) + (((int16_t)dataBME[1])&0x0F);
	if( HAL_I2C_Mem_Read(&hi2c1, 0xEC, 0xE6, I2C_MEMADD_SIZE_8BIT, dataBME, 1, 0x10000)==HAL_OK )
		;
	dig_H5 = (((int16_t)dataBME[1])>>4) + (((int16_t)dataBME[0])<<4);
	if( HAL_I2C_Mem_Read(&hi2c1, 0xEC, 0xE7, I2C_MEMADD_SIZE_8BIT, dataBME, 1, 0x10000)==HAL_OK )
			;
	dig_H6 = dataBME[0];

	HAL_Delay(5);
	sprintf(bufTx, "\ndig_H1 = %d \ndig_H2 = %d \ndig_H3 = %d \ndig_H4 = %d \ndig_H5 = %d \ndig_H6 = %d",
				dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6);
	CDC_Transmit_FS(bufTx, strlen(bufTx));


	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
			((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r *
			((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
			((int32_t)dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r>>12);

}

void BME280_reset()
{
	  HAL_Delay(5);//
	  dataBME[0] = 0xB6;
	  if( HAL_I2C_Mem_Write(&hi2c1, 0xEC, 0xE0, I2C_MEMADD_SIZE_8BIT, dataBME, 1, 0x10000)==HAL_OK )
		  CDC_Transmit_FS("\nreset BME280", strlen);
	  HAL_Delay(1000);
	  if( HAL_I2C_Mem_Read(&hi2c1, 0xEC, 0xF3, I2C_MEMADD_SIZE_8BIT, dataBME, 1, 0x10000)==HAL_OK )
	  		  CDC_Transmit_FS("\nstatus reading", strlen);
	  HAL_Delay(5);
	  dataBME[0] &= 0b00001001;
	  sprintf(bufTx, "\nstatus=0x%X", dataBME[0]);
	  CDC_Transmit_FS(bufTx, strlen(bufTx));
}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
	  /*PinReset(LEDB12_GPIO_Port, LEDB12_Pin);*/
	  HAL_GPIO_WritePin(LEDB12_GPIO_Port, LEDB12_Pin, GPIO_PIN_RESET);
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
