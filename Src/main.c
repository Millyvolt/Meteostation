/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

#include <stdarg.h>

#define MICROSECONDS50 50

#include "fonts.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/



uint8_t x_adr, y_adr, dataDHT[5];
uint16_t rh, tmpr;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/



void LCD_Init(void);
void PinSet(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void PinReset(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void SendBitSet(void);
void SendBitClr(void);
void SendByte(uint8_t Byte);
void SendData(uint8_t Data);
void SendCom(uint8_t Command);
void LCD_RAM_Clr(void);
void PrintFrame(void);
void XadressLCD(void);
void YadressLCD(void);
void PrintFont(uint8_t symbol);
void PinAOut(uint16_t GPIO_Pin);
void PinAIn(uint16_t GPIO_Pin);
void StartDHT(uint16_t GPIO_Pin);
void ReadDHT(uint16_t GPIO_Pin);
void PrintRHTmprFont(uint16_t rh_or_tmpr);
void NextLine(void);
void PrintBig(uint8_t num[][14]);
void PrintToBig(uint8_t number);
void PrintRHTmprBig(uint16_t rh_or_tmpr);
void PrintString(uint8_t *strng);
void myputc(uint8_t symbol);



/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

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

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  LCD_Init();
  LCD_RAM_Clr();
  PrintFrame();

  HAL_Delay(1000);	/*DHT2_ NEEDS 2S DELAY FOR INITIALIZATION*/

  x_adr = 2;
  XadressLCD();
  y_adr = 1;
  YadressLCD();
  PrintString("Привет");
  HAL_Delay(9000);

  while (1)
  {


		ReadDHT(SDA1_Pin);				//reading DHT21
		rh = dataDHT[0]*256 + dataDHT[1];
		tmpr = dataDHT[2]*256 + dataDHT[3];
		PrintRHTmprBig(tmpr);
		PrintRHTmprBig(rh);
		HAL_Delay(3000);

		ReadDHT(SDA2_Pin);				//reading DHT22
		rh = dataDHT[0]*256 + dataDHT[1];
		tmpr = dataDHT[2]*256 + dataDHT[3];
		PrintRHTmprBig(tmpr);
		PrintRHTmprBig(rh);
		HAL_Delay(3000);





  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDB12_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDA1_Pin SDA2_Pin */
  GPIO_InitStruct.Pin = SDA1_Pin|SDA2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */




void LCD_Init(void)
{
	uint8_t Byte;
	uint16_t delay;

	PinSet(GPIOA, CE_Pin);

	PinReset(GPIOA, RST_Pin);
	//задержка (10) ??мс
	for (delay = 0xFFFF; delay !=0; delay--)
		;

	PinSet(GPIOA, RST_Pin);

	Byte = 0b00100001;	//включение дисплея PV=0, расширенные команды H=1
	SendCom(Byte);
	Byte = 0b00010011;	// смещение напряжения (Bias)
	SendCom(Byte);
	Byte = 0b00000100;  //температурная коррекция 0
	SendCom(Byte);
//	Byte = 0b11000000;	//вкл. ген. повыш. напряжения 72
	Byte = 0b11000000; //
	SendCom(Byte);
	Byte = 0b00100000;	//обычные команды
	SendCom(Byte);
	Byte = 0b00001100;	//графический нормальный режим
	SendCom(Byte);

}

void PinSet(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void PinReset(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

void SendBitSet(void)
{
	PinReset(GPIOA, CLK_Pin);
	PinSet(GPIOA, DIN_Pin);
	PinSet(GPIOA, CLK_Pin);
}

void SendBitClr(void)
{
	PinReset(GPIOA, CLK_Pin);
	PinReset(GPIOA, DIN_Pin);
	PinSet(GPIOA, CLK_Pin);
}

void SendByte(uint8_t Byte)
{
	uint8_t i;
	PinReset(GPIOA, CE_Pin);
	for (i=7; i<=7; i--)
	{
		if ( Byte&(1<<i) )
			SendBitSet();
		else
			SendBitClr();
	}
	PinSet(GPIOA, CE_Pin);
}

void SendData(uint8_t Data)
{
	PinSet(GPIOA, DC_Pin);
	SendByte(Data);
}

void SendCom(uint8_t Command)
{
	PinReset(GPIOA, DC_Pin);
	SendByte(Command);
}

void XadressLCD(void)
{
	SendCom(0b10000000 + x_adr);
}

void YadressLCD(void)
{
	SendCom(0b01000000 + y_adr);
}

void LCD_RAM_Clr(void)
{
	uint8_t i, byte;

	byte = 0;
	for (i=0; i<252; i++)
		SendData(byte);
	x_adr = 0;
	y_adr = 3;
	XadressLCD();
	YadressLCD();
	for (i=0; i<252; i++)
		SendData(byte);
}

void PrintFrame(void)
{
	uint8_t i;
	uint8_t byte;
	/*uint16_t delay;*/

	x_adr = 0;
	y_adr = 3;
	XadressLCD();
	YadressLCD();
	for (i=0; i<84; i++)
	{
		byte = 1;
		SendData(byte);
	}
	x_adr = 0;
	y_adr = 5;
	XadressLCD();
	YadressLCD();
	for (i=0; i<84; i++)
	{
		byte = 0b10000000;
		SendData(byte);
	}

	x_adr = 0;
	y_adr = 0;
	XadressLCD();
	YadressLCD();
	for (i=0; i<84; i++)
	{
		byte = 1;
		SendData(byte);
	}

	x_adr = 0;
	y_adr = 2;
	XadressLCD();
	YadressLCD();
	for (i=0; i<84; i++)
	{
		byte = 0b10000000;
		SendData(byte);
	}

	byte = 0b00100010;	//vertical adressation
	SendCom(byte);

	x_adr = 0;
	y_adr = 0;
	XadressLCD();
	YadressLCD();
	for (i=0; i<6; i++)
	{
		byte = 0xFF;
		SendData(byte);
	}

//	for (delay = 0xFFFF; delay !=0; delay--)
//			;

	x_adr = 83;
	y_adr = 0;
	XadressLCD();
	YadressLCD();
	for (i=0; i<6; i++)
	{
		byte = 0xFF;
		SendData(byte);
	}

//	for (delay = 0xFFFF; delay !=0; delay--)
//			;
	x_adr = 64;
	y_adr = 0;
	XadressLCD();
	YadressLCD();
	for (i=0; i<6; i++)
	{
		byte = 0xFF;
		SendData(byte);
	}

//	for (delay = 0xFFFF; delay !=0; delay--)
//			;

	byte = 0b00100000;	//horisontal adressation back
	SendCom(byte);
}

void PrintFont (uint8_t symbol)
{
	uint8_t i;

	for(i=0; i<5; i++)
		SendData( font [symbol][i] );

	x_adr += 6;
	XadressLCD();
}

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
	PinReset(GPIOA, GPIO_Pin);
	HAL_Delay(2);				/*delay 2ms*/
	PinAIn(GPIO_Pin);

	while( HAL_GPIO_ReadPin(GPIOA, GPIO_Pin)==GPIO_PIN_SET );
	while( HAL_GPIO_ReadPin(GPIOA, GPIO_Pin)==GPIO_PIN_RESET );
	while( HAL_GPIO_ReadPin(GPIOA, GPIO_Pin)==GPIO_PIN_SET );


}

void ReadDHT(uint16_t GPIO_Pin)
{
	uint8_t i, j;

	StartDHT(GPIO_Pin);

	for(i=0; i<5; i++)
	{
		for(j=7; j<=7; j--)
		{
			TIM2->CNT = 0;
			while( HAL_GPIO_ReadPin(GPIOA, GPIO_Pin)==GPIO_PIN_RESET );
			TIM2->CR1 = 1;
			while( HAL_GPIO_ReadPin(GPIOA, GPIO_Pin)==GPIO_PIN_SET );
			TIM2->CR1 = 0;

			if( (TIM2->CNT)>MICROSECONDS50 )	/*75us: data bit=1*/
				dataDHT[i] |= 1<<j;				/*25us: data bit=0*/
			else
				dataDHT[i] &= ~(1<<j);
		}
	}


	/*MAYBE LATER SHOULD  ADD CHECKING DATA FROM DHT BY CRC*/

}

void PrintRHTmprFont(uint16_t rh_or_tmpr)
{
	uint8_t tens, units, tenth;

	tenth = rh_or_tmpr%10;
	rh_or_tmpr /= 10;
	units = rh_or_tmpr%10;
	tens = rh_or_tmpr/10;

	PrintFont(tens);
	PrintFont(units);
	PrintFont(tenth);
}

void NextLine(void)
{
	x_adr = 0;
	XadressLCD();
	y_adr += 1;
	YadressLCD();
}

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
		temp = 0;
	else
		temp = 3;

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


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
	  PinReset(LEDB12_GPIO_Port, LEDB12_Pin);
  }
  /* USER CODE END Error_Handler_Debug */ 
}



#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
