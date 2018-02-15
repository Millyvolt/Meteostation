/*
 * nokia5110.c
 *
 *  Created on: 15 февр. 2018 г.
 *      Author: Vova
 */

#include "stm32f1xx_hal.h"		/*if not include this - doesnt recognise uint8_t type*/


extern uint8_t x_adr, y_adr;

void SendBitSet(void)
{
	/*PinReset(GPIOA, CLK_Pin);*/
	HAL_GPIO_WritePin(GPIOA, CLK_Pin, GPIO_PIN_RESET);
	/*PinSet(GPIOA, DIN_Pin);*/
	HAL_GPIO_WritePin(GPIOA, DIN_Pin, GPIO_PIN_SET);
	/*PinSet(GPIOA, CLK_Pin);*/
	HAL_GPIO_WritePin(GPIOA, CLK_Pin, GPIO_PIN_SET);
}
void SendBitClr(void)
{
	/*PinReset(GPIOA, CLK_Pin);*/
	HAL_GPIO_WritePin(GPIOA, CLK_Pin, GPIO_PIN_RESET);
	/*PinReset(GPIOA, DIN_Pin);*/
	HAL_GPIO_WritePin(GPIOA, DIN_Pin, GPIO_PIN_RESET);
	/*PinSet(GPIOA, CLK_Pin);*/
	HAL_GPIO_WritePin(GPIOA, CLK_Pin, GPIO_PIN_SET);
}
void SendByte(uint8_t Byte)
{
	uint8_t i;
	/*PinReset(GPIOA, CE_Pin);*/
	HAL_GPIO_WritePin(GPIOA, CE_Pin, GPIO_PIN_RESET);
	for (i=7; i<=7; i--)
	{
		if ( Byte&(1<<i) )
			SendBitSet();
		else
			SendBitClr();
	}
	/*PinSet(GPIOA, CE_Pin);*/
	HAL_GPIO_WritePin(GPIOA, CE_Pin, GPIO_PIN_SET);
}
void SendData(uint8_t Data)
{
	/*PinSet(GPIOA, DC_Pin);*/
	HAL_GPIO_WritePin(GPIOA, DC_Pin, GPIO_PIN_SET);
	SendByte(Data);
}
void SendCom(uint8_t Command)
{
	/*PinReset(GPIOA, DC_Pin);*/
	HAL_GPIO_WritePin(GPIOA, DC_Pin, GPIO_PIN_RESET);
	SendByte(Command);
}

void LCD_Init(void)
{
	uint8_t Byte;
	uint16_t delay;

	/*PinSet(GPIOA, CE_Pin);*/
	HAL_GPIO_WritePin(GPIOA, CE_Pin, GPIO_PIN_SET);

	/*PinReset(GPIOA, RST_Pin);*/
	HAL_GPIO_WritePin(GPIOA, RST_Pin, GPIO_PIN_RESET);
	//задержка (10) ??мс
	for (delay = 0xFFFF; delay !=0; delay--)
		;

	/*PinSet(GPIOA, RST_Pin);*/
	HAL_GPIO_WritePin(GPIOA, RST_Pin, GPIO_PIN_SET);

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

void XadressLCD(void)
{
	SendCom(0b10000000 + x_adr);
}
void YadressLCD(void)
{
	SendCom(0b01000000 + y_adr);
}
void SetXY(uint8_t x, uint8_t y)
{
	  x_adr = x;
	  XadressLCD();
	  y_adr = y;
	  YadressLCD();
}

void LCD_RAM_Clr(void)
{
	uint8_t i;

	x_adr = 0;
	XadressLCD();
	y_adr = 0;
	YadressLCD();

	for (i=0; i<252; i++)
		SendData(0);
	x_adr = 0;
	y_adr = 3;
	XadressLCD();
	YadressLCD();
	for (i=0; i<252; i++)
		SendData(0);
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
void NextLine(void)
{
	x_adr = 0;
	XadressLCD();
	y_adr += 1;
	YadressLCD();
}
