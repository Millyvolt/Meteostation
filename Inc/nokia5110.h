/*
 * nokia5110.h
 *
 *  Created on: 15 февр. 2018 г.
 *      Author: Vova
 */

#ifndef NOKIA5110_H_
#define NOKIA5110_H_


void LCD_Init(void);
void SendBitSet(void);
void SendBitClr(void);
void SendByte(uint8_t Byte);
void SendData(uint8_t Data);
void SendCom(uint8_t Command);
void LCD_RAM_Clr(void);
void PrintFrame(void);
void NextLine(void);
void XadressLCD(void);
void YadressLCD(void);
void SetXY(uint8_t x, uint8_t y);


#endif /* NOKIA5110_H_ */
