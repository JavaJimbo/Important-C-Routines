/********************************************************************
* FileName:        LCDNewHaven.h
* For New Haven LCD display #NHD-0216K1Z-NSW-BBW-L
* Processor: 	   PIC 32MX340F512H Olimex PIC-32MX board
* Compiler: 	   Microchip XC32
*
* Change History:
* 08-07-2014 JBS Modified for LCD timer project
* PORTE RE0,RE1,RE2,RE3 connected to LCD D4, D5, D6, D7
* LCD_EN: RD2, LCD_RS: RD1
*
********************************************************************/

#include "plib.h"

#define TRUE  	1                           
#define FALSE 	0			

#define ENABLE  1                       
#define DISABLE 0			

#define SET  	1                       
#define CLEAR	0			


#define LCD_EN      PORTDbits.RD2       
#define LCD_RS      PORTDbits.RD1       

void WaitLCD(void);
void WriteNibble(unsigned char CommandFlag, unsigned char byte);
void WriteByte(unsigned char CommandFlag, unsigned char byte);
void LCDInit(void);
void LCDClear(void);
void LCDGoto(unsigned char Pos,  unsigned char Ln);
void LCDPutChar(unsigned char Data);
void LCDPutByte(unsigned char Val);
void LCDWriteStr(const char  *Str);
void LCDWriteArray (unsigned char  *arrayPtr);

