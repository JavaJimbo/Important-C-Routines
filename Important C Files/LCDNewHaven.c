/********************************************************************
* FileName:        LCDNewHaven.c
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
#include "LCDNewHaven.h"
#include "Delay.h"

//*****************************************************************************
//                            CONSTANT DEFINITION
//*****************************************************************************

#define	LCD_STROBE {LCD_EN=1; DelayMs(1); LCD_EN=0;}


void WaitLCD(void){
	DelayMs(4);
}

/*
* Function  		WriteNibble
*
* @brief         	This function writes the specified nibble to the LCD and
*                  	notifies the (LCD) controller whether it has to interpret
*                  	the nibble as data or command.
*
* @param	        CommandFlag=TRUE for command, FALSE to write character
*                   	
*
* @param	        byte command or character byte
*
*/
/*******************************************************************/

void WriteNibble(unsigned char CommandFlag, unsigned char byte){
    if (CommandFlag) LCD_RS = 0;
    else LCD_RS = 1;    
    PORTE = (byte & 0x0F);          				// Combine & write back to the data lines
    LCD_STROBE
    DelayUs(40);
    DelayUs(40);
    DelayUs(40);
    DelayUs(40);
}

/********************************************************************
* Function:         WriteByte
* 
*
* @brief          	This function writes the specified Byte to the LCD and
*                   notifies the (LCD) controller whether it has to interpret
*                   the Byte as data or command. 
*
* @param	        CommandFlag		This flag specifies whether the data to be written to
*                   		the LCD is a command or data to be displayed.
*
* @param	        byte		Actual data or command to be written to the LCD controller.
*
* @note    			This routine is meant to be used from within this module only.
*/
/*******************************************************************/ 

void WriteByte(unsigned char CommandFlag, unsigned char byte){
unsigned char ch;
    ch = byte;
    ch = (ch >> 4) & 0x0F;
    WriteNibble(CommandFlag, ch);            // Output the high nibble to the LCD
    ch = byte;
    WriteNibble(CommandFlag, ch);                 // Now send the low nibble
}

/********************************************************************
* Function:         LCDInit
*
* @brief          	This routine is called once at start up to initialize the
*                   MCU hardware for proper LCD operation.
*
* @note    			Should be called at system start up only.
*/                                                                          
/*******************************************************************/


// initialise the LCD - put into 4 bit mode
void LCDInit(void){
	LCD_RS = 0;	// write control bytes
	DelayMs(15);	// power on delay
	DelayMs(15);	// power on delay
	DelayMs(15);	// power on delay
	DelayMs(15);	// power on delay
	PORTE = 0x03;	// attention!
	LCD_STROBE;
	DelayMs(5);
	DelayMs(5);
	DelayMs(5);
	DelayMs(5);
	LCD_STROBE;
	DelayUs(100);
	DelayUs(100);
	DelayUs(100);
	DelayUs(100);
	LCD_STROBE;
	DelayMs(5);
	DelayMs(5);
	DelayMs(5);
	DelayMs(5);
	PORTE = 0x02;	// set 4 bit mode
	LCD_STROBE;
	DelayUs(40);
	DelayUs(40);
	DelayUs(40);
	DelayUs(40);
	WriteByte(TRUE, 0x28);	// 4 bit mode, 1/16 duty, 5x8 font
	WriteByte(TRUE, 0x10);	// set cursor
	WriteByte(TRUE, 0x0F);	// Display ON; Blinking cursor
	WriteByte(TRUE, 0x06);	// entry mode
}


/********************************************************************
* Function:         LCDClear
* 
* PreCondition: 	None
*
* @brief         	This function is called to wipe the LCD display out.
*
* @note    			None.
*/  
/*******************************************************************/

void LCDClear(void){
  WriteByte(TRUE,0x01);                       // Send clear display command
  WaitLCD();                                  // Wait until command is finished
}

/********************************************************************
* Function:         LCDGoto
*
* @brief          	This function positions the cursor at the specified Line
*                   and column.
*
* @param	        Pos		Column (0 to 15) the cursor should be positioned at.
*
* @param	        Ln		Line (0 or 1) the cursor should be positioned at.
*
* @note    			0 <= Pos <= 15               
* @note				0 <= Ln <= 1
*/                                                                    
/*******************************************************************/

void LCDGoto(unsigned char Pos,  unsigned char Ln){
	if (Ln==0) WriteByte(TRUE, 0x80|Pos);
	else WriteByte(TRUE, 0xC0|Pos);  	
	WaitLCD();                                      			 // Wait for the LCD to finish
}

/********************************************************************
* Function:         LCDPutChar
*
* @brief            This function displays the specified ASCII character at
*                   current position on the LCD
*
* @param	    ch - ASCII data character to be displayed.
*
*/ 
/*******************************************************************/

void LCDPutChar(unsigned char ch){
  WriteByte(FALSE, ch);              // Go output the character to the display
  WaitLCD();                          // Wait until it's finished
}

/********************************************************************
* Function:         LCDPutByte
*
* @brief            This function displays the specified binary value at
*                   current position on the LCD. It converts the binary
*                   value into displayable ASCII characters.
*
* @param	    Val		Binary data byte to be displayed
*
* @note    			In the present configuration, this routine displays a
*                   2-digit value, by prefilling with '0' any value lower
*                   than 10.
*/
/*******************************************************************/

void LCDPutByte(unsigned char Val){
  LCDPutChar(Val/10+'0');                   // Output the high digit
  LCDPutChar(Val % 10+'0');                 // Output low
}

/********************************************************************
* Function:         LCDWriteStr
*
* @brief          	This function displays the specified string starting from
*                   current position on the LCD.
*
* @param	        Str		IF 0; Terminated string to be displayed.
*
* @note    			None
*/
/*******************************************************************/

void LCDWriteStr(const char  *Str){
  unsigned char i = 0;                                     // Char index buffer

  while (Str[i])                                   // While string not finished
    LCDPutChar(Str[i++]);                          // Go display current char
}

#define MAX_LCD_STRING 16
void LCDWriteArray (unsigned char  *arrayPtr){
unsigned char ch, i=0;                                     // Char index buffer

	i=0;
	for(i=0; i<MAX_LCD_STRING; i++){
		ch=arrayPtr[i];
		if(ch)LCDPutChar(ch);                          // Go display current char
		else break;
	}
}

