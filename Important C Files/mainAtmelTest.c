/***********************************************************************************************************
 * ATMEL TEST
 * 
 * Simple project for verifying ATMEL AT45DB161 memory functions on LED MATRIX CONTROLLER BOARD REV 1.0
 * Created 6-14-17
 * Cleaned up and tested all routines. Fixed initAtmelSPI() issues
 ************************************************************************************************************/

#ifndef MAIN_C
#define MAIN_C

#define true TRUE
#define false FALSE

/** INCLUDES *******************************************************/
// #include <XC.h>
#include <plib.h>

#include "usb.h"
#include "HardwareProfile.h"
#include "Delay.h"
#include "AT45DB161.h"
#include "GenericTypeDefs.h"
#include "Compiler.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/** CONFIGURATION **************************************************/

#pragma config UPLLEN   = ON        // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier $$$$
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select


/** PRIVATE PROTOTYPES *********************************************/
static void InitializeSystem(void);
void UserInit(void);

#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR


#define MAXBUFFER 128
unsigned char HOSTRxBuffer[MAXBUFFER];
unsigned char HOSTRxBufferFull = false;

int main(void) {
    short i = 0;
    //unsigned char outTest[] = "Scrunch munch lunch";
    short outTestLength;
    unsigned char AtmelRAM[PAGESIZE] = "Diddy wah diddy";
    unsigned char AtmelFetchRAM[PAGESIZE];

    outTestLength = strlen(AtmelRAM);
    
    UserInit();   
    DelayMs(10);
    printf("\r\rTESTING ATMEL");
    
    
    #define ATMEL_BUFFER 1
    short pageNum = 850;
    short byteAddress = 500;
        
    DelayMs(10);

    printf("\rInitializing SPI");
    initAtmelSPI();
    
    printf("\rErasing Sector #3");
    EraseFLASHsector(3);    
    
    printf("\rWriting page to Buffer %d:", ATMEL_BUFFER);
    WriteAtmelBuffer(ATMEL_BUFFER, AtmelRAM);
    printf("\rProgramming flash");
    ProgramFLASH(ATMEL_BUFFER, pageNum);
    
    
    printf("\rTransferring flash");
    TransferFLASH (ATMEL_BUFFER, pageNum);
    printf("\rReading Buffer %d:", ATMEL_BUFFER);    
    ReadAtmelBuffer(ATMEL_BUFFER, AtmelFetchRAM);
    printf(" %s", AtmelFetchRAM);
    
    //printf("\r\rSECTOR ERASE TEST: ");            
    //printf("\rErasing Sector #3");
    //EraseFLASHsector(3);
    
    //printf("\r\rTESTING ATMEL WITH DELAY");
    //printf("\rErasing FLASH");
    //EraseFLASHpage(pageNum);
    
    //printf("\rWriting to buffer");
    //WriteAtmelBytes(ATMEL_BUFFER, outTest, byteAddress , outTestLength);
    //printf("\rProgramming flash");
    //ProgramFLASH (ATMEL_BUFFER, pageNum);
    
    
    //printf("\rTransferring flash");
    //TransferFLASH (ATMEL_BUFFER, pageNum);
    //printf("\rReading RAM");
    //ReadAtmelBytes (ATMEL_BUFFER, AtmelRAM, byteAddress, outTestLength);
    
    //printf("\rData: ");
    //for(i = 0; i < outTestLength; i++) printf("%c", AtmelRAM[i]);
    while(1);
    
    
    //unsigned short inData = 65432;
    //printf("\r\rTEST #0 Storing %d at address %d page %d", inData, byteAddress, pageNum);    
    //storeShortToAtmel(pageNum, byteAddress, inData);
    
    printf("\rFetching integer: ");
    unsigned short outData = fetchShortFromAtmel(pageNum, byteAddress);    
    printf("%d", outData);
    
    while(1);
    
}    



void UserInit(void) {

    // Set up main UART
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
#define SYS_FREQ 80000000
    UARTSetDataRate(HOSTuart, SYS_FREQ, 57600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_DISABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);

    //PORTSetPinsDigitalOut(IOPORT_G, BIT_0);
    //PORTClearBits(IOPORT_G, BIT_0);  // RS485 control pin should be set to receive
    
    //PORTSetPinsDigitalIn(IOPORT_C, BIT_14 | BIT_13);
    //PORTSetPinsDigitalOut(IOPORT_B, BIT_2);
    //PORTSetBits(IOPORT_B, BIT_2);
    
    // Set up Atmel CS output:
    PORTSetPinsDigitalOut(IOPORT_C, BIT_3);
    PORTSetBits(IOPORT_C, BIT_3);   // Set high to deselect
    
    // Set up Atmel WR protect:
    PORTSetPinsDigitalOut(IOPORT_E, BIT_8);
    PORTSetBits(IOPORT_E, BIT_8);    // Set high to enable writes

    // Turn on the interrupts
    // INTEnableSystemMultiVectoredInt();
    
}//end UserInit

/** EOF main.c *************************************************/
#endif
