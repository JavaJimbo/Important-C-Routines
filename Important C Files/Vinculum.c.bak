/**************************************************************************************
 *
 *                  DISPATCHER for UB32 Board
 *
 **************************************************************************************
 * FileName:      Main.c		 
 *
 * 5-1-14  Recompiled using XC32 - UART 2 works
 * 5-24-14 Removed LINX512 - added LINX
 **************************************************************************************/
#include <string.h>
#include <plib.h>
#include "Atmel642.h"
#include "HardwareProfile.h"

/*
#include "plib.h"
#include <stdlib.h> 
#include <stdio.h> 
#include "Delay.h"
#include <string.h>    
#include <ctype.h>

#include "HardwareProfileDispatcher.h"
#include "p32mx795f512l.h"
#include "HardwareProfile.h"
*/

#define false 0
#define true !false
#define FALSE false
#define TRUE true


#define ESCAPE 27
#define ENTER 13

#define LINXUart		UART1
#define HOSTuart	UART2
#define XBEEUart	UART3
#define MIDIUart	UART4

#define ENDFRAME 0x99
#define STARTFRAME 0x00

#define STANDBY 0
#define ENCODE 	1
#define SEND 	2
#define HOSTTOXBEE	3
#define QUERY	4


// Timeouts are in milliseconds I think
#define MIDI_TIMEOUT 	8  // 
#define REMOTE_TIMEOUT  64  // 
#define REMOTE_PAUSE    64 // 
#define HOST_TIMEOUT    8  //
#define UART5_TIMEOUT   8

#define INC_MIDIRxHead(); MIDIRxHead++; if (MIDIRxHead>=MAXBUFFER) MIDIRxHead=0;		
#define INC_MIDIRxTail(); MIDIRxTail++; if (MIDIRxTail>=MAXBUFFER)	MIDIRxTail=0;
#define INC_MIDITxHead(); MIDITxHead++; if (MIDITxHead>=MAXBUFFER) MIDITxHead=0;			
#define INC_MIDITxTail(); MIDITxTail++; if (MIDITxTail>=MAXBUFFER) MIDITxTail=0;

#define INC_HOSTRxHead(); HOSTRxHead++; if (HOSTRxHead>=MAXBUFFER)	HOSTRxHead=0;	
#define DEC_HOSTRxHead(); if (HOSTRxHead>0)	HOSTRxHead--; else HOSTRxHead=MAXBUFFER-1;
	
#define INC_HOSTRxTail(); HOSTRxTail++; if (HOSTRxTail>=MAXBUFFER)	HOSTRxTail=0;			
#define INC_HOSTTxHead(); HOSTTxHead++; if (HOSTTxHead>=MAXBUFFER)	HOSTTxHead=0;	
#define INC_HOSTTxTail(); HOSTTxTail++; if (HOSTTxTail>=MAXBUFFER)	HOSTTxTail=0;				
	
#define INC_XBEERxHead(); XBEERxHead++; if (XBEERxHead>=MAXBUFFER)	XBEERxHead=0;			
#define INC_XBEERxTail(); XBEERxTail++; if (XBEERxTail>=MAXBUFFER)	XBEERxTail=0;	
#define INC_XBEETxHead(); XBEETxHead++; if (XBEETxHead>=MAXBUFFER)	XBEETxHead=0;			
#define INC_XBEETxTail(); XBEETxTail++; if (XBEETxTail>=MAXBUFFER)	XBEETxTail=0;

#define INC_UART5RxHead(); UART5RxHead++; if (UART5RxHead>=MAXBUFFER)	UART5RxHead=0;
#define INC_UART5RxTail(); UART5RxTail++; if (UART5RxTail>=MAXBUFFER)	UART5RxTail=0;
	
	
//#define INC_LINXTxHead(); LINXTxHead++; if (LINXTxHead>=MAXBUFFER) LINXTxHead=0;
//#define INC_LINXTxTail(); LINXTxTail++; if (LINXTxTail>=MAXBUFFER) LINXTxTail=0;
	
#define INC_LINXRxHead(); LINXRxHead++; if (LINXRxHead>=MAXBUFFER) LINXRxHead=0;
#define INC_LINXRxTail(); LINXRxTail++; if (LINXRxTail>=MAXBUFFER) LINXRxTail=0;

const unsigned char SETSYS=0xE0;
const unsigned char SETBOT=0x80;
 
const unsigned char LINXBOT=0x05;
const unsigned char MIDIBOT=0x06;

const unsigned char LINX_CONTROL=	0xC4;
const unsigned char MIDI_CONTROL=	0xC5;

unsigned char LINXflag=false;
unsigned char botNumber=0;

unsigned char SYSsubCommand=0;
unsigned char promptFlag=false;

#define	STX	2		// First byte of every packet
#define ETX 3		// Last byte of every packet
#define DLE 6		// Escape chracter preceding data bytes to avoid confusing them with STX and ETX
#define PLUS 43 	

#define	LINX_STANDBY	0
#define	LINX_BREAK 	1
#define LINX_MARK 	2
#define LINX_START 	3
#define LINX_DATA	4
#define LINXLENGTH 	16
#define TICK		10000
	
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_8

#define MAXBUFFER 128
unsigned char HOSTTxBuffer[MAXBUFFER+1];
unsigned char HOSTRxBuffer[MAXBUFFER+1];
unsigned char HOSTRxHead=0;
unsigned char HOSTRxTail=0;
unsigned char HOSTTxHead=0;
unsigned char HOSTTxTail=0;
unsigned char HOSTtimeout=0;
unsigned char UART2flag=false;
#define MAXCOMMAND 32
unsigned char commandBuffer[MAXCOMMAND];




unsigned char UART5TxBuffer[MAXBUFFER+1];
unsigned char UART5RxBuffer[MAXBUFFER+1];
unsigned char UART5RxHead=0;
unsigned char UART5RxTail=0;
unsigned char UART5timeout=0;



unsigned char MIDITxBuffer[MAXBUFFER+1]; // ={0xB0,0x00,0x7F,0x20,0x00,0xC0,0x30};
unsigned char MIDIRxBuffer[MAXBUFFER+1];
unsigned char MIDIRxHead=0;
unsigned char MIDIRxTail=0;
unsigned char MIDITxHead=0; 
unsigned char MIDITxTail=0;
unsigned char HOSTstate=STANDBY;
unsigned char MIDIdata=false;
unsigned char LINXdata=false;
unsigned int  MIDItimeout=0;

unsigned char XBEETxBuffer[MAXBUFFER];
unsigned char XBEERxBuffer[MAXBUFFER];
unsigned char XBEETxTail=0;
unsigned char XBEETxHead=0;
unsigned char XBEERxTail=0;
unsigned char XBEERxHead=0;
unsigned char XBEEtimeout=0;

static unsigned char LINXstate=LINX_STANDBY;

unsigned char LINXRxBuffer[MAXBUFFER+1];
unsigned char LINXTxBuffer[MAXBUFFER+1];

unsigned char UARTflag=FALSE;

unsigned int SYS_LEDtimeout=0;
#define SysLEDOn()  PORTSetBits(IOPORT_B, BIT_10)
#define SysLEDOff() PORTClearBits(IOPORT_B, BIT_10)

#define SetRS485()  PORTSetBits(IOPORT_B, BIT_1)
#define ClearRS485() PORTClearBits(IOPORT_B, BIT_1)



unsigned char 	HOSTcounter=0;
unsigned int 	HOST_LEDtimeout=0;
unsigned char 	HOST_LEDflag=false;
unsigned char 	LEDflash=false;
#define HOST_LEDOn()  PORTSetBits(IOPORT_G, BIT_9)
#define HOST_LEDOff() PORTClearBits(IOPORT_G, BIT_9)

void XBEEComPacket(unsigned char command, unsigned char subcommand);

#define COMMAND_LINX_LOCAL 1

#define QUERY_COMMAND 	 0xD9
#define QUERY_SUBCOMMAND 0x12

#define MAXREMOTES 3
#define FIRST_REMOTE 1
unsigned char remoteIndex=0;
unsigned char XBEEremote[MAXREMOTES]={0xC1, 0xC2, 0xC3};
#define NUMREMOTES 2

unsigned char HOSTdecode();
unsigned char command=0;

void XBEEdecode();

#define NUM_SERVOS 6
struct InStruct {
	unsigned char pot[NUM_SERVOS];	
} Remote[MAXREMOTES];

unsigned int testTimer=0;
unsigned char testChar;
unsigned char commandChar=0;
int sendReceiveVinculum (const char *ptr);



int main(void){
int	UART1baudrate, UART2baudrate, UART3baudrate, UART4baudrate, sysClockRate, strSize;
unsigned char ch;
unsigned short	i, j;
#define MAXSTRING 64
unsigned char MIDIstring[MAXSTRING];
unsigned char scopeFlag=false;
unsigned char TxString[128];
unsigned char RS485enable=FALSE;
int linxCounter=0;

        //if (U2STAbits.FERR)
        //    U2STAbits.OERR = 1;
/*
   		if (1==U1STAbits.OERR)				// If LINX overrun occurs, clear overrun flag
				U1STAbits.OERR=0;

		if (U1STAbits.FERR==1){
   			dummy=UARTGetDataByte(LINXUart);
 			dummyCounter=0;
			LINXRxPtr=0;
		}
*/

/*
	// Clear input data struct
	for(i=0;i<MAXREMOTES;i++){
		for(j=0;j<NUM_SERVOS;j++)
			Remote[i].pot[j]=0;
	}

	// Enable optimal performance
	sysClockRate=SYSTEMConfigPerformance(GetSystemClock());
	mOSCSetPBDIV(OSC_PB_DIV_1);					// Use 1:1 CPU Core:Peripheral clocks		

	// Set up Port B outputs:
	PORTSetPinsDigitalOut(IOPORT_B, BIT_0 | BIT_1 | BIT_4);
	PORTClearBits(IOPORT_B, BIT_0 | BIT_1 | BIT_4);	

	// Set up Test out:
	PORTSetPinsDigitalOut(IOPORT_B, BIT_10);  	// $$$$
	PORTClearBits(IOPORT_B, BIT_10);				// $$$$
	
	// Set up MIDITest out:
	PORTSetPinsDigitalOut(IOPORT_G, BIT_9);  	// 
	PORTClearBits(IOPORT_D, BIT_9);				// 
	
	// Set up Atmel BUSY input:
	PORTSetPinsDigitalIn(IOPORT_E, BIT_8);


	// Set up Atmel chip selects:	
	PORTSetPinsDigitalOut(IOPORT_D, BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_5);	// Make Atmel chip selects outputs
	PORTSetBits(IOPORT_D, BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_5);			// Deselect them: make them all high.

	// Set up SPI port #1:
	SpiChnOpen(SPI_CHANNEL1, SPI_OPEN_MODE8|SPI_OPEN_CKE_REV|SPI_OPEN_ON|SPI_OPEN_MSTEN, 80);	
	
	// Enable multi-vectored interrupts
	INTEnableSystemMultiVectoredInt();
	
	// Set up UART #1 for LINX512 @ 25000 baud
	UARTConfigure(LINXUart, UART_ENABLE_HIGH_SPEED|UART_ENABLE_PINS_TX_RX_ONLY);
	UARTSetFifoMode(LINXUart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
	UARTSetLineControl(LINXUart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
	UART1baudrate=UARTSetDataRate(LINXUart, GetPeripheralClock(), 9600);
        UARTEnable(LINXUart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
   	
	// Configure UART #1 interrupts
	INTEnable(INT_SOURCE_UART_RX(LINXUart), INT_ENABLED);
	INTEnable (INT_U1TX, INT_DISABLED);
   	INTSetVectorPriority(INT_VECTOR_UART(LINXUart), INT_PRIORITY_LEVEL_1);
   	INTSetVectorSubPriority(INT_VECTOR_UART(LINXUart), INT_SUB_PRIORITY_LEVEL_0);
	




	// Set up UART #3 for XBEE at 57600 baud	
	UARTConfigure(XBEEUart, UART_ENABLE_HIGH_SPEED|UART_ENABLE_PINS_TX_RX_ONLY);      	
	UARTSetFifoMode(XBEEUart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);        
	UARTSetLineControl(XBEEUart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);		
	UART2baudrate=UARTSetDataRate(XBEEUart, GetPeripheralClock(), 57600);               
        UARTEnable(XBEEUart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));


	// Configure UART #3 Interrupts
	INTEnable (INT_U3TX, INT_DISABLED);
	INTEnable (INT_SOURCE_UART_RX(XBEEUart), INT_ENABLED);
   	INTSetVectorPriority(INT_VECTOR_UART(XBEEUart), INT_PRIORITY_LEVEL_2);
   	INTSetVectorSubPriority(INT_VECTOR_UART(XBEEUart), INT_SUB_PRIORITY_LEVEL_0);


	
	// Set up UART #5 for 57600 baud
	UARTConfigure(UART5, UART_ENABLE_HIGH_SPEED|UART_ENABLE_PINS_TX_RX_ONLY);
	UARTSetFifoMode(UART5, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
	UARTSetLineControl(UART5, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
	UART2baudrate=UARTSetDataRate(UART5, GetPeripheralClock(), 9600);
        UARTEnable(UART5, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

	// Configure UART #5 Interrupts
	INTEnable (INT_U5TX, INT_DISABLED);
	INTEnable (INT_SOURCE_UART_RX(UART5), INT_ENABLED);
   	INTSetVectorPriority(INT_VECTOR_UART(UART5), INT_PRIORITY_LEVEL_2);
   	INTSetVectorSubPriority(INT_VECTOR_UART(UART5), INT_SUB_PRIORITY_LEVEL_0);


   	
	// Set up UART #4 for MIDI @ 31250 baud
        UARTConfigure(MIDIUart, UART_ENABLE_PINS_TX_RX_ONLY);
//	UARTConfigure(MIDIUart, UART_ENABLE_HIGH_SPEED|UART_ENABLE_PINS_TX_RX_ONLY);      	
	UARTSetFifoMode(MIDIUart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);        
	UARTSetLineControl(MIDIUart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);		
	UART4baudrate=UARTSetDataRate(MIDIUart, GetPeripheralClock(), 31250);               
        UARTEnable(MIDIUart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

	// Configure UART #4 Interrupts
	INTEnable (INT_U4TX, INT_DISABLED);	
	INTEnable (INT_SOURCE_UART_RX(MIDIUart), INT_ENABLED);
   	INTSetVectorPriority(INT_VECTOR_UART(MIDIUart), INT_PRIORITY_LEVEL_3);
   	INTSetVectorSubPriority(INT_VECTOR_UART(MIDIUart), INT_SUB_PRIORITY_LEVEL_0);
*/

    // Set up main UART
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED|UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(HOSTuart, GetPeripheralClock(), 57600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable (INT_U2TX, INT_DISABLED);
    INTEnable (INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);

   SYSTEMConfigPerformance(GetSystemClock());
   mOSCSetPBDIV(OSC_PB_DIV_2);

    // Enable multi-vectored interrupts
    INTEnableSystemMultiVectoredInt();

    // Enable interrupts
   INTEnableInterrupts();

   	
/*
   	// Configure Timer 1 using internal clock, 1:8 prescale, Postscale = 10,000 for 1 millisecond interrupts
   	OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_8, TICK);     	
   
	// Set up the timer interrupt with a priority of 2
	ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
    
	// Set up Timer 2 for 1 millisecond roll-over rate
 	OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_32, 328);
 	
 	// set up the core timer interrupt with a priority of 5 and zero sub-priority     
 	ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_5);     
	
   	// Enable interrupts
   	INTEnableInterrupts();     	   		
	DelayMs(100);	 
	HOST_LEDOff();  
*/
	printf ("\rTesting LINX at 9600 baud");
	while(1){
		if (commandChar){
			commandChar=0;
			printf("\rCommand received");
		}

                if (LINXflag){
                    LINXflag=false;
                    printf("\rFrame error #d detected", linxCounter++);
                }
	}

	printf("\rTesting Vinculum commands");
	while(1){

		if (UART5RxTail!=UART5RxHead){
			ch=UART5RxBuffer[UART5RxTail];
			INC_UART5RxTail();
			while (!UARTTransmitterIsReady(UART2));
    		UARTSendDataByte(UART2, ch);    					
		} 		
		
		if (commandChar=='E'){
			printf("\rSynch: ");
			sendReceiveVinculum("E\r");
			commandChar=0;
		}		

		if (commandChar=='F'){
			printf("\rVersion: ");
			sendReceiveVinculum("FWV\r");
			commandChar=0;
		}	
		
		if (commandChar=='X'){
			printf("\rEXTENDED mode: ");
			sendReceiveVinculum("ECS\r");
			commandChar=0;
		}				
		
		if (commandChar=='A'){
			printf("\rASCII mode: ");
			sendReceiveVinculum("IPA\r");
			commandChar=0;
		}				
		
		if (commandChar=='O'){
			printf("\rOPEN: ");
			sendReceiveVinculum("OPW Ricky.txt\r");
			commandChar=0;
		}				
		
		if (commandChar=='W'){
			printf("\rWRITE: ");
			sendReceiveVinculum("WRF 8\r12345678");
			commandChar=0;
		}				
					
		if (commandChar=='C'){
			printf("\rCLOSE: ");
			sendReceiveVinculum("CLF Ricky.txt\r");
			commandChar=0;
		}			
		
	}	
	return (0);
}
		


// XBEE interrupt handler - UART #3
// it is set at priority level 2
void __ISR(_UART_3_VECTOR, ipl2) IntUart3Handler(void){
unsigned char ch;

	if(INTGetFlag(INT_SOURCE_UART_RX(XBEEUart))){		
	   INTClearFlag(INT_SOURCE_UART_RX(XBEEUart));
		
		if (UARTReceivedDataIsAvailable(XBEEUart)){		
			ch=UARTGetDataByte(XBEEUart);					
			XBEERxBuffer[XBEERxHead]=ch;
			INC_XBEERxHead();			
			//XBEEtimeout=REMOTE_TIMEOUT;		
		}			
	}		
		
	// TX interrupts not used
	if (INTGetFlag(INT_SOURCE_UART_TX(XBEEUart)) ){
		INTClearFlag(INT_SOURCE_UART_TX(XBEEUart));	
		
		INTEnable (INT_U3TX, INT_DISABLED);			
	}
	
}


//	MIDI INTERRUPTS
// 	UART 4 interrupt handler for receiving and transmitting MIDI data
void __ISR(_UART_4_VECTOR, ipl3) IntUart4Handler(void){
unsigned char 	ch, dummy;
	
	// RX interrupts
	if(INTGetFlag(INT_SOURCE_UART_RX(MIDIUart))){
		// Clear the RX interrupt Flag
	   INTClearFlag(INT_SOURCE_UART_RX(MIDIUart));
	   
   		if (1==U4STAbits.OERR)	
				U4STAbits.OERR=0;										   
	   
		if (U4STAbits.FERR==1)	
   			dummy=UARTGetDataByte(MIDIUart); 																							
					
		if (UARTReceivedDataIsAvailable(MIDIUart)){			
			ch=UARTGetDataByte(MIDIUart);				
			if (ch!=0xFE & ch!=0xF8){		
				MIDItimeout=MIDI_TIMEOUT;		
				MIDIRxBuffer[MIDIRxHead]=ch;							
				INC_MIDIRxHead();		
			}				
		}			
	}	
}	





//	LINX INTERRUPTS
// 	UART 1 interrupt handler for receiving and transmitting LINX512 data
// 	Priority level 1
void __ISR(_UART1_VECTOR, ipl1) IntUart1Handler(void){
static unsigned short 	LINXTxPtr=0;
static unsigned short	LINXRxPtr=0;
unsigned char           ch, dummy;
	
	// RX interrupts
	if(INTGetFlag(INT_SOURCE_UART_RX(LINXUart))){
		// Clear the RX interrupt Flag
	   INTClearFlag(INT_SOURCE_UART_RX(LINXUart));
	   
   		if (1==U1STAbits.OERR)				// If LINX overrun occurs, clear overrun flag
				U1STAbits.OERR=0;										   
	   
		if (U1STAbits.FERR==1){
                    LINXflag=true;
   			dummy=UARTGetDataByte(LINXUart); 			
			LINXRxPtr=0;
		}
		else if (UARTReceivedDataIsAvailable(LINXUart)){
			ch=UARTGetDataByte(LINXUart);
                        //if (ch=='\r')LINXflag=true;
			if (LINXRxPtr<LINXLENGTH){
				LINXRxBuffer[LINXRxPtr]=ch;
				LINXRxPtr++;									
			}	
		}			
	}	

	// TX interrupts: 
	if (INTGetFlag(INT_SOURCE_UART_TX(LINXUart))){
		INTClearFlag(INT_SOURCE_UART_TX(LINXUart));
	}		
}


// This routine creates a command packet in the XBEE TxBuffer[]
void XBEEComPacket(unsigned char command, unsigned char subcommand){
unsigned char ch;
	
	XBEETxBuffer[XBEETxHead]=STX;
	INC_XBEETxHead();	
	
	ch=command;
	if(ch==STX||ch==ETX||ch==DLE||ch==PLUS){	// Insert escape charaacters as necessary
		XBEETxBuffer[XBEETxHead]=DLE;
		INC_XBEETxHead();						
	}	
	XBEETxBuffer[XBEETxHead]=ch;
	INC_XBEETxHead();	
	
	ch=subcommand;
	if(ch==STX||ch==ETX||ch==DLE||ch==PLUS){	// Insert escape charaacters as necessary
		XBEETxBuffer[XBEETxHead]=DLE;
		INC_XBEETxHead();						
	}	
	XBEETxBuffer[XBEETxHead]=ch;
	INC_XBEETxHead();	
	
	XBEETxBuffer[XBEETxHead]=ETX;
	INC_XBEETxHead();			
}	

#define BACKSPACE 0x08

// HOST Communication processed here
// UART 2 interrupt handler for RS232 diagnostics
// it is set at priority level 2
void __ISR(_UART2_VECTOR, ipl2) IntUart2Handler(void){
unsigned char ch;	


    if(INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))){
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));

        if (UARTReceivedDataIsAvailable(HOSTuart)){
            ch=toupper(UARTGetDataByte(HOSTuart));
            if (ch==BACKSPACE){
                while (!UARTTransmitterIsReady(HOSTuart));
				UARTSendDataByte(HOSTuart, ' ');
				while (!UARTTransmitterIsReady(HOSTuart));
				UARTSendDataByte(HOSTuart, BACKSPACE);
				DEC_HOSTRxHead();
            }
            else commandChar=ch;
            
        }
    }
	
    // TX interrupt: If Tx is true and
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart)) ){
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
        INTEnable (INT_U2TX, INT_DISABLED);
    }
}

#define START 		1
#define COMMAND 	2
#define SUBCOMMAND 	3
#define DATA 		4
#define ESCAPE_CHAR 5
#define END 		6

// HOST Communication processed here
unsigned char 	HOSTdecode(){
unsigned char 	escapeFlag=false;
unsigned char 	command=0;
unsigned char 	subcommand=0;
unsigned char 	LINXTxPtr=0;
unsigned char 	ch=0;
unsigned char  	state=0;	
unsigned char	endFlag=false;
		
		
	while (HOSTRxTail!=HOSTRxHead){ 
		ch=HOSTRxBuffer[HOSTRxTail];
		INC_HOSTRxTail();
			
    	if (escapeFlag==false){
    	    if (ch==DLE){
    	        escapeFlag=true;
    	        if (state>=SUBCOMMAND)
    	        	state=ESCAPE_CHAR;
			}        	        
    	    else if (ch==STX)        	        
    	        state=START;    	        				
    	    else if (ch==ETX)
				state=END;																										
    	   	else if (state==START){					
    	   		command=ch&0xF0;
    	   		botNumber=ch&0x0F;
    	   		if (botNumber==LINXBOT) // If HOST mode isn't enabled, don't accept LINX commands
    	   			botNumber=0;        	   		
    	   		state=COMMAND;
    	   		LINXTxPtr=0;
			}        	   		
			else if (state==COMMAND){					
				subcommand=ch;
				state=SUBCOMMAND;
			}	
			else if (state==SUBCOMMAND)				
				state=DATA;								
		}
		else {
			escapeFlag=false;
			if (state==ESCAPE_CHAR)
				state=DATA;
		}		
		
		if (state==DATA){				
			if(botNumber==MIDIBOT){
				MIDITxBuffer[MIDITxHead]=ch;
				INC_MIDITxHead();
			}
			else if(botNumber==LINXBOT){
				if (LINXTxPtr<LINXLENGTH){
					LINXTxBuffer[LINXTxPtr]=ch;
					LINXTxPtr++;
				}							
			}						
		}																		
		if (botNumber!=0 && botNumber!=MIDIBOT && botNumber!=LINXBOT){
			if (state==COMMAND){
				XBEETxBuffer[XBEETxHead]=STX;  
				INC_XBEETxHead();					
			}			
			XBEETxBuffer[XBEETxHead]=ch;  
			INC_XBEETxHead();	
		}	
		//else if (state==END && botNumber==1)				
		//	XBEEstate=HOST;		
			
		if (state==END){
			if(command==SETSYS){ 
				if(subcommand==STARTFRAME){  // If this is the beginning of the frame, 
					HOSTstate=ENCODE;		 // encode existing MIDI,LINX, and XBEE data to send to HOST
				}		
				else if(subcommand==ENDFRAME){
					endFlag=true;	 	// Now that HOST data is loaded into XBEE Tx buffer, start querying remotes.														
					LEDflash=true;
				}					
				else SYSsubCommand=subcommand; 
			}
			
			botNumber=0;
			state=0;
		}												
	}					
	return(endFlag);
}



// Incoming Data from XBEE is processed here
void XBEEdecode(){
unsigned char 	escapeFlag=false;
unsigned char 	command=0;
unsigned char 	ch=0;
unsigned char  	state=0;	
unsigned char   subCommand=0;
unsigned char   inDevice=0;
unsigned char   i,j;
		
	while (XBEERxTail!=XBEERxHead){ 
		ch=XBEERxBuffer[XBEERxTail];
		INC_XBEERxTail();
			
    	if (escapeFlag==false){
    	    if (ch==DLE){
    	        escapeFlag=true;
    	        if (state>=SUBCOMMAND)
    	        	state=ESCAPE_CHAR;
			}        	        
    	    else if (ch==STX)        	        
    	        state=START;    	        				
    	    else if (ch==ETX){ 					
				state=END;																		
			}					
    	   	else if (state==START){					
    	   		command=ch&0xF0;
    	   		inDevice=ch&0x0F;    	   		
    	   		if (inDevice>=MAXREMOTES)
    	   			inDevice=0;
    	   		state=COMMAND;   
    	   		j=0; 	   		
			}        	   		
			else if (state==COMMAND){									
				subCommand=ch;									
				state=SUBCOMMAND;
			}	
			else if (state==SUBCOMMAND)				
				state=DATA;								
		}
		else {
			escapeFlag=false;
			if (state==ESCAPE_CHAR)
				state=DATA;
		}		
		
		if (inDevice && state==DATA){
			i = inDevice-1;
			if (i<MAXREMOTES && j<NUM_SERVOS){
				Remote[i].pot[j]=ch;			
				j++;
			}				
		}																		
			
		if (state==END){
			command=0;
			subCommand=0;
			state=0;
			inDevice=0;
			i=0;
			j=0;											
		}			
	}					
}

void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void){   
	
	mT2ClearIntFlag();		// clear the interrupt flag			
	
	if (testTimer)
		testTimer--;
			
	
	if (XBEEtimeout)
		XBEEtimeout--;

        if (UART5timeout)
            UART5timeout--;

	if (MIDItimeout) 
		MIDItimeout--;
		
	if (HOSTtimeout)
		HOSTtimeout--;		
	
	if (HOST_LEDtimeout){
		HOST_LEDtimeout--;
		if (HOST_LEDtimeout==0)
			HOST_LEDOff();
	}
	
	if (SYS_LEDtimeout){
		SYS_LEDtimeout--;
		if (SYS_LEDtimeout==0)
			SysLEDOff();
	}			
	
}		



void __ISR(_TIMER_1_VECTOR, ipl2) Timer1Handler(void){   
static unsigned int Timer1Counter=0;
static unsigned char LEDflag=true;

	mT1ClearIntFlag();	 // clear the interrupt flag

        if (LEDflag){
            SysLEDOn();
            LEDflag=false;
        }
        else{
            SysLEDOff();
            LEDflag=true;
        }

		Timer1Counter++; 	
		if (Timer1Counter>100){
			LINXstate=LINX_BREAK;							// This is the beginnning of the BREAK
			Timer1Counter=0;		
		}		
					
		if (LINXstate){
			if (LINXstate==LINX_BREAK){					// This is beginning of BREAK, so
				PORTSetBits(IOPORT_B, BIT_4); 			// pull down the transmit line to start LINX packet
				LINXstate=LINX_MARK;
			}					
			else if (LINXstate==LINX_MARK){				// MARK before transmitting data:
		   		PORTClearBits(IOPORT_B, BIT_4);			// Release transmit line, let it go high to mark end of break	   		
		   		LINXstate=LINX_START;
			}				
			else if (LINXstate==LINX_START)
				INTEnable (INT_U1TX, INT_ENABLED);				
		}			
	
}		

// UART5 interrupt handler - UART #5
// it is set at priority level 2
void __ISR(_UART_5_VECTOR, ipl2) IntUart5Handler(void){
unsigned char ch;

	if(INTGetFlag(INT_SOURCE_UART_RX(UART5))){
	   INTClearFlag(INT_SOURCE_UART_RX(UART5));

		if (UARTReceivedDataIsAvailable(UART5)){
			ch=UARTGetDataByte(UART5);
            UART5RxBuffer[UART5RxHead]=ch;
            INC_UART5RxHead();
            UART5timeout=UART5_TIMEOUT;
		}
	}

	// TX interrupts not used
	if (INTGetFlag(INT_SOURCE_UART_TX(UART5)) ){
		INTClearFlag(INT_SOURCE_UART_TX(UART5));
		INTEnable (INT_U5TX, INT_DISABLED);
	}

}


int sendReceiveVinculum (const char *ptr){
unsigned char ch, i;

	i=0;
	while(ch=ptr[i]){
		while (!UARTTransmitterIsReady(UART5));
        UARTSendDataByte(UART5, ch);
		while (!UARTTransmitterIsReady(UART2));
    	UARTSendDataByte(UART2, ch);        
		i++;
	}
	
	while (!UARTTransmitterIsReady(UART2));
    UARTSendDataByte(UART2, '\r');    

	return(0);
	
}