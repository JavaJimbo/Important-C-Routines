/**************************************************************************************
 *
 * DMX CODE FROM: Work\Robotnik Dispatcher XC32\Dispatcher 4-x-x\Main.c
 *
 **************************************************************************************
 * FileName:      Main.c		 
 *
 * 5-1-14  Recompiled using XC32 - UART 2 works
 * 
 **************************************************************************************/
#include "plib.h"
// #include <p32xxxx.h>
#include <stdlib.h> 
#include <stdio.h> 
//#include "ModbusTimer.h"
#include "Delay.h"
//#include "plib.h"
#include <string.h>    
#include <ctype.h>
// #include "Atmel642.h"
#define false FALSE
#define true TRUE

//#include "Delay.h"
#include "HardwareProfileDispatcher.h"
#include "p32mx795f512l.h"
#include "HardwareProfile.h"

//#include "SD-SPI.h"
//#include "FSIO.h"



#define DMXUart		UART1
#define HOSTUart	UART2
#define XBEEUart	UART3
#define MIDIUart	UART4

#define ENDFRAME 0x99
#define STARTFRAME 0x00

#define STANDBY 0
#define ENCODE 	1
#define SEND 	2
#define HOSTTOXBEE	3
#define QUERY	4


#define MIDI_TIMEOUT 	8  // 
#define REMOTE_TIMEOUT  64  // 
#define REMOTE_PAUSE    64 // 
#define HOST_TIMEOUT    8  // 

#define INC_MIDIRxHead(); MIDIRxHead++; if (MIDIRxHead>=MAXBUFFER) MIDIRxHead=0;		
#define INC_MIDIRxTail(); MIDIRxTail++; if (MIDIRxTail>=MAXBUFFER)	MIDIRxTail=0;
#define INC_MIDITxHead(); MIDITxHead++; if (MIDITxHead>=MAXBUFFER) MIDITxHead=0;			
#define INC_MIDITxTail(); MIDITxTail++; if (MIDITxTail>=MAXBUFFER) MIDITxTail=0;

#define INC_HOSTRxHead(); HOSTRxHead++; if (HOSTRxHead>=MAXBUFFER)	HOSTRxHead=0;	
#define INC_HOSTRxTail(); HOSTRxTail++; if (HOSTRxTail>=MAXBUFFER)	HOSTRxTail=0;			
#define INC_HOSTTxHead(); HOSTTxHead++; if (HOSTTxHead>=MAXBUFFER)	HOSTTxHead=0;	
#define INC_HOSTTxTail(); HOSTTxTail++; if (HOSTTxTail>=MAXBUFFER)	HOSTTxTail=0;				
	
#define INC_XBEERxHead(); XBEERxHead++; if (XBEERxHead>=MAXBUFFER)	XBEERxHead=0;			
#define INC_XBEERxTail(); XBEERxTail++; if (XBEERxTail>=MAXBUFFER)	XBEERxTail=0;	
#define INC_XBEETxHead(); XBEETxHead++; if (XBEETxHead>=MAXBUFFER)	XBEETxHead=0;			
#define INC_XBEETxTail(); XBEETxTail++; if (XBEETxTail>=MAXBUFFER)	XBEETxTail=0;
	
	
//#define INC_DMXTxHead(); DMXTxHead++; if (DMXTxHead>=MAXBUFFER) DMXTxHead=0;			
//#define INC_DMXTxTail(); DMXTxTail++; if (DMXTxTail>=MAXBUFFER) DMXTxTail=0;	
	
#define INC_DMXRxHead(); DMXRxHead++; if (DMXRxHead>=MAXBUFFER) DMXRxHead=0;			
#define INC_DMXRxTail(); DMXRxTail++; if (DMXRxTail>=MAXBUFFER) DMXRxTail=0;		

const unsigned char SETSYS=0xE0;
const unsigned char SETBOT=0x80;
 
const unsigned char DMXBOT=0x05;
const unsigned char MIDIBOT=0x06;

const unsigned char DMX_CONTROL=	0xC4;
const unsigned char MIDI_CONTROL=	0xC5;


#define DMXLOCAL  	0xD0
#define DMXHOST 	0xD9
unsigned char DMXmode=DMXLOCAL;
unsigned char botNumber=0;

unsigned char SYSsubCommand=0;

#define	STX	2		// First byte of every packet
#define ETX 3		// Last byte of every packet
#define DLE 6		// Escape chracter preceding data bytes to avoid confusing them with STX and ETX
#define PLUS 43 	

#define	DMX_STANDBY	0
#define	DMX_BREAK 	1
#define DMX_MARK 	2
#define DMX_START 	3
#define DMX_DATA	4
#define DMXLENGTH 	16
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

unsigned char MIDITxBuffer[MAXBUFFER+1]; // ={0xB0,0x00,0x7F,0x20,0x00,0xC0,0x30};
unsigned char MIDIRxBuffer[MAXBUFFER+1];
unsigned char MIDIRxHead=0;
unsigned char MIDIRxTail=0;
unsigned char MIDITxHead=0; 
unsigned char MIDITxTail=0;
unsigned char HOSTstate=STANDBY;
unsigned char MIDIdata=false;
unsigned char DMXdata=false;
unsigned int  MIDItimeout=0;

unsigned char XBEETxBuffer[MAXBUFFER];
unsigned char XBEERxBuffer[MAXBUFFER];
unsigned char XBEETxTail=0;
unsigned char XBEETxHead=0;
unsigned char XBEERxTail=0;
unsigned char XBEERxHead=0;
unsigned char XBEEtimeout=0;

static unsigned char DMXstate=DMX_STANDBY;

unsigned char DMXRxBuffer[MAXBUFFER+1];
unsigned char DMXTxBuffer[MAXBUFFER+1];
unsigned char DMXflag=false;

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

#define COMMAND_DMX_LOCAL 1

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

int main(void){
int	UART1baudrate, UART2baudrate, UART3baudrate, UART4baudrate, sysClockRate, strSize;
unsigned char ch;
unsigned short	i, j;
#define MAXSTRING 64
unsigned char MIDIstring[MAXSTRING];
unsigned char scopeFlag=false;
unsigned char TxString[128];
unsigned char RS485enable=FALSE;


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
	
	// Set up UART #1 for DMX512 @ 25000 baud   	
	UARTConfigure(DMXUart, UART_ENABLE_HIGH_SPEED|UART_ENABLE_PINS_TX_RX_ONLY);   
	UARTSetFifoMode(DMXUart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);  	  	
	UARTSetLineControl(DMXUart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_2);	
	UART1baudrate=UARTSetDataRate(DMXUart, GetPeripheralClock(), 250000);  
        UARTEnable(DMXUart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
   	
	// Configure UART #1 interrupts
	INTEnable(INT_SOURCE_UART_RX(DMXUart), INT_ENABLED);
	INTEnable (INT_U1TX, INT_DISABLED);
   	INTSetVectorPriority(INT_VECTOR_UART(DMXUart), INT_PRIORITY_LEVEL_1);
   	INTSetVectorSubPriority(INT_VECTOR_UART(DMXUart), INT_SUB_PRIORITY_LEVEL_0);   	   			
	

	// Set up UART #2 for 57600 baud
//    UARTConfigure(HOSTUart, UART_ENABLE_PINS_TX_RX_ONLY);	
	UARTConfigure(HOSTUart, UART_ENABLE_HIGH_SPEED|UART_ENABLE_PINS_TX_RX_ONLY);      	
	UARTSetFifoMode(HOSTUart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);        
	UARTSetLineControl(HOSTUart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);		
	UART2baudrate=UARTSetDataRate(HOSTUart, GetPeripheralClock(), 57600);               
        UARTEnable(HOSTUart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

	// Configure UART #2 Interrupts
	INTEnable (INT_U2TX, INT_DISABLED);	
	INTEnable (INT_SOURCE_UART_RX(HOSTUart), INT_ENABLED);
   	INTSetVectorPriority(INT_VECTOR_UART(HOSTUart), INT_PRIORITY_LEVEL_2);
   	INTSetVectorSubPriority(INT_VECTOR_UART(HOSTUart), INT_SUB_PRIORITY_LEVEL_0);

	// Set up UART #3 for XBEE at 57600 baud
//    UARTConfigure(XBEEUart, UART_ENABLE_PINS_TX_RX_ONLY);	
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
   	
   	// Display startup banners:

        /*
	sprintf (TxString, "\r\rSTART: clock = %d, Tick = %d", sysClockRate, TICK);
	putsUART2(TxString);        		
   	
   	sprintf (TxString, "\rUart #%d, baudrate = %d", DMXUart+1,  UART1baudrate);
   	putsUART2(TxString);  
   	
   	sprintf (TxString, "\rUart #%d, baudrate = %d", HOSTUart+1,  UART2baudrate);
   	putsUART2(TxString);  	
   	
   	sprintf (TxString, "\rUart #%d, baudrate = %d", XBEEUart+1,  UART3baudrate);
   	putsUART2(TxString);  	

   	sprintf (TxString, "\rUart #%d, baudrate = %d", MIDIUart+1,  UART4baudrate);
   	putsUART2(TxString);  	
*/

 	
   	// Configure Timer 1 using internal clock, 1:8 prescale, Postscale = 10,000 for 1 millisecond interrupts
   	OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_8, TICK);     	
   
        // Set up the timer interrupt with a priority of 2
        ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
    
	// Set up Timer 2 for 100 microsecond roll-over rate
 	OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_32, 328);
 	
 	// set up the core timer interrupt with a priority of 5 and zero sub-priority     
 	ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_5);     
	
   	// Enable interrupts
   	INTEnableInterrupts();     	   		
	//DelayMs(100);	 
	HOST_LEDOff();
	SysLEDOn();
	SYS_LEDtimeout=4000;   

        printf("\rHOST receives, enables RS485, transmits out UART #3 to RS485 bus");


	DelayMs(200);
	strcpy(HOSTRxBuffer, "\rThat's what I call ballin the jack");
	HOSTRxTail=0;
	HOSTRxHead=34;
       
	while(1){

		                           
		// If a complete packet has been received from the HOST, process it:
		while (HOSTtimeout==0 && HOSTRxTail!=HOSTRxHead){
                    ch=HOSTRxBuffer[HOSTRxTail];
                    INC_HOSTRxTail();
                    XBEETxBuffer[XBEETxHead]=ch;
                    INC_XBEETxHead();
		}
		
		
		while (XBEETxTail!=XBEETxHead){
                    if (!RS485enable){
                        RS485enable=TRUE;
                        SetRS485();
                        DelayMs(4);
                    }
                    if (UARTTransmitterIsReady(XBEEUart)){
                        ch=XBEETxBuffer[XBEETxTail];
			INC_XBEETxTail();						
			UARTSendDataByte(XBEEUart, ch);															
                    }
		}

                if (RS485enable){
                    DelayMs(4);
                    RS485enable=FALSE;
                    ClearRS485();
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





//	DMX512 INTERRUPTS
// 	UART 1 interrupt handler for receiving and transmitting DMX512 data
// 	Priority level 1
void __ISR(_UART1_VECTOR, ipl1) IntUart1Handler(void){
static unsigned short 	DMXTxPtr=0;
static unsigned short	DMXRxPtr=0;
static unsigned char 	dummyCounter=0;
unsigned char 			ch, dummy;
	
	// RX interrupts
	if(INTGetFlag(INT_SOURCE_UART_RX(DMXUart))){
		// Clear the RX interrupt Flag
	   INTClearFlag(INT_SOURCE_UART_RX(DMXUart));
	   
   		if (1==U1STAbits.OERR)				// If DMX overrun occurs, clear overrun flag
				U1STAbits.OERR=0;										   
	   
		if (U1STAbits.FERR==1){					
   			dummy=UARTGetDataByte(DMXUart); 	
 			dummyCounter=0;
			DMXRxPtr=0;																									
		}			
		else if (UARTReceivedDataIsAvailable(DMXUart)){
			ch=UARTGetDataByte(DMXUart);	
			if(dummyCounter<1)
				dummyCounter++;			
			else if (DMXRxPtr<DMXLENGTH){	
				DMXRxBuffer[DMXRxPtr]=ch;			 
				DMXRxPtr++;
				if (DMXRxPtr==DMXLENGTH){
					DMXflag=true;					
				}					
			}	
		}			
	}	

	// TX interrupts: 
	if (INTGetFlag(INT_SOURCE_UART_TX(DMXUart))){
		INTClearFlag(INT_SOURCE_UART_TX(DMXUart));
		if (DMXstate>=DMX_START){
			if (DMXTxPtr<DMXLENGTH){
				if (DMXstate==DMX_START){			
					ch=0x00;
					DMXstate++;
				}
				else {
					ch=DMXTxBuffer[DMXTxPtr];	// was DMXRxBuffer
					DMXTxPtr++;
				}
				while (!UARTTransmitterIsReady(DMXUart));				
				UARTSendDataByte(DMXUart, ch);								
			}									
			else {
				DMXstate=DMX_STANDBY;	
				INTEnable (INT_U1TX, INT_DISABLED);
				DMXTxPtr=0;					
			}		
		}			
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


// HOST Communication processed here
// UART 2 interrupt handler for RS232 diagnostics
// it is set at priority level 2
void __ISR(_UART2_VECTOR, ipl2) IntUart2Handler(void){
unsigned char ch;	

	if(INTGetFlag(INT_SOURCE_UART_RX(HOSTUart))){		
	   INTClearFlag(INT_SOURCE_UART_RX(HOSTUart));
		
		if (UARTReceivedDataIsAvailable(HOSTUart)){
                        ch=UARTGetDataByte(HOSTUart);
			HOSTRxBuffer[HOSTRxHead]=ch;
			INC_HOSTRxHead();		
			HOSTtimeout=HOST_TIMEOUT;
		}				
	}			
	
	// TX interrupt: If Tx is true and 
	if (INTGetFlag(INT_SOURCE_UART_TX(HOSTUart)) ){
		INTClearFlag(INT_SOURCE_UART_TX(HOSTUart));
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
unsigned char 	DMXTxPtr=0;
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
    	   		if (botNumber==DMXBOT && DMXmode==DMXLOCAL) // If HOST mode isn't enabled, don't accept DMX commands
    	   			botNumber=0;        	   		
    	   		state=COMMAND;
    	   		DMXTxPtr=0;
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
			else if(botNumber==DMXBOT){
				if (DMXTxPtr<DMXLENGTH){
					DMXTxBuffer[DMXTxPtr]=ch;
					DMXTxPtr++;
				}							
			}						
		}																		
		if (botNumber!=0 && botNumber!=MIDIBOT && botNumber!=DMXBOT){
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
					HOSTstate=ENCODE;		 // encode existing MIDI,DMX, and XBEE data to send to HOST										
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

	mT1ClearIntFlag();								// clear the interrupt flag		

		Timer1Counter++; 	
		if (Timer1Counter>100){
			DMXstate=DMX_BREAK;							// This is the beginnning of the BREAK
			Timer1Counter=0;		
		}		
					
		if (DMXstate){				
			if (DMXstate==DMX_BREAK){					// This is beginning of BREAK, so
				PORTSetBits(IOPORT_B, BIT_4); 			// pull down the transmit line to start DMX packet
				DMXstate=DMX_MARK;
			}					
			else if (DMXstate==DMX_MARK){				// MARK before transmitting data:
		   		PORTClearBits(IOPORT_B, BIT_4);			// Release transmit line, let it go high to mark end of break	   		
		   		DMXstate=DMX_START;
			}				
			else if (DMXstate==DMX_START)
				INTEnable (INT_U1TX, INT_ENABLED);				
		}			
	
}		

