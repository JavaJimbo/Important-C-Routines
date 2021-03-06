/* ========================================================================================
 * 
 * Encoder Two Servo.c - For PIC 18F2520 
 * 
 *
 * =========================================================================================
 */

#include 	<pic18.h>
#include 	"DELAY16.H"

#include 	<string.h>
#include	<ctype.h>
#include	<math.h>
#include	<stdlib.h>
#include	<stdio.h>
#include 	<htc.h>

#pragma config IESO = OFF, OSC = HS, FCMEN = OFF, BOREN = OFF, PWRT = ON, WDT = OFF, CCP2MX = PORTC, PBADEN = OFF, LPT1OSC = OFF, MCLRE = ON, DEBUG = OFF, STVREN = OFF, XINST = OFF, LVP = OFF, CP0 =	OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF	

#define STX	0x02
#define ETX	0x03
#define DLE	0x10

#define DIRECTION_A		PORTCbits.RC4
#define DIRECTION_B		PORTCbits.RC5
#define FAULT 			PORTCbits.RC3
#define TESTOUT			FAULT

#define MAXBUFFER 64
#define FALSE 0
#define TRUE !FALSE
#define true TRUE
#define false FALSE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h> 
#include <math.h>
#include "DELAY16.H"

unsigned char IntOnChange=false;
unsigned char UARTflag=false;
unsigned char UARTbuffer[MAXBUFFER+1];
unsigned char errorFlag;
void initializePorts(void);
void putch (char byte);
int readEncoderB(unsigned char directionChange);
int readEncoderA(unsigned char directionChange);
int readAD(void);
void ADsetChannel(unsigned char channel);

//void SetDCPWM1(int dutyCycle);
//void SetDCPWM2(int dutyCycle);

union LNG {
	long l;
	unsigned long ul;
	int i[2];
	unsigned int ui[2];
	char b[4];
	unsigned char ub[4];
};

union LNG accel, temp, u0, ypid, velact, phase1dist;
long Bposition=0;
long newPositionA=0, positionA=0, prevPositionA=0, lngTempVelocity;

int mvelocityA=0, prevVelocityA=0, maccelA=0, DnCountA, UpCountA;
int mvelocityB, DnCountB, UpCountB;
unsigned char Direction;	
unsigned char Timer2Flag=false;
unsigned int lastCountA=0;


void main(void) {
unsigned char i, PortBreg, Direction, resetFlag=false;
char dummy, ch;
//int TestCount, PotAValue, PotBValue, lastPotAValue, lastPotBValue;
unsigned int UpCount;
int loopCounter=0;
unsigned int accel=0x0001;

	// ReadTimer0();

	SetDCPWM1(512);

	//eeprom_write (123, 99);
	//dummy = eeprom_read (123);	

	IntOnChange=FALSE;
	//lastPotAValue = -1;
	//lastPotBValue = -1;
	//PotAValue = 0x80;
	//PotBValue = 0x80;

	initializePorts();		
	positionA=0;
	Bposition=0;

	DIRECTION_A = 1;
	DIRECTION_B = 1;
	printf("\r\rTesting PID stuff...\r");
	
	// UpCount = ReadTimer0();

	while(1){	
/*		
		if(IntOnChange)
		{
			IntOnChange=FALSE;
			printf("Direction = %x\r", Direction);					
		}
*/		
		if(UARTflag){				
			UARTflag = FALSE;		
			printf("\rReceived: ");
			for(i=0; i<MAXBUFFER; i++){
				ch = UARTbuffer[i];
				putch(ch);		
				if (ch=='\r')break;						
			}
			putch('\r');
		}		
	
		if (Timer2Flag){
			Timer2Flag=false;

			//ADsetChannel(0); // Read Pot A
			//DelayUs(30);
			//PotAValue = readAD();		
 			//ADsetChannel(1); // Read Pot B
			//DelayUs(30);
			//PotBValue = readAD();	
        	
			//CCPR2L = PotAValue; // PWM #2 is for motor A
			//CCPR1L = PotBValue; // PWM #1 is for motor B			
        	
			if(PORTBbits.RB1==0) 
				DIRECTION_A=0;
			else
				DIRECTION_A=1;
        	
			if(PORTBbits.RB0==0) 
				DIRECTION_B=0;
			else
				DIRECTION_B=1;
				
			if(PORTBbits.RB2==0){
				if (!resetFlag){
					resetFlag=true;
					printf("\rRESET");
					positionA=0;
					prevPositionA=0;
					mvelocityA=0;
					prevVelocityA=0;
					loopCounter=0;
					lastCountA=0;					
					TMR0H=0;						
					TMR0L=0;
					lastCountA=0;
				}								
			}
			else {
				if (resetFlag){
					resetFlag=false;
					T2CONbits.TMR2ON=1;		// Start Timer 2
				}					
					
				newPositionA = positionA;				
				lngTempVelocity = newPositionA - prevPositionA;
				mvelocityA = (int)lngTempVelocity;
				maccelA = mvelocityA - prevVelocityA;
				
				printf("\r#%d: PREV = %ld, POS = %ld, VEL = %d, ACCEL = %d", loopCounter, prevPositionA, newPositionA, mvelocityA, maccelA);
				loopCounter++;
				
				prevVelocityA=mvelocityA;
				prevPositionA=newPositionA;
			}									
				
        	
			//if(PORTBbits.RB5==0)
			//	printf("Motor A = FORWARD, ");
			//else
			//	printf("Motor A = REVERSE, ");
        	
			//if(PORTBbits.RB4==0)
			//	printf("Motor B = FORWARD, ");
			//else
			//	printf("Motor B = REVERSE, ");
			
			// printf("A = %d, B = %d, ", PotAValue, PotBValue);
			// printf("VEL A = %d, VEL B = %d\r", mvelocityA, mvelocityB);
			
			// printf("POS = %ld, VEL = %d, ACCEL = %d\r", positionA, mvelocityA, );
			
			//lastPotAValue=PotAValue;
			//lastPotBValue=PotBValue;			
		}			
	}

}


// This routine returns the number of TICs the encoder
// has moved since the last read. 
// This is the VELOCITY.

// The lastCount variable stores the last read.
// This is subtracted from the new read,
// and the difference is returned.
//
// If the direction has just changed, 
// then the difference is calculated and returned,
// but the timer registers and the lastCount
// are set to zero.
int readEncoderB (unsigned char directionChange){
unsigned int newCount, diffCount, tempCount;
static unsigned int lastCount=0;

	temp.b[0] = TMR1L;					// Read Timer #1. This has been enabled for 16 bit mode, 
	temp.b[1] = TMR1H;					// so low byte should be read first.

	newCount = temp.ui[0];		
	
	if (newCount==lastCount)			// If motor hasn't moved, return 0
		return(0);		
	else if(newCount>lastCount)			
		diffCount=newCount-lastCount;		
	else {								// If new count is LESS than last count, then timer mut have rolled over.		
		tempCount=~lastCount;			// If Timer 1 has rolled over since last read, get difference before rollover and add it to new count
		diffCount=newCount+tempCount;
	}	
	if(directionChange){					// If direction has just changed, reset Timer1 and reset last count.	
		TMR1H=0;						// Using 16 bit mode, so high byte must be written to first
		TMR1L=0;
		lastCount=0;
	}
	else
		lastCount=newCount;
	return(diffCount);
}

// This works the same way as the above routine, except that Timer #0 is used for Encoder A.
int readEncoderA(unsigned char directionChange){
unsigned int newCount, diffCount, tempCount;
//static unsigned int lastCountA=0;

	temp.b[0] = TMR0L;					
	temp.b[1] = TMR0H;					

	newCount = temp.ui[0];	
	
	if (newCount==lastCountA)				
		return(0);			
	else if(newCount>lastCountA)	
		diffCount = newCount-lastCountA;				
	else{	
		tempCount = ~lastCountA;	
		diffCount = newCount+tempCount;
	}	
	if(directionChange){
		TMR0H=0;						
		TMR0L=0;
		lastCountA=0;
	}
	else
		lastCountA=newCount;
	return(diffCount);
}


void initializePorts(void){
	// Initialize ports	
	ADCON1 = 0b00001010;	// Set up Port A for five analog inputs, use VCC and VSS for references.
	ADCON2 = 0b00111111;  	// Use FRC internal oscillator for A/D clock, left justified result
	ADCON0 = 0b00000000;	// Initialize A/D converter, turn it off for now, set for sensorNumber 0

	TRISA = 0b11111111; // All inputs
	TRISB = 0b11111111; // Port B is half input, half output for pushbutton grid.	

	RBPU = 0;			// Enable Port B pullups
	TRISC = 0b10000001; // Input: RC7 = Rx

	// Set up Timer 0 as a counter for Encoder A
	T0CON=0x00;		// Clear everything
	T08BIT=0;		// 16 bit mode
	PSA=1;			// No prescaler
	T0CS=1;			// Use counter input
	T0SE=1;			// Low to high transitions
	TMR0H=0;		
	TMR0L=0;		
	TMR0ON=1;		// Enable counter.	

	// Set up Timer 1 as a counter for Encoder B
	T1CON=0x00;		// Clear prescale
	T1RD16=1;		// Enable 16 bit operation.
	TMR1CS=1;		// Use counter input
	TMR1H=0;		
	TMR1L=0;	
	TMR1ON=1;		// Let her rip

	// Set up Timer 2 for 18 kHz PWM, interrupts every 0.888 ms
	// T2CON = 0x4C;		// 18 kHz PWM, interrupts every 0.55 milliseconds		
	T2CON = 0x00;
	T2CONbits.T2OUTPS0=1;	// POstscaler = 1:16
	T2CONbits.T2OUTPS1=1;
	T2CONbits.T2OUTPS2=1;
	T2CONbits.T2OUTPS3=1;	
	T2CONbits.T2CKPS0=0;	// Prescaler = 1:1
	T2CONbits.T2CKPS1=0;
	PR2 = 0xFF;							
	T2CONbits.TMR2ON=1;		// Start Timer 2
	
	CCPR1L = 0x80;			// Initial duty cycle
	CCP1CON = 0b00001100;	// PWM #1 
	CCPR2L = 0x80;			// Initial duty cycle
	CCP2CON = 0b00001100;	// PWM #2
	
	CCP1CONbits.DC1B0=0;		// PWM #1 Bit 0
	CCP1CONbits.DC1B1=0;		// PWM #1 Bit 1	
	CCP2CONbits.DC2B0=0;		// PWM #2 Bit 0
	CCP2CONbits.DC2B1=0;		// PWM #2 Bit 1	

	// Set up Timer 3 for 1 ms interrupts
	T3CON=0x00;		// Prescaler = 1:1, use internal clock	
	T3RD16=1;		// 16 bit read mode for TMR3L and TMR3H
	TMR3H=238;		// Load timer 3 to roll over 
	TMR3L=0x00;
	TMR3IE=1;		// Enable interrupts.
	TMR3ON=1;		// Start Timer 3.

	BRGH = 1;		// high speed baud rate	
	SPBRG = 19;		// set the baud rate to 57,600 for 18.432 Mhz clock

	SYNC = 0;		// asynchronous 
	SPEN = 1;		// enable serial port pins 
	CREN = 1;		// enable reception 
	SREN = 0;		// no effect 
	TXIE = 0;		// disable tx interrupts 
	RCIE = 0;		// disable rx interrupts 
	TX9  = 0;		// 8- or 9-bit transmission 
	RX9  = 0;		// 8- or 9-bit reception 
	TXEN = 1;		// enable the transmitter 
 
	INTCON = 0x00;  // First, clear all interrupts
	PIE1 = 0;       // Clear all peripheral interrupts
	SSPIE = 0;      // Disable SSP interrupts

	TXIE = 0;       // Disable UART Tx interrupts 
	RCIE = 1;		// Enabled UART Rx interrupts
    
	PEIE = 1;       // Enable peripheral interrupts.
	TMR1IE = 0;		// Disable timer 1 interrupts.
	TMR2IF = 0;		
	TMR2IE = 1;		// Enable Timer2 interrupts
	GIE = 1;        // Enable global interrupts
}


static void interrupt
isr (void){
unsigned char dummy, ch;
static unsigned char timeout=0;
static unsigned char i=0;
static unsigned char TestFlag=false;
static unsigned int Timer2Counter=0;
int PotAValue;

	if (OERR == 1){		// If overrun occurs, flush buffer
   						// and reset receive enable.		
   		CREN = 0;
		CREN = 1;
		dummy = RCREG;
		dummy = RCREG;
	}

	if(RBIF==1){	
		RBIF=0;
		IntOnChange=TRUE;
		Direction=PORTB&0x30;	
	}

	if (RCIF==1){	
		RCIF=0;	
		ch=RCREG;			

		if(i<MAXBUFFER){		
			UARTbuffer[i]=ch;
			i++;	
		}			

		if('\r'==ch){		
			UARTflag=TRUE;
			i=0;
		}
	}

	if(TMR2IF){	
		TMR2IF = 0;	
		Timer2Counter++;
		if (Timer2Counter>=128){
			Timer2Counter=0;
			Timer2Flag=true;
		}				
		
		ADsetChannel(0); // Read Pot A			
		PotAValue = readAD();		
		CCPR2L = PotAValue; // PWM #2 is for motor A		
		
		positionA = positionA + readEncoderA(0);
		
		// mvelocityA = readEncoderA(0);		
		// positionA = positionA + mvelocityA;
		
		// mvelocityB = readEncoderB(0);		
		// Bposition = Bposition + mvelocityB;
		
		if (TestFlag){
			TestFlag=false;
			TESTOUT=1;
		}
		else {
			TestFlag=true;
			TESTOUT=0;
		}						
	}

	if(TMR3IF){ 							// Interrupts every millisecond. For general use.
		TMR3IF=0;		
		TMR3H=238;
		TMR3L=0;							// Load timer 3 to start next 1 ms interrupt	
	}
}



void putch (char byte){
	while(!TXIF)	/* set when register is empty */
		continue;
	TXREG = byte;	
	return;
}


void ADsetChannel(unsigned char channel){
	ADCON0 = (channel << 2) + 0x01;		// enable ADC, RC osc.
}


int readAD(void){
int ADresult;
	GODONE = 1;
	while(GODONE)
		continue;	// wait for conversion complete
	ADresult = (int)ADRESH;

	return(ADresult);
}

