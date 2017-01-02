/* ========================================================================================
  	Servo Rohm - Single servo control with pot position feedback

	9-8-13: Works well with single turn pots.
	Slight overshoot but pretty quick response time.
	No derivative correction yet.
 	9-8-13: Added Derivative correction - works beautifully.
 		Additional changes to work with 12V supply.
  =========================================================================================
 */
 
#define TESTOUT PORTEbits.RE0
#define ADStartConversion(ch){ADCON0 = (ch << 2) | 0x03;}  // Set AD channel, enable converter, start conversion 
 
#include 	<pic18.h>
#include 	"DELAY16.H"
#include 	<string.h>
#include	<ctype.h>
#include	<math.h>
#include	<stdlib.h>
#include	<stdio.h>
//#include 	<htc.h>
#include 	<stdlib.h>
#include 	<stdio.h>
#include 	<ctype.h>

#pragma config IESO = OFF, OSC = HS, FCMEN = OFF, BOREN = OFF, PWRT = ON, WDT = OFF, CCP2MX = PORTC, PBADEN = OFF, LPT1OSC = OFF, MCLRE = ON, DEBUG = OFF, STVREN = OFF, XINST = OFF, LVP = OFF, CP0 =	OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF	

#define STANDBY 	0 
#define START		1 
#define RUN 		2 
#define BRAKE 		3 
#define STOP 		4 

#define FORWARD 0
#define REVERSE 1

#define BUFFERSIZE 64
#define FALSE 0
#define TRUE !FALSE
#define true TRUE
#define false FALSE

#define MAX_PWM 512

unsigned char TimerFlag=FALSE;

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h> 
#include <math.h>
#include "DELAY16.H"          

#define KP 100
#define KD 100
#define KI 1
#define ACCEL 20

int kI=KI;


unsigned char UARTflag=false;
unsigned char UARTbuffer[BUFFERSIZE];

void initializePorts(void);
void putch (char byte);
void ADsetChannel(unsigned char channel);
int convertAD(unsigned char channel);
int filterAD(unsigned char channel, int rawADdata);
int readAD (void);
int getADresult(void);

void taskServo (int destPos, int actualPos);
void taskMotor(unsigned char motorState, unsigned char direction, int motorPWM);
void processSerialCommand(void);




#define RAW_WEIGHT		4
#define FILTER_WEIGHT	6
#define DIVIDER (RAW_WEIGHT + FILTER_WEIGHT)

#define NUM_CHANNELS 5
int ADfilter[NUM_CHANNELS];
int ADdata[NUM_CHANNELS];

int measPos, comPos, setPos, diffPos;



int filterAD(unsigned char channel, int rawADdata){
	ADfilter[channel] = ((ADfilter[channel]*FILTER_WEIGHT) + (rawADdata*RAW_WEIGHT)) / DIVIDER;
	return(ADfilter[channel]);
}		
	
unsigned char brakeTimer2Flag=FALSE;	
#define setMotorFORWARD SetDCPWM2
#define setMotorREVERSE SetDCPWM1
unsigned char servoEnable=TRUE;

void main(void) {	
int ADreading, PWMout;
int i, destPos, actualPos;

		
	initializePorts();	
	setMotorFORWARD(0);
	setMotorREVERSE(0);			

	DelayMs(100);
	printf("\rStarting Servo program");
	TESTOUT = 0;
	
	for (i=0; i<NUM_CHANNELS; i++){
		ADdata[i]=0;
		ADfilter[i]=0;
	}		
		
	while(1){						
		if (UARTflag){
			UARTflag=FALSE;
			processSerialCommand();
		}			
				
		if (TimerFlag){
			TimerFlag=FALSE;		
			destPos = ADdata[4];	// Destination pot is AN4 = RA5
			actualPos = ADdata[1];	// Servo pot is AN1 = RA1
			if (servoEnable) taskServo(destPos, actualPos);
		}	
		else if(!servoEnable) {
			setMotorFORWARD(0);
			setMotorREVERSE(0);				
		}					
	}	
}		


#define DEADBAND 10 // was 4
#define PWM_MIN 75 // was 100
#define PWM_MAX 1023

#define COUNTDOWN 6 // was 10
#define BRAKE_TIME 10
#define DISPLAY_COUNTER 10
 
void taskServo (int destPos, int actualPos){
static unsigned char state=START;
static int previousSetPos=0;
static unsigned char saturation=FALSE;	
static unsigned char direction=FORWARD;
static unsigned char previousDirection=FORWARD;
static int stopCounter=COUNTDOWN;
static int brakeCounter=COUNTDOWN;
static int commandPos=0;
static int errorPWM=0;
static int servoPWM=0;
long KIerr, KPerr, KDerr, error, diffError, prevError;
int KPerrInt, KIerrInt, KDerrInt;
int errorInt;
static int timeout=1000;
static long sumError=0;
static int displayCounter=0;
int diffPos, diffPot;
static unsigned char testFlag=TRUE;
#define NUM_ERRORS 6
static int errorArray[NUM_ERRORS];
static unsigned char errIndex=0;	
		
	if (testFlag){
		testFlag=FALSE;
		TESTOUT=1;
	}
	else{
		testFlag=TRUE;
		TESTOUT=0;
	}					
		
	diffPos = abs(actualPos-destPos);				
					
	if (state){				
		switch(state){
			case START:				
				if (destPos<actualPos)
					direction=FORWARD;														
				else direction=REVERSE;	
				
				state=RUN;
				sumError=0;
				printf ("\r\rSTART DEST: %d", destPos);
				commandPos=actualPos;								
				break;
				
			case BRAKE:     
				if (diffPos>(DEADBAND*2)){
					stopCounter=COUNTDOWN;
					state=RUN;
				}					
				else if (stopCounter){
					stopCounter--;		
					break;
				}								
				else {					
					servoPWM=0;	
					sumError=0;		
					state=STOP;			
					break;
				}									
			case RUN:							
				if (diffPos<DEADBAND){																	
					if (brakeCounter)
						brakeCounter--;
				}										
				else brakeCounter=COUNTDOWN;

				if (brakeCounter==0)
					state=BRAKE;													
				else {
					stopCounter=COUNTDOWN;				
											
					// Update commanded position until 
					// it catches up with destination position:
					if (commandPos<destPos){
						commandPos=commandPos+ACCEL;
						if (commandPos>destPos) 
							commandPos=destPos;
					}		
					else if (commandPos>destPos){
						commandPos=commandPos-ACCEL;
						if (commandPos<destPos) 
							commandPos=destPos;
					}	
					
					// Determine motor direction.
					// Then get position error. 
					// Error should be negative if actual is less than 
					// command position (servo is running slow)				
					if (actualPos<commandPos){
						direction=FORWARD;		
						error = actualPos - commandPos;
					}						
					else if (actualPos>commandPos){
						direction=REVERSE;	
						error = commandPos - actualPos;
					}		
					else error=0;		
						
					if (abs(error)<32000)
						errorArray[errIndex]=(int) error;
					else errorArray[errIndex]=32000;					
					errIndex++;
					if (errIndex>=NUM_ERRORS) errIndex=0;
					
					prevError=(long)errorArray[errIndex];						
					diffError=error-prevError; 
					
					if (previousDirection!=direction)
						sumError=0;
					previousDirection=direction;					 					
						
					// Now compute PID correction:									
					if (!saturation)
						sumError = sumError + error;
					KPerr = KP * error;	
					KIerr = KI * sumError;	
					KDerr = KD * diffError;
					errorPWM = (KPerr + KIerr + KDerr)/100;
					
					errorInt=(int)error;
					KPerrInt = (int) KPerr;
					KIerrInt = (int) KIerr;
					KDerrInt = (int) KDerr;
					
					
					// Update PWM:
					servoPWM = PWM_MIN - errorPWM;				
					
					// Clamp PWM within limits:
					saturation = FALSE;		
					if (servoPWM<0) 
						servoPWM=0;
					else if (servoPWM>PWM_MAX) {
						servoPWM = PWM_MAX;		
						saturation = TRUE;
					}	
				}					
				break;			
			
			case STOP:		
				if (diffPos>DEADBAND)
					state=RUN;
				else {
					servoPWM=0;						
					printf ("\rSTOP: Actual = %d, Dest = %d, Diff = %d",  actualPos, destPos, diffPos);
				}					
				commandPos=actualPos;					
				break;																							
		}		
		taskMotor(state, direction, servoPWM);
	}
	else taskMotor(STANDBY, 0, 0);
		
	//if (displayCounter)
	//	displayCounter--;
	//if (!displayCounter){				
	//	displayCounter=10;				
		/*
		if (state==RUN){
			if (direction==FORWARD)	printf ("\rFORWARD ");	
			else printf ("\rREVERSE ");	
		}			
		else if (state==STOP) printf ("\rSTOP ");
		else if (state==BRAKE) printf ("\rBRAKE ");
		else printf ("\rSTANDBY ");	
		*/
			
		if (state==RUN) 
			printf ("\rERR: %d, KpERR: %d, KiERR: %d, KDerr: %d, PWM: %d", errorInt, KPerrInt, KIerrInt, KDerrInt, servoPWM);
		else if(state==BRAKE) printf ("\rBRAKE");		
	//}			
}						
	


/* taskMotor()
	This routine accepts a command PWM value
	and controls the motor accordingly.
	It returns TRUE if the value is written to the PWM 
	or FALSE if the motor has to reverse direction
	and needs to brake beforehand.
*/  	
 

#define BRAKE_PWM 1023  

 
void taskMotor(unsigned char motorState, unsigned char direction, int motorPWM){
	switch (motorState){	    
    	case BRAKE:
			setMotorFORWARD(BRAKE_PWM);
			setMotorREVERSE(BRAKE_PWM);
			break;    		
			
		case RUN:
			if (direction==FORWARD){
				setMotorFORWARD(motorPWM);
				setMotorREVERSE(0);
			}
			else {		
				setMotorFORWARD(0);
				setMotorREVERSE(motorPWM);
			}						
			break;
    
    	
		case STOP:
		case STANDBY:
		default:			
			setMotorFORWARD(0);
			setMotorREVERSE(0);
			break;				
	}			
}	

void initializePorts(void){
	
	// Initialize A/D converter
	ADCON1 = 0;	
	ADCON2 = 0;
	ADCON0 = 0;	
	
	// ADCON0 register
	ADON=1;	// Enable A/D converter, set channels later
	
	// ADCON1 register
	VCFG0=0;	// Use +5V for +VREF (1 when RA3 is VREF)
	VCFG1=0;	// Use GND for -VREF
	PCFG0=1;	// Use analog channels 0,1,2,3,4 
	PCFG1=0;
	PCFG2=0;
	PCFG3=1;	

	// ADCON2 register
	ADFM=1; 	// Right justified A/D conversion
	ACQT2=1;	// A/D Acquisition time = 20 TAD
	ACQT1=1;
	ACQT0=1;			
	ADCS2=1;	// A/D conversion clock use FRC
	ADCS1=1;	
	ADCS0=1;			

	TRISA = 0b11111111; // All inputs
	TRISB = 0b11111111; // Port B is half input, half output for pushbutton grid.	

	RBPU = 0;			// Enable Port B pullups
	TRISC = 0b10000001; // Input: RC7 = Rx
	TRISE = 0b00000000; // Output

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

	// Set up Timer 2 for 25 kHz PWM, 	
	T2CON = 0x00;
	T2CONbits.T2OUTPS0=1;	// Postscaler = 1:16
	T2CONbits.T2OUTPS1=1;
	T2CONbits.T2OUTPS2=1;
	T2CONbits.T2OUTPS3=1;	
	
	T2CONbits.T2CKPS0=0;	// Prescaler = 1:1
	T2CONbits.T2CKPS1=0;
	PR2 = 183;
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
	TMR3H=238;		// Load Timer 3 to roll over 
	TMR3L=0x00;	
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
	ADIE = 0;		
	TMR1IE = 0;		 
	TMR2IE = 1;		 
	TMR3IE = 1;		 
	GIE = 1;        // Enable global interrupts
}



static void interrupt
isr (void){
unsigned char dummy, ch;
static unsigned char i=0;
static unsigned int Timer3Counter=0;
static unsigned int Timer2Counter=0;
static unsigned char channel=0;
int rawADdata;

	if (OERR == 1){		// If overrun occurs, flush buffer and COUNTDOWN receive enable.		
   		CREN = 0;
		CREN = 1;
		dummy = RCREG;
		dummy = RCREG;
	}
	
	if (ADIF){
		ADIF=0;				
		rawADdata=getADresult();
		ADdata[channel]=filterAD(channel, rawADdata);		
		channel++;
		if (channel>=NUM_CHANNELS){
			channel=0;
			ADIE=0;			
		}			
		else ADStartConversion(channel);				
	}		

	if (RBIF==1){	
		RBIF=0;				
	}

	if (RCIF==1){	
		RCIF=0;	
		ch = toupper(RCREG);	
		if (ch==' ') servoEnable=FALSE;
		else if(i<BUFFERSIZE){
			UARTbuffer[i]=ch;
			i++;		
		}
		if(ch=='>')
			i=0;
		if (ch=='\r'){
			UARTflag=TRUE;			
			i=0;
		}			
	}

	if (TMR2IF){					// Interrupts every 625 uS
		TMR2IF=0;		
		
		Timer2Counter++;
		if (Timer2Counter>=16){		// 100 Hz:
			brakeTimer2Flag=TRUE;
			Timer2Counter=0;	
			
			channel=0;	
			ADStartConversion(channel);	
			ADIE=1;			
		}			
	}
						
	if (TMR3IF){ 		// Interrupts every millisecond. 
		TMR3IF=0;
		TMR3H=238;
		TMR3L=0;		// Load brakeTimer 3 to start next 1 ms interrupt	
		
		Timer3Counter++;
		if (Timer3Counter>=10){
			Timer3Counter=0;
			TimerFlag=TRUE;
		}				
	}
}



void putch (char byte){
	while(!TXIF)	// set when register is empty
		continue;
	TXREG = byte;	
	return;
}


void ADsetChannel(unsigned char channel){
	ADCON0 = (channel << 2) | 0x03;		// Set AD channel, enable converter, start conversion
}


int readAD(void){
int ADresult;
	ADresult = (int) (ADRESH<<8);	
	ADresult = ADresult | ADRESL;	
	return(ADresult);
}

int convertAD(unsigned char channel){
int ADresult;
	ADCON0 = channel << 2; 			// Set A/D channel		
	ADON=1;							// Enable converter
	GODONE = 1;						// Start acquisition/conversion
	
	while(GODONE) continue;			// wait for conversion complete
	ADresult = (int) (ADRESH<<8);	
	ADresult = ADresult | ADRESL;
	
	return(ADresult);
}


int getADresult(void){
int ADresult; 
	ADresult=(int)(ADRESH<<8);	
	ADresult=ADresult|ADRESL; 
	return(ADresult);
} 



void processSerialCommand(void){  
unsigned char i, j;	
unsigned char command, ch;
unsigned char numBuff[9];
int numValue=0;

	j=0;
	for (i=0; i<BUFFERSIZE; i++){
		ch=UARTbuffer[i];
		if (isdigit(ch) && j<8){
			numBuff[j]=ch;
			j++;
		}			
	}	
	if(j>0 && j<9){
		numBuff[j]='\0';
		numValue=atoi(numBuff);
	}	
	else numValue=0;
	
	command=toupper(UARTbuffer[0]);
	printf ("\r\nCOMMAND = %c", command);
	
	switch(command){
					
		//case 'A':
		//	ACCEL=numValue;
		//	printf(" ACCEL = %d", ACCEL);
		
		case 'I':
			kI=numValue;
			printf(" KI = %d", kI);
					
		case 'O':
			servoEnable=TRUE;
			printf ("\rMOTOR ENABLED");
			break;			
				
		default:
			printf ("\r\nBAD COMMAND");						
			break;
	}		
}		