// 	AD5172_I2C - written by Jim Sedgwick
//	Routines for I2C communication with the AD5172 Pot IC
// 	Includes routines for reading and writing
// 	AD5172 is at fixed hardware address of 0x5E
// 	Global variable errorFlag is set to "1" if an acknowledge error occurs
// 	Revision history
//	2007-12-14 JBS: first pass routines were created.
//	2008-1-1 JBS: 	errorFlag is not cleared at beginning of reads or writes.
// 	2008-1-3 JBS: 	print OTP_Validation_Flag.
//  2008-1-16 JBS: Version that works with new board.
//					Fixed I2C bugs pertaining to ACKS for Reads.
//	2008-4-9 JBS:	Added read and write routines for I2C
// 	2008-4-18 JBS:	Added I2C TIMEOUT errorFlag to AD5172.

#define AD5172_SLAVE_ADDRESS 0x5E
#define MAXCOUNTS 10

#include 	<pic18.h>
#include 	"DELAY16.H"
#include 	"Humidity Cal Master.h"
#include 	"AD5172_I2C.h"
#include	"stdio.h"

extern unsigned char timeout;
extern unsigned char errorFlag, WCOLflag;
extern unsigned char OTP_Validation_Flag;
extern unsigned char outBuffer[];
extern unsigned char inBuffer[];

#define LED	RD3


//	This routine sets up I2C functions for master mode on the PIC
void initialize_I2C(void)
{
	// 0010 1000B = 0x28
	SSPCON1 = 0x28; 	// SSPEN = 1, SDA and SCL for I2C
					// I2C Master mode = Fosc/ (4 * (SSPADD + 1))
	SSPADD = 9;	// FBUS = OSC/4 /(N+1)  OSC = 4, OCS =0.4; 10 IS JUST RIGHT
	SSPSTAT = 0;	// Clear MSSP status register

	// Make sure I2C is STOPPED:
	AD5172_Stop();
}


//  Send stop condition
// 	  - data low-high while clock high
void
AD5172_Stop(void)
{	
	SSPIF = 0;			// CLEAR FLAG
	PEN = 1;			// Send STOP condition
	timeout=0;
	while ((SSPIF == 0)&&(timeout<MAXCOUNTS)); // WAIT TILL SSPIF SET IN PIR REGISTER	
	if (SSPIF == 0) errorFlag=TIMEOUT;
}

//	Send start condition
void
AD5172_Start(void)
{
	SSPIF=0;			// Clear flag before starting
	SEN = 1;			// Send START condition	
	timeout=0;
	while ((SSPIF == 0)&&(timeout<MAXCOUNTS)); // WAIT TILL SSPIF SET IN PIR REGISTER	
	if (SSPIF == 0) errorFlag=TIMEOUT;	
}


//	Send restart condition
void
AD5172_Restart(void)
{
	SSPIF = 0;			// CLEAR FLAG
	RSEN = 1;			// Send REPEATED START condition
	timeout=0;
	while ((SSPIF == 0)&&(timeout<MAXCOUNTS)); // WAIT TILL SSPIF SET IN PIR REGISTER	
	if (SSPIF == 0) errorFlag=TIMEOUT;
}

//	Send a byte to the slave
void AD5172_SendByte(unsigned char byte)
{
	SSPIF = 0;			// CLEAR FLAG
	SSPBUF = byte;		// Send byte
	timeout=0;
	while ((SSPIF == 0)&&(timeout<MAXCOUNTS)); // WAIT TILL SSPIF SET IN PIR REGISTER	
	if (SSPIF == 0) errorFlag=TIMEOUT;		
}

//	Check for an acknowledge
// 	  - returns ack or ~ack
unsigned char get_Acknowledge(void)
{
unsigned char ack;

	ack = ACKSTAT;
	return ack;
}

//	Send  an acknowledge
void send_Acknowledge(unsigned char ack)
{
 	SSPIF=0;
	ACKDT=ack; 		// Set acknowledge bit
	ACKEN=1;		// Send it	
	timeout=0;
	while ((SSPIF == 0)&&(timeout<MAXCOUNTS)); // WAIT TILL SSPIF SET IN PIR REGISTER	
	if (SSPIF == 0) errorFlag=TIMEOUT;		
}

//	Read a byte from the slave
int
AD5172_ReceiveByte(void)
{
unsigned char byte;
	
	SSPIF=0;
	RCEN = 1;	// Enable receive mode for I2C
	timeout=0;
	while ((SSPIF == 0)&&(timeout<MAXCOUNTS)); // WAIT TILL SSPIF SET IN PIR REGISTER	
	if (SSPIF == 0) errorFlag=TIMEOUT;		
	byte = SSPBUF;
	return (int)byte;
}


void AD5172_Write(unsigned char potSelect, unsigned char dataByte) 
{
unsigned char instructionByte=0x00;
	//errorFlag = 0x00;	// Clear error flag
	// Send START condition and WRITE command OR'd with the AD5172 slave address, which is 0x5E	
	AD5172_Start();	
	AD5172_SendByte(I2C_WRITE|AD5172_SLAVE_ADDRESS);	
	errorFlag = errorFlag|get_Acknowledge();

	// Send pot select instruction byte 
	if(POT1==potSelect)
		instructionByte=0x08;
	else
		instructionByte=0x88;
	
	AD5172_SendByte(instructionByte);
	errorFlag = errorFlag|get_Acknowledge();

	// Now send DATA byte, then read acknowledge:
	AD5172_SendByte(dataByte);
	errorFlag = errorFlag|get_Acknowledge();

	// Send STOP condition
	AD5172_Stop();	
}



void AD5172_Set(unsigned char potSelect) 
{
unsigned char instructionByte=0x00;
	//errorFlag = 0x00;	// Clear error flag
	// Send START condition and WRITE command OR'd with the AD5172 slave address, which is 0x5E	
	AD5172_Start();	
	AD5172_SendByte(I2C_WRITE|AD5172_SLAVE_ADDRESS);	
	errorFlag = errorFlag|get_Acknowledge();

	// Send pot select instruction byte 
	if(POT1==potSelect)
		instructionByte=0x00;
	else
		instructionByte=0x80;
	
	AD5172_SendByte(instructionByte);
	errorFlag = errorFlag|get_Acknowledge();

	// Send STOP condition
	AD5172_Stop();	
}


void AD5172_Program(unsigned char potSelect)
{
unsigned char instructionByte=0x00;
	//errorFlag = 0x00;	// Clear error flag
	// Send START condition and WRITE command OR'd with the AD5172 slave address, which is 0x5E	
	AD5172_Start();	
	AD5172_SendByte(I2C_WRITE|0x5E);	
	errorFlag = errorFlag|get_Acknowledge();

	if(POT1==potSelect)instructionByte=0x28;	// Program pot 1
	else instructionByte=0xA8;					// Program pot 2

	AD5172_SendByte(instructionByte);
	errorFlag = errorFlag|get_Acknowledge();

	// Send STOP condition
	AD5172_Stop();	
}

// This routine reads the data registers for whichever pot was selected from the previous WRITE
int AD5172_Read(void)
{
int dataByte, temp;
unsigned char status;

	// errorFlag = 0x00;	// Clear error flag
	// Send START condition and AD5172 address OR'd with READ command:
	AD5172_Start();
	AD5172_SendByte(0x5E|I2C_READ);
	errorFlag = errorFlag | get_Acknowledge();

	// Now read the first byte returned from the AD5172.
	// For some reason, the data sheet refers to this byte as the "Instruction Byte"
	// even though it is actually the pot data 
	// as opposed to an instruction byte:
	dataByte = AD5172_ReceiveByte();
	send_Acknowledge(0);// Send ACK
	
	// Now read OTP validation bits, followed by acknowledge and STOP:
	temp = AD5172_ReceiveByte(); // Get E0 and E1 OTP validation bits: see data sheet for AD5172	
	status = temp & 0xC0; // Mask off E0 and E1
	if(0x00==status)OTP_Validation_Flag=FUSE_READY; 		// Not programmed yet
	else if(0xC0==status)OTP_Validation_Flag=FUSE_BLOWN;	// Successful program
	else OTP_Validation_Flag=FUSE_BAD;					// Fatal program error
	send_Acknowledge(1);// Send NO ACK
	AD5172_Stop();	
	return((int)dataByte);
}

/*********************** NEW ROUTINES FOR WRITING TO SLAVE PIC ******************************/

// Writes bytes to a SLAVE device
void i2c_SlaveWrite(unsigned char device, unsigned char numDataBytes) {
unsigned char i, dataByte, WCOLflag;

	errorFlag = 0x00;	// Clear error flag

	// Send START condition and WRITE command:
	AD5172_Start();
	//i2c_SendControlByte(device, I2C_WRITE);	
	AD5172_SendByte(device|I2C_WRITE);
	errorFlag = errorFlag | get_Acknowledge();

	// Now send DATA array, and get acknowledges following each byte
	for(i=0; i<numDataBytes; i++)
	{
		dataByte = outBuffer[i];
		AD5172_SendByte(dataByte);				
		errorFlag = errorFlag | get_Acknowledge();
		//DelayMs(10);
		WCOLflag=WCOL;
	}

	// Send STOP condition
	AD5172_Stop();
}

// Reads bytes from SLAVE device. Number of bytes is set by numBytes
void i2c_SlaveRead(unsigned char device, unsigned char numBytes) {
unsigned char i;

	errorFlag = 0x00;	// Clear error flag

	// Send START condition and READ command:
	AD5172_Start();	
	AD5172_SendByte(device|I2C_READ);
	errorFlag = errorFlag | get_Acknowledge();
	
	i = 0;	
	do {
		// Now read each DATA byte, followed by acknowledge:
		inBuffer[i] = AD5172_ReceiveByte();
		i++;				

		if (i < numBytes)
			send_Acknowledge(SEND_ACK);			// Send ACK
		else
			send_Acknowledge(SEND_NACK);		// Send NACK: leave scknowledge high for last byte				
	} while (i < numBytes);
	AD5172_Stop();	
}


