// EEPROM_I2C - written by Jim Sedgwick
// Contains routines for reading and writing to external EEPROM using PIC I2C feature
// EEPROM is at fixed hardware address of 0xA0 as defined in EEPROM_I2C.h
// Other EEPROMs may be added also to share same SCL and SDA lines.
//
// Global variable errorFlag is set to "1" if am acknowledge error occurs
// outDataBlock[] is used for storing outgoing data to be written to EEPROM
// EEPROMblock[] is used for storing incoming data read from EEPROM

// Revision history
//	2006-1-13 JBS: small modifications to prevent EEPROMblock[] from being overrun.
//  2006-4-11 JBS: added "device" argument to read and write routines to access multiple EEPROMs on fixture board.
//	2007-12-15 JBS:	fixed bugs in acknowledge routines. Was reading wrong acknowledge flag. 
//					Created new i2c_GetAcknowledge() routine which reads ACKSTAT flag.

#include 	<pic18.h>
#include 	"DELAY16.H"
#include 	"Humidity Cal Master.h"
#include 	"EEPROM_I2C.h"
#include	"stdio.h"

extern unsigned char errorFlag;
extern unsigned char EEPROMbuffer[];

//	This routine sets up I2C functions for master mode

void initialize_I2C (void)
{
	// 0010 1000B = 0x28
	SSPCON1 = 0x28; 	// SSPEN = 1, SDA and SCL for I2C
					// I2C Master mode = Fosc/ (4 * (SSPADD + 1))
	SSPADD = 9;	// FBUS = OSC/4 /(N+1)  OSC = 4, OCS =0.4; 10 IS JUST RIGHT
	SSPSTAT = 0;	// Clear MSSP status register

	// Make sure I2C is STOPPED:
	i2c_Stop();
}


//  Send stop condition
// 	  - data low-high while clock high

void
i2c_Stop(void)
{
	PEN = 1;			// Send STOP condition
	while (SSPIF == 0); // WAIT TILL SSPIF SET IN PIR REGISTER
	SSPIF = 0;			// CLEAR FLAG
}

//	Send start condition

void
i2c_Start(void)
{
	SEN = 1;			// Send START condition
	while (SSPIF == 0); // WAIT TILL SSPIF SET IN PIR REGISTER
	SSPIF = 0;			// CLEAR FLAG
}


//	Send restart condition

void
i2c_Restart(void)
{
	RSEN = 1;			// Send REPEATED START condition
	while (SSPIF == 0); // WAIT TILL SSPIF SET IN PIR REGISTER
	SSPIF = 0;			// CLEAR FLAG
}

//	Send a byte to the slave
// 	  - returns true on error

unsigned char
i2c_SendByte(unsigned char byte)
{

	SSPBUF = byte;		// Send byte
	while (SSPIF == 0); // WAIT TILL SSPIF SET IN PIR REGISTER
	SSPIF = 0;			// CLEAR FLAG	

	return FALSE;
}

//	send control byte and data direction to the slave
//  	- 7-bit control byte (lsb ignored)
// 	  	- direction (0 = write )

unsigned char
i2c_SendControlByte(unsigned char controlByte, unsigned char direction)
{
unsigned char outByte;

		outByte = controlByte | direction;
        return i2c_SendByte (outByte);
}

//	Check for an acknowledge from slave EEPROM after sending it a command or data
// 	  - returns ack or ~ack
unsigned char i2c_GetAcknowledge(void)
{
unsigned char ack;

	ack = ACKSTAT;
	return ack;
}



int
i2c_ReadByte(void)
{
unsigned char byte;
	
	RCEN = 1;	// Enable receive mode for I2C
	while (SSPIF == 0); // WAIT TILL SSPIF SET IN PIR REGISTER
	SSPIF = 0;			// CLEAR FLAG	
	byte = SSPBUF;
	return (int)byte;
}

//	Send an (~)acknowledge to the slave
//		- status of I2C_LAST implies this is the last byte to be sent
//    	- if there are more bytes, then ACK must be brought low
//		- if there are no more bytes, then ACK is left high
//
// Returns nothing

void
i2c_SendAcknowledge(unsigned char status)
{
        if ( status & 0x01) {
 			ACKDT = 0; // drive line low -> more to come
        }
        else { 
 			ACKDT = 1; // line left high -> last expected byte has been received
	}	
	ACKEN = 1;	// Initiate acknowledge sequence
	while (SSPIF == 0); // WAIT TILL SSPIF SET IN PIR REGISTER
	SSPIF = 0;			// CLEAR FLAG
}


// Writes one byte to a single EEPROM address

void EepromWriteByte(unsigned char device, unsigned int address, unsigned char dataByte) {
unsigned char addressHigh, addressLow;

	errorFlag = 0x00;	// Clear error flag

	addressHigh = (unsigned char) ((address & 0xFF00) >> 8);
	addressLow = (unsigned char) (address & 0x00FF);

	// Send START condition and WRITE command:
	i2c_Start();
	i2c_SendControlByte(device, I2C_WRITE);
	errorFlag = errorFlag | i2c_GetAcknowledge();

	// Send two byte EEPROM address:
	i2c_SendByte(addressHigh);
	errorFlag = errorFlag | i2c_GetAcknowledge();
	i2c_SendByte(addressLow);
	errorFlag = errorFlag | i2c_GetAcknowledge();

	// Now send DATA byte, then get acknowledge:
	i2c_SendByte(dataByte);
	errorFlag = errorFlag | i2c_GetAcknowledge();

	// Send STOP condition
	i2c_Stop();
}

// Reads one byte from a single EEPROM address
int EepromReadByte(unsigned char device, unsigned int address) {
unsigned char addressHigh, addressLow;
int dataByte;

	errorFlag = 0x00;	// Clear error flag

	addressHigh = (unsigned char) ((address & 0xFF00) >> 8);
	addressLow = (unsigned char) (address & 0x00FF);

	// Send START condition and WRITE command:
	i2c_Start();
	i2c_SendControlByte(device, I2C_WRITE);
	errorFlag = errorFlag | i2c_GetAcknowledge();

	// Send two byte EEPROM address:
	i2c_SendByte(addressHigh);
	errorFlag = errorFlag | i2c_GetAcknowledge();
	i2c_SendByte(addressLow);
	errorFlag = errorFlag | i2c_GetAcknowledge();

	// Now send READ command:
	i2c_Restart();	
	i2c_SendControlByte(device, I2C_READ);
	errorFlag = errorFlag | i2c_GetAcknowledge();

	// Now read DATA byte, followed by STOP condition and no acknowledge:
	dataByte = i2c_ReadByte();
	i2c_SendAcknowledge(I2C_LAST); // No acknowledge because no more data is being read
	i2c_Stop();
	return(dataByte);
}


// Writes a block of bytes to EEPROM beginnning at startAddress
// Number of bytes is set by numBytes

void EepromWriteBlock(unsigned char device, unsigned int startAddress, unsigned char numBytes) {
unsigned char addressHigh, addressLow;
unsigned char i, dataByte;

	if (numBytes <= BUFFERSIZE)
	{
		errorFlag = 0x00;	// Clear error flag

		addressHigh = (unsigned char) ((startAddress & 0xFF00) >> 8);
		addressLow = (unsigned char) (startAddress & 0x00FF);

		// Send START condition and WRITE command:
		i2c_Start();
		i2c_SendControlByte(device, I2C_WRITE);
		errorFlag = errorFlag | i2c_GetAcknowledge();

		// Send two byte EEPROM address:
		i2c_SendByte(addressHigh);
		errorFlag = errorFlag | i2c_GetAcknowledge();
		i2c_SendByte(addressLow);
		errorFlag = errorFlag | i2c_GetAcknowledge();
		
		i = 0;	
		do {
			// Now send each DATA byte, and read acknowledge from EEPROM:

			dataByte = EEPROMbuffer[i];
			i2c_SendByte(dataByte);
			errorFlag = errorFlag | i2c_GetAcknowledge();
			i++;				
		} while (i < numBytes);

		// Send STOP condition
		i2c_Stop();
		// Delay to transfer data to EEPROM
		DelayMs(8); // Was 8. This is really about 5 milliseconds.				
	}
}


// Reads a block of bytes from EEPROM beginnning at startAddress
// Number of bytes is set by numBytes

void EepromReadBlock(unsigned char device, unsigned int startAddress, unsigned char numBytes) {
unsigned char addressHigh, addressLow;
unsigned char i;

	if (numBytes <= BUFFERSIZE)
	{
		errorFlag = 0x00;	// Clear error flag

		addressHigh = (unsigned char) ((startAddress & 0xFF00) >> 8);
		addressLow = (unsigned char) (startAddress & 0x00FF);

		// Send START condition and WRITE command:
		i2c_Start();
		i2c_SendControlByte(device, I2C_WRITE);
		errorFlag = errorFlag | i2c_GetAcknowledge();

		// Send two byte EEPROM address:
		i2c_SendByte(addressHigh);
		errorFlag = errorFlag | i2c_GetAcknowledge();
		i2c_SendByte(addressLow);
		errorFlag = errorFlag | i2c_GetAcknowledge();

		// Now send READ command:
		i2c_Restart();	
		i2c_SendControlByte(device, I2C_READ);
		errorFlag = errorFlag | i2c_GetAcknowledge();

		i = 0;	
		do {
			// Now read each DATA byte, followed by acknowledge:
			EEPROMbuffer[i] = i2c_ReadByte();
			i++;				

			if (i < numBytes)
				i2c_SendAcknowledge(I2C_MORE);
			else
				i2c_SendAcknowledge(I2C_LAST);	// Leave scknowledge high for last byte	

		} while (i < numBytes);

		i2c_Stop();
	}
}


