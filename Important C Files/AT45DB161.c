/* AT45DB161.C - Routines for reading and writing to ATMEL AT45DB161 memory
 * This is the modified version of AT45DB642.c for the smaller '161 Atmel
 *
 * 02-11-12 JBS:    Retooled version with latest corrections
 * 05-01-13 JBS:    Tested 0-4095 page writes
 *                  Swapped in pointers in place of global arrays
 *                  Added write and read line routines.
 * 5-26-14 JBS:     Adapted for PIC32 & MPLABX.
 *                  Added initSPI(), WriteAtmelPage(), ReadAtmelPage()
 * 6-6-14           INVERTED return values so functions return TRUE or FALSE instead of error.
 * 12-16-14         Works with LED COntroller Board - SPI #2
 *                  initSPI() commented out below because it isn't working
 */

#define _SUPPRESS_PLIB_WARNING
#include <plib.h>
#include "AT45DB161.h"

#define FALSE 0
#define TRUE !FALSE

void initSPI(void){
    SpiChnOpen(ATMEL_SPI_CHANNEL, SPI_CON_MODE8|SPI_CON_ON, 4);
}

// This version uses the PIC SPI port fro PIC32's
int SendReceiveSPI(unsigned char dataOut){
int dataIn;

	SpiChnPutC(ATMEL_SPI_CHANNEL, dataOut);
	dataIn =SpiChnGetC(ATMEL_SPI_CHANNEL);

	return(dataIn);
}

int WriteAtmelLine (unsigned char *buffer, unsigned int lineNumber){		
unsigned char byteHIGH = 0x00;
unsigned char byteLOW = 0x00;
unsigned int tempAddress;
int i;

	if (buffer==NULL) return(FALSE); // ERROR
	if (lineNumber>=MAX_ATMEL_LINE) return(FALSE); // ERROR
	
	tempAddress = lineNumber * LINESIZE;
	
	byteLOW = (unsigned char)(tempAddress & 0x00FF);
	tempAddress = tempAddress >> 8;
	byteHIGH = (unsigned char)(tempAddress & 0x00FF);

	AtmelBusy(1);						// Read Status register and make sure Atmel isn't busy with any previous activity.	

	ATMEL_CS=0;							// select EEPROM as target
	SendReceiveSPI(0x84);			// Command: Buffer #1 write
	SendReceiveSPI(0);				// Address
	SendReceiveSPI(byteHIGH);
	SendReceiveSPI(byteLOW);		
	
	for(i=0; i<LINESIZE; i++)	// write line to Atmel buffer	
		SendReceiveSPI(buffer[i]);	
	ATMEL_CS=1;		
	return(TRUE); // No errors - write was successful
}

// This function writes an entire 528 byte page to the Atmel memory buffer.
// *buffer points to input array.
int WriteAtmelPage (unsigned char *buffer){
int i;

 	if (buffer==NULL) return(FALSE); // ERROR

	AtmelBusy(1);						// Read Status register and make sure Atmel isn't busy with any previous activity.

	ATMEL_CS=0;							// select EEPROM as target
	SendReceiveSPI(0x84);			// Command: Buffer #1 write
	SendReceiveSPI(0);				// Address
	SendReceiveSPI(0);
	SendReceiveSPI(0);

	for(i=0; i<ATMEL_PAGESIZE; i++)	// write the data to Atmel buffer
		SendReceiveSPI(buffer[i]);
	ATMEL_CS=1;
	return(TRUE); // No errors - write was successful
}



// This function writes to the Atmel memory buffer.
// *buffer points to input array.	
// The "numberOfBytes" input is the number of bytes that will be 
// copied from the AtmelWriteArray[] array to the Atmel.
// If there are no errors, it returns TRUE
int WriteAtmelBuffer (unsigned char *buffer, unsigned int bufferAddress, unsigned int numberOfBytes){			
unsigned char byteHIGH = 0x00;
unsigned char byteLOW = 0x00;
unsigned int tempAddress;
int i;
 
 	if (buffer==NULL) return(FALSE); // ERROR
 
	if ((bufferAddress+numberOfBytes) >= ATMEL_PAGESIZE) return(BUFFER_OVERRRUN); // Make sure we don't overrun the Atmel page buffer
	if (numberOfBytes > ARRAYLENGTH) return(FALSE);  // Make sure we don't overrun the AtmelWriteArray[]

	tempAddress = bufferAddress;
	byteLOW = (unsigned char)(tempAddress & 0x00FF);
	tempAddress = tempAddress >> 8;
	byteHIGH = (unsigned char)(tempAddress & 0x00FF);

	AtmelBusy(1);						// Read Status register and make sure Atmel isn't busy with any previous activity.	

	ATMEL_CS=0;							// select EEPROM as target
	SendReceiveSPI(0x84);			// Command: Buffer #1 write
	SendReceiveSPI(0);				// Address
	SendReceiveSPI(byteHIGH);
	SendReceiveSPI(byteLOW);		
	
	for(i=0; i<numberOfBytes; i++)	// write the data to Atmel buffer	
		SendReceiveSPI(buffer[i]);	
	ATMEL_CS=1;		
	return(TRUE); // No errors - write was successful
}

	

/* For PIC 18
// This version uses the PIC SPI port
int SendReceiveSPI(unsigned char dataOut)
{
int dataIn;

	SSPBUF=dataOut;			// send data out to SPI device
	while(BF==0);			// wait for eprom to return data
	dataIn=(int)SSPBUF;		// Clears BF, data may be good or useless
	return(dataIn);
}
*/

int ReadAtmelLine (unsigned char *buffer, unsigned int lineNumber){
unsigned char byteHIGH = 0x00;
unsigned char byteLOW = 0x00;
unsigned int tempAddress;
unsigned char dataIn;
int i;

	if (buffer==NULL) return(FALSE); // ERROR
	if (lineNumber>=MAX_ATMEL_LINE) return(FALSE); // ERROR

	tempAddress = lineNumber * LINESIZE;	
	
	byteLOW = (unsigned char)(tempAddress & 0x00FF);
	tempAddress = tempAddress >> 8;
	byteHIGH =(unsigned char)(tempAddress & 0x00FF);	

	AtmelBusy(1);						// Read Status register and make sure Atmel isn't busy with any previous activity.	
		
	ATMEL_CS=0;						
	SendReceiveSPI(0xD4);			// Buffer #1 read command
	SendReceiveSPI(0);
	SendReceiveSPI(byteHIGH);
	SendReceiveSPI(byteLOW);
	SendReceiveSPI(0);				// Additional don't care byte	
	
	for(i=0; i<LINESIZE; i++){
		dataIn = (unsigned char) SendReceiveSPI(0x00);
		buffer[i] = dataIn;		
	}
	ATMEL_CS=1;
	return(TRUE); // No errors - read was successful
}



int ReadAtmelPage (unsigned char *buffer){
unsigned char dataIn;
int i;

	AtmelBusy(1);						// Read Status register and make sure Atmel isn't busy with any previous activity.

	ATMEL_CS=0;
	SendReceiveSPI(0xD4);			// Buffer #1 read command
	SendReceiveSPI(0);
	SendReceiveSPI(0);
	SendReceiveSPI(0);
	SendReceiveSPI(0);				// Additional don't care byte

	for(i=0; i<ATMEL_PAGESIZE; i++){
		dataIn = (unsigned char) SendReceiveSPI(0x00);
		buffer[i] = dataIn;
	}
	ATMEL_CS=1;
	return(TRUE); // No errors - read was successful
}


int ReadAtmelBuffer (unsigned char *buffer, unsigned int bufferAddress, unsigned int numberOfBytes){
unsigned char byteHIGH = 0x00;
unsigned char byteLOW = 0x00;
unsigned int tempAddress;
unsigned char dataIn;
int i;

	if ((bufferAddress+numberOfBytes) >= ATMEL_PAGESIZE) return(FALSE); // Make sure we don't overrun the Atmel page buffer
	if (numberOfBytes > ARRAYLENGTH) return(FALSE);  // Make sure we don't overrun the AtmelReadArray[]

	tempAddress = bufferAddress;
	byteLOW = (unsigned char)(tempAddress & 0x00FF);
	tempAddress = tempAddress >> 8;
	byteHIGH =(unsigned char)(tempAddress & 0x00FF);	

	AtmelBusy(1);						// Read Status register and make sure Atmel isn't busy with any previous activity.	
		
	ATMEL_CS=0;						
	SendReceiveSPI(0xD4);			// Buffer #1 read command
	SendReceiveSPI(0);
	SendReceiveSPI(byteHIGH);
	SendReceiveSPI(byteLOW);
	SendReceiveSPI(0);				// Additional don't care byte	
	
	for(i=0; i<numberOfBytes; i++){
		dataIn = (unsigned char) SendReceiveSPI(0x00);
		buffer[i] = dataIn;		
	}
	ATMEL_CS=1;
	return(TRUE); // No errors - read was successful
}


// This programs the buffer into a previously erased page in flash memory.
// The pages are numbered 0-4095.
// The routine always returns 0, however it can be modified to return an error code as well.
unsigned char ProgramPage (unsigned short page){	
unsigned char byteHIGH = 0x00;
unsigned char byteLOW = 0x00;
unsigned int tempAddress;

	tempAddress = page << 2;									// The '161 page gets shifted down  2 to get six lowest page bits.
	byteLOW = (unsigned char)(tempAddress & 0x00FC); 	// Mask off two lowest bits.
	tempAddress = page >> 6;									// Shift down 6 to get upper six bits.
	byteHIGH = (unsigned char)(tempAddress & 0x003F);	// Mask off two highest bits.

	AtmelBusy(1);					// Read Status register and make sure Atmel isn't busy with any previous activity.	

	ATMEL_CS=0;						// select eprom as target
	SendReceiveSPI(0x88);			// Page program without built-in erase
	SendReceiveSPI(byteHIGH);		// Page high byte
	SendReceiveSPI(byteLOW);		// Page low byte	
	SendReceiveSPI(0);				// 
	ATMEL_CS=1;						// disable eprom as target
	
	return(TRUE);
}


// This copies a page into Atmel Buffer 1
// The pages are numbered 0-4095.
// The routine returns FALSE page is out of bounds, otherwise returns TRUE
unsigned char TransferPage (unsigned short page){
unsigned char byteHIGH = 0x00;
unsigned char byteLOW = 0x00;
unsigned int tempAddress;

	if (page>MAX_PAGE) return(FALSE);
        AtmelBusy(1);                                           // Make sure Atmel isn't busy

	tempAddress = page << 2;									// The '161 page gets shifted down  2 to get six lowest page bits.
	byteLOW = (unsigned char)(tempAddress & 0x00FC); 	// Mask off two lowest bits.
	tempAddress = page >> 6;									// Shift down 6 to get upper six bits.
	byteHIGH = (unsigned char)(tempAddress & 0x003F);	// Mask off two highest bits.

	AtmelBusy(1);					// Read Status register and make sure Atmel isn't busy with any previous activity.	

	ATMEL_CS=0;						
	SendReceiveSPI(0x53);			// Command
	SendReceiveSPI(byteHIGH);		// Page high byte
	SendReceiveSPI(byteLOW);		// Page low byte	
	SendReceiveSPI(0);				// 
	ATMEL_CS=1;	
	
	return(TRUE);
}

// This erases one page in flash memory.
// The pages are numbered 0-4095.
// The routine returns ERROR of page is out of bounds, otherwise returns 0.
unsigned char ErasePage (unsigned short page){	
unsigned char byteHIGH = 0x00;
unsigned char byteLOW = 0x00;
unsigned short tempAddress;

	if (page>MAX_PAGE) return(FALSE);

	tempAddress = page << 2;				// The '161 page gets shifted down  2 to get six lowest page bits.
	byteLOW = (unsigned char)(tempAddress & 0x00FC); 	// Mask off two lowest bits.
	tempAddress = page >> 6;                                // Shift down 6 to get upper six bits.
	byteHIGH = (unsigned char)(tempAddress & 0x003F);	// Mask off two highest bits.

	AtmelBusy(1);                                           // Make sure Atmel isn't busy

	ATMEL_CS=0;						// select Atmel as target
	SendReceiveSPI(0x81);		// Page erase
	SendReceiveSPI(byteHIGH);	// Page high byte
	SendReceiveSPI(byteLOW);	// Page low byte	
	SendReceiveSPI(0x00);		// Don't care byte		
	ATMEL_CS=1;						// disable Atmel as target
	
	return(TRUE);
}

// This erases one sector or 256 pages on the Atmel.
// Sectors 1 to 15 can be erased using this routine.
// Sectors 0a and 0b must be erased using a separate routine.
// The routine returns FALSE if sector number is out of bounds, otherwise returns TRUE
int EraseSector(unsigned char sector){	
unsigned char sectorByte;

	if (sector>MAX_SECTOR) return(FALSE);
	sectorByte = (sector<<2) & 0b00111100;	  

	AtmelBusy(1);						// Read Status register and make sure Atmel isn't busy with any previous activity.

	ATMEL_CS=0;						
	SendReceiveSPI(0x7C);			// erase sector command
	SendReceiveSPI(sectorByte);		// sector shifted up two bits
	SendReceiveSPI(0x00);			// Don't care byte
	SendReceiveSPI(0x00);			// Don't care byte		
	ATMEL_CS=1;					
		
	return(TRUE);
}

// This routine erases the entire flash memeory
int EraseEntireChip(void){	
	AtmelBusy(1);	// Read Status register and make sure Atmel isn't busy with any previous activity.

	ATMEL_CS=0;						
	SendReceiveSPI(0xC7);
	SendReceiveSPI(0x94);	
	SendReceiveSPI(0x80);	
	SendReceiveSPI(0x9A);		
	ATMEL_CS=1;					
		
	return(TRUE);
}



// This routine reads the Atmel memory status register
// to check the READY/BUSY flag.
// This flag goes high when a page programming operation 
// has completed. It returns TRUE if complete
// and FALSE otherwise.
//
// If waitFlag is a 1 then it will sit in a loop
// and keep going until the status register
// READY/BUSY flag reads high.
int AtmelBusy (unsigned char waitFlag){	
unsigned char status, inByte;
int i;
		
	ATMEL_CS=0;				
	SendReceiveSPI(0xD7);	// Send read Status register command		

	// Read status byte from Atmel and mask off MSB. 
	// This is the READ/BUSY bit:
	do {
		inByte = (unsigned char) SendReceiveSPI(0x00);
		status = (0x80 & inByte); 
	} while ((0==status)&&(waitFlag));  // Keep looping until status bit goes high, or quit after one loop if waitFlag is false

	ATMEL_CS=1;

	if(0==status)
		return(FALSE);
	else
		return(TRUE);		// Retuirn TRUE if Atmel isn't busy
}
