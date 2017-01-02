/* AT45DB161.h: Header file for routines for reading and writing to ATMEL AT45DB161 memory
 *
 * 2-11-12 JBS: 	Retooled version with latest corrections
 * 5-01-13 JBS:  	Added write and read line routines.
 * 5-26-14 JBS:		Added initSPI(), WriteAtmelPage(), ReadAtmelPage()
 * 12-16-14             Added initAtmelSPI() and ATMEL_SPI_CHANNEL below. 
 *                      Works with LED Controller Board. initSPI() not working.
 */
#define PAGESIZE 528
#define ARRAYLENGTH 21
#define ATMEL_PAGESIZE 528
#define BUFFER_OVERRRUN 1
#define ARRAY_OVERRRUN 2
#define MAX_PAGE 4095
#define MAX_SECTOR 15
#define ERROR 1
#define LINESIZE 16
#define MAX_ATMEL_LINE 33

#define ATMEL_WRITE_PROTECT PORTEbits.RE8
#define ATMEL_CS PORTCbits.RC3  
#define ATMEL_SPI_CHANNEL 2

#define SPI_START_CFG_A     (PRI_PRESCAL_1_1 | SEC_PRESCAL_1_1 | MASTER_ENABLE_ON | SPI_CKE_ON | SPI_SMP_ON)
#define SPI_START_CFG_B     (SPI_ENABLE)
#define initAtmelSPI() OpenSPI(SPI_START_CFG_A, SPI_START_CFG_B) // Initialize SPI #1 for Atmel

// void initSPI(void);
int SendReceiveSPI(unsigned char dataOut);
unsigned char ProgramPage (unsigned short page);
unsigned char TransferPage (unsigned short page);
unsigned char ErasePage (unsigned short page);

int EraseSector(unsigned char sector);
int AtmelBusy(unsigned char waitFlag);
int EraseEntireChip(void);

int WriteAtmelPage (unsigned char *buffer);
int ReadAtmelPage (unsigned char *buffer);

int WriteAtmelBuffer (unsigned char *buffer, unsigned int bufferAddress, unsigned int numberOfBytes);
int ReadAtmelBuffer (unsigned char *buffer, unsigned int bufferAddress, unsigned int numberOfBytes);

int WriteAtmelLine (unsigned char *buffer, unsigned int lineNumber);
int ReadAtmelLine (unsigned char *buffer, unsigned int lineNumber);




