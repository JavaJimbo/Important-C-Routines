// Written by Jim Sedgwick, 12/15/2007

#define EEPROM 0xA0 			// This addresses the external EEPROM that holds the PIC program code

#define I2C_READ	0x01		/* read bit used with address */
#define I2C_WRITE	0x00		/* write bit used with address */

#define I2C_ERROR	(-1)
#define I2C_LAST	FALSE		/* SendAck: no more bytes to send */
#define I2C_MORE	TRUE		/* SendAck: more bytes to send */


// BUFFERSIZE is equal to the number of bytes in the internal program Write Buffer of the PIC 18F2550.
#define BUFFERSIZE	32				

extern unsigned char 	i2c_GetAcknowledge(void);
extern unsigned char	i2c_SendControlByte(unsigned char, unsigned char);
extern unsigned char	i2c_SendByte(unsigned char);
extern int				i2c_ReadByte(void);
extern void				i2c_Start(void);
extern void				i2c_Restart(void);
extern void				i2c_Stop(void);
extern void				i2c_SendAcknowledge(unsigned char);
extern unsigned char	i2c_read(unsigned char);
extern void 			initialize_I2C (void);
extern unsigned char	i2c_WriteTo(unsigned char controlByte);
extern unsigned char	i2c_ReadFrom(unsigned char controlByte);

extern void 	EepromWriteBlock(unsigned char device, unsigned int startAddress, unsigned char numBytes);
extern void 	EepromReadBlock(unsigned char device, unsigned int startAddress, unsigned char numBytes);
extern void 	EepromWriteByte(unsigned char device, unsigned int address, unsigned char dataByte);
extern int 		EepromReadByte(unsigned char device, unsigned int address);





