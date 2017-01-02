// Written by Jim Sedgwick, 12/22/2005

#define POT_1 		0x00 				// This command writes to the AD5172 pot #1
#define POT_2 		0x80 				// This command writes to the AD5172 pot #2
#define SET_FUSE	0x

#define AD5172_SLAVE_ADDRESS 0x5E
#define I2C_READ	0x01		// read bit used with address 
#define I2C_WRITE	0x00		// write bit used with address 
#define TIMEOUT		0x80		// Timeout error for I2C

#define I2C_ERROR	(-1)
#define SEND_NACK	1			// Send Nack: no more bytes to send 
#define SEND_ACK	0			// Send Ack: more bytes to send 

extern void AD5172_Write(unsigned char potSelect, unsigned char dataByte);
extern int AD5172_Read(void);
extern void initialize_I2C(void);

extern void AD5172_Stop(void);
extern void AD5172_Start(void);
extern void AD5172_Restart(void);
extern void AD5172_SendByte(unsigned char byte);

extern void AD5172_Program(unsigned char potSelect);
extern void AD5172_Set(unsigned char potSelect);
extern int AD5172_ReceiveByte(void);

extern void i2c_SlaveRead(unsigned char device, unsigned char numBytes);
extern void i2c_SlaveWrite(unsigned char device, unsigned char numDataBytes);

extern void send_Acknowledge(unsigned char ack);
extern unsigned char get_Acknowledge(void);


