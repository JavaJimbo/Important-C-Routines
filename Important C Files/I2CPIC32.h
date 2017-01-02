/* 
 * File:   I2PIC32.h adapted from MMA8452.h 5-12-16
 *
 *	Minimum initialization sequence:
 
 *   I2CSetFrequency(EEPROM_I2C_BUS, GetPeripheralClock(), I2C_CLOCK_FREQ);
 *   I2CEnable(EEPROM_I2C_BUS, TRUE);
 *   I2CStop(EEPROM_I2C_BUS);
 * 
 */

#ifndef I2PIC32_H
#define	I2PIC32_H

// #include "GenericTypeDefs.h"

#define I2C_CLOCK_FREQ             5000
#define EEPROM_I2C_BUS              I2C3
#define EEPROM_ADDRESS              0x1D

extern BOOL TransmitOneByte(unsigned char data);
extern BOOL StartTransfer(BOOL restart);
extern void StopTransfer(void);
// extern unsigned char initMMA8452(void);
extern unsigned char setRegister(unsigned char deviceID, unsigned char deviceREGISTER);
extern unsigned char sendREADcommand(unsigned char deviceID);
// extern unsigned char readRegisters(unsigned char deviceID, unsigned char deviceREGISTER, unsigned char numRegisters, unsigned char *registerPtr);
extern unsigned char writeByteToRegister(unsigned char deviceID, unsigned char deviceREGISTER, unsigned char dataByte);
short convertValue(unsigned char MSBbyte, unsigned char LSBbyte);

#endif	

