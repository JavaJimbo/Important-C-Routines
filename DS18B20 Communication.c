/***********************************************************************************************************
 * DS18B20 Communication  
 * Written for PIC 32MX795 using XC32 compiler V 1.30
 * Uses PORTB bit 2 for one wire communication
 * The DS18B20 requires about 750 ms for temp conversion.
 * 
 * Contains routines for communicating and measuring temperature
 * using one wire bus plus power and ground.
 * Bus is referred to as "DQ"
 * Assumes only one device is on bus.
 * 09-22-17 JBS: Measures temperature, works well. 
 ************************************************************************************************************/

#define true TRUE
#define false FALSE
#define HOSTuart UART2

/** INCLUDES *******************************************************/
#include <XC.h>
#include "Delay.h"
#include "GenericTypeDefs.h"
#include "Compiler.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON        // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier $$$$
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select

#define DQin  PORTBbits.RB2             // Use PORTB bit 2 for DQ bus


/** PRIVATE PROTOTYPES *********************************************/
static void InitializeSystem(void);
void UserInit(void);



unsigned char MicroSecondTimerDelay(unsigned short timeout, unsigned char DQwaitFlag);
unsigned char WaitForTristate(void);
unsigned short readDQ(void);
void sendByteToDS18B20(unsigned char dataByte);
void sendBitToDS18B20(unsigned char bitState);
unsigned char resetDS18B20(void);
unsigned char readDQbit(void);
unsigned char readDQbyte(void);
unsigned char waitForTempConversion(void);
short readTemperature(void);

int main(void) {
    short i = 0;
    
    UserInit();
    DelayMs(10);
    printf("\r\rTESTING DS TEMP");

    while(1) printf("\r#%d T: %d", i++, readTemperature());
}

// Reads temperature from DS18B20 in Celsius.
// Returns 12 bit integer times ten.
// Assumes temperature is positive, meaning > 0 degrees C.
// Assumes only one device is on bus.
short readTemperature(void){
short temperature;
unsigned char tempLowByte, tempHighByte; 

        resetDS18B20();                 // Start DS18B20 transaction sequence
        sendByteToDS18B20(0xCC);        // Send SKIP ROM command    
        sendByteToDS18B20(0x44);        // Send CONVERT T command
        if (!waitForTempConversion()){  // Wait for DS18B20 to send a 1, indicating conversion is done
            printf("\rT conversion error: no response from DS18B20");
            return(0);
        }
        
        resetDS18B20();                 // Start another transaction sequence
        sendByteToDS18B20(0xCC);        // Send SKIP ROM command
        sendByteToDS18B20(0xBE);        // Send READ SCRATCHPAD command
        tempLowByte = readDQbyte();     // Read temperature high and low bytes
        tempHighByte = readDQbyte();    // OR bytes and multiply times 0.625 to get temp in Celsius time ten
        temperature = tempLowByte + (tempHighByte << 8);
        temperature = (temperature * 5) / 8;
        return(temperature);
}

// Read in eight data bits from the DS18B20, LSB first
unsigned char readDQbyte(void){
unsigned char dataByte = 0x00;
unsigned short i;    
    
    for (i = 0; i < 8; i++){   
        dataByte = dataByte >> 1;
        if (readDQbit()) dataByte = dataByte | 0x80;                
    }
    return(dataByte);
}

// Reads one data bit from DS18B20
// by monitoring DQ bus. 
// Returns 0 if bus remains low > 60 microseconds,
// otherwise returns 1. In either case, 
// a read requires about 80 microseconds.
unsigned char readDQbit(void){
unsigned char bitValue = 1;

    // Pull DQ low for a couple microseconds
    PORTSetPinsDigitalOut(IOPORT_B, BIT_2);
    PORTClearBits(IOPORT_B, BIT_2);    

    // Then tristate bus,
    PORTSetPinsDigitalIn(IOPORT_B, BIT_2);
    
    // Now monitor to see when DQ goes high.
    // If DQ remains low more then 60 microseconds,
    // then incoming bit is a 0.
    // Otherwise if bit is a 1,
    // bus should go high immediately.
    if (WaitForTristate() > 60) bitValue = 0;  
    
    // Allow total of 80 microseconds for each bit
    while (!mT4GetIntFlag());    
    return(bitValue);
}

// This routines is called after sending 
// a temperature conversion command
// to the DS18B20. It monitors the DQ bus
// and returns true when the device sends a 1,
// indicating conversion is complete.
// Otherwise it times out in 8 seconds.
unsigned char waitForTempConversion(void){
unsigned long i = 0;

    i = 100000;
    do{
        if (readDQbit()) break; // Keep reading DQ bus to see when device sends a 1
    } while(i--);    
    if(i) return(true);     // Success! Temp conversion complete
    else return(false);     // Something wrong: no conversion after 8 seconds
}

// Send RESET pulse on DQ bus to wake up DS18B20
// This must be done at beginning 
// of every communication sequence.
// Returns true if device responds.
unsigned char resetDS18B20(void) {
    // Send RESET pulse by pulling DQ bus low for 500 uS
    PORTSetPinsDigitalOut(IOPORT_B, BIT_2);
    PORTClearBits(IOPORT_B, BIT_2);
    MicroSecondTimerDelay(500, false);

    // Tristate and pause 10 uS before polling bus:
    PORTSetPinsDigitalIn(IOPORT_B, BIT_2);
    MicroSecondTimerDelay(10, false);

    // Wait for response from DS18B20
    // The device should pull DQ low within 600 uS     
    if (MicroSecondTimerDelay(600, true)) {      
        // After the DS18B20 pulls DQ low, 
        // wait until 600 uS has elapsed:
        while (!mT4GetIntFlag()); 
        return (true); // Success
    } 
    // Otherwise if the DS18B20 doesn't respond 
    // within 600 uS, indicate error:
    else return (false);
}

// Send eight bits to the DS18B20, LSB first
void sendByteToDS18B20(unsigned char commandByte) {    
    unsigned short i;

    for (i = 0; i < 8; i++) {
        if (commandByte & 0x01) sendBitToDS18B20(1);
        else sendBitToDS18B20(0);
        commandByte = commandByte >> 1;
    }
}

// Send a 1 or a 0 to the DS18B20
void sendBitToDS18B20(unsigned char bitState) {
    // Set DQ pin to digital out and set it low:
    PORTSetPinsDigitalOut(IOPORT_B, BIT_2);
    PORTClearBits(IOPORT_B, BIT_2);

    // If desired bit is a '0', keep DQ low for 60 microseconds
    if (!bitState) {
        MicroSecondTimerDelay(60, false);
        PORTSetPinsDigitalIn(IOPORT_B, BIT_2);
        MicroSecondTimerDelay(10, false);
    }
        // otherwise if bit is a '1', keep DQ low for only 6 microseconds
    else {
        MicroSecondTimerDelay(6, false);
        PORTSetPinsDigitalIn(IOPORT_B, BIT_2);
        MicroSecondTimerDelay(64, false);
    }
}

// This routine creates delays for DS18B20 communication.
// It creates a timeout in microseconds.
// If DQ flag is true, then the DQ bus is monitored
// and if device responds before timeout,
// it returns true to indicate success.
unsigned char MicroSecondTimerDelay(unsigned short timeout, unsigned char DQflag) {
    T4CONbits.TON = 0; // Stop timer to set registers
    PR4 = timeout * 10;
    TMR4 = 0x0000;
    mT4ClearIntFlag();
    T4CONbits.TON = 1; // Let her rip      
    
    // Wait for timeout
    while (!mT4GetIntFlag()) {        
        if (DQflag && DQin == 0)    // Monitor bus if desired             
            return (true);          // return true if device responds
    }
    return (false);                 // Device hasn't responded within timeout period
}

// This routine measures length of low pulses on DQ bus.
// When DS releases DQ and it goes high,
// the length of the low pulse is returned
// in microseconds times 10.
// It allows up to 80 microseconds for a response
unsigned char WaitForTristate(void) {
    T4CONbits.TON = 0;  // Stop timer to set registers
    PR4 = 800;         // Set timeout for 80 uS.
    TMR4 = 0x0000;      // Clear timer & flag
    mT4ClearIntFlag();  
    T4CONbits.TON = 1;  // Let her rip      
    while (DQin == 0 && !mT4GetIntFlag());    // Wait for DQ to go high or timeout, whichever comes first.
    return(TMR4);       // Return elapsed time of low pulse    
}

void UserInit(void) {

    // Set up main UART
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
#define SYS_FREQ 80000000
    UARTSetDataRate(HOSTuart, SYS_FREQ, 57600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_DISABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);

    //PORTSetPinsDigitalIn(IOPORT_C, BIT_14 | BIT_13);
    PORTSetPinsDigitalIn(IOPORT_B, BIT_2);
    // PORTSetBits(IOPORT_B, BIT_2);

    // Set up Atmel CS output:
    PORTSetPinsDigitalOut(IOPORT_C, BIT_3);
    PORTSetBits(IOPORT_C, BIT_3); // Set high to deselect

    // Set up Atmel WR protect:
    PORTSetPinsDigitalOut(IOPORT_E, BIT_8);
    PORTSetBits(IOPORT_E, BIT_8); // Set high to enable writes

    // Set up Timer 4 for 10 Khz (100 uS) rollovers
    // Do NOT enable interrupts!
    T4CON = 0x00;
    T4CONbits.TCKPS2 = 0; // 1:8 Prescaler    
    T4CONbits.TCKPS1 = 1;
    T4CONbits.TCKPS0 = 1;
    T4CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    T4CONbits.TON = 0; // Timer remains off until needed      

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

}//end UserInit


