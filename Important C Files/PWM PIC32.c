// From: http://electronics.stackexchange.com/questions/69232/pic32-pwm-minimal-example
#include <stdio.h>
#include <stdlib.h>
#include "p32xxxx.h"
#include "plib.h"
#define SYSTEM_FREQ_HZ 80000000
#pragma config FPLLODIV = DIV_1, FSOSCEN = OFF, FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FWDTEN = OFF, FPBDIV = DIV_1, POSCMOD = XT, FNOSC = PRIPLL, CP = OFF
#pragma config FMIIEN = ON, FETHIO = OFF, FUSBIDIO = OFF, FVBUSONIO = OFF   // external PHY in RMII/alternate configuration
#pragma config UPLLEN = ON,UPLLIDIV = DIV_2
/*
 * 
 */
int main(void)
{
    SYSTEMConfig(SYSTEM_FREQ_HZ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

    OpenOC1( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenTimer2( T2_ON | T2_PS_1_1 | T2_SOURCE_INT, 0xFFFF);
    unsigned short a;
    int b;
    while(1)
    {
        for(b=0;b<100;b++)
            Nop();
        SetDCOC1PWM(a++);
    }
}