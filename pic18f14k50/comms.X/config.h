/* 
 * File:   config.h
 * Author: Bison
 * Comments:
 * Created on 2017/5/10, 12:00
 */

#ifndef COMFIG_H
#define	CONFIG_H 

// PIC18F14K50 Configuration Bit Settings
// 'C' source line config statements

#pragma config PWRTEN = OFF, BOREN = OFF, BORV = 30
#pragma config WDTEN = OFF, WDTPS = 32768
#pragma config STVREN = ON
#pragma config FOSC = HS
#pragma config PLLEN = ON, PCLKEN = ON, CPUDIV = NOCLKDIV, USBDIV = OFF
#pragma config FCMEN = OFF, IESO = OFF, HFOFST = OFF
#pragma config LVP = OFF, XINST = OFF, BBSIZ = OFF
#pragma config CP0 = OFF, CP1 = OFF, CPB = OFF, CPD = OFF
#pragma config WRT0 = OFF, WRT1 = OFF
#pragma config WRTB = OFF, WRTC = OFF, WRTD = OFF
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTRB = OFF

#endif /* CONFIG_H */