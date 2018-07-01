/*
 * File:   stdfunc.c
 * Author: Kinari
 *
 * Created on 2018/05/11, 14:20
 */

#include "stdfunc.h"

void intr_init(){
    INTCONbits.GIE = 0;
    RCONbits.IPEN = 0;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
}

void init(){
    ANSEL = 0;
    ANSELH = 0;
    LATC = 0;
    TRISC = 0;
    TRISB = 0;
    OSCCON2bits.PRI_SD = 1;
    OSCCON = 0;
    intr_init();
}

// EEPROM
void EE_init(){
    EECON1 = 0x00;
    IPR2bits.EEIP = 1;
    PIE2bits.EEIE = 1;
}

unsigned char EE_read(int adrs){
    EEADR = adrs;
    EECON1 = 0x00;
    EECON1bits.RD = 1;
    while(EECON1bits.RD);
    return (EEDATA);
}

void EE_write(int adrs, unsigned char data){
    EEADR = adrs;
    EEDATA = data;
    EECON1bits.WREN = 1;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    while(EECON1bits.WR);
    EECON1bits.WREN = 0;
}

void EE_clear(){
    int address;
    char i = 0;
    for(address = 0; address < EESIZE; address++){
        EE_write(address, i);
    }
}

// ADC
void ADC_init(){
    ANSEL = 0;
    ANSELH = 0b00000010;
    TRISCbits.RC7 = 1; // AN9
    ADCON0 = 0b00100101;
    ADCON1 = 0;
    ADCON2 = 0b00010001;
    IPR1bits.ADIP = 1;
    PIE1bits.ADIE = 1;
}

char ADC_read(){
    ADCON0bits.GO_DONE = 1;
    while(ADCON0bits.GO_DONE);
    char adc = ADRESH;
    return adc;
}

// I2C
void i2c_init(char select){ // 0 : MASTER, else : SLAVE 
    TRISB	= 0b01010000; // RB4 : SDA, RB6 : SCL
    SSPSTAT = 0;
    IPR1bits.SSPIP = 1;
    PIE1bits.SSPIE = 1;
    if(!select){
        SSPCON1 = 0b00101000;
        SSPCON2 = 0b00001000;
        SSPADD = 29;
    }
    else{
        SSPCON1 = 0b00110110; // 7bit addres, intterupt enable
        SSPCON2 = 0b10000001;
        SSPADD = 0x10;
    }
}

char which_am_i(){
    if( // master 7nits address mode
        SSPCON1bits.SSPM0 == 0 &&
        SSPCON1bits.SSPM1 == 0 &&
        SSPCON1bits.SSPM2 == 0 &&
        SSPCON1bits.SSPM3 == 1){
        return 0;
    }
    else if( // slave 7bits address mode
        SSPCON1bits.SSPM0 == 0 &&
        SSPCON1bits.SSPM1 == 1 &&
        SSPCON1bits.SSPM2 == 1 &&
        SSPCON1bits.SSPM3 == 0){
        return 1;
    }
    else
        return 2;
}

/* MASTER */
char i2c_write(int addr, char *data, char num){
    INTCONbits.GIE = 0;
    char sum = 0;
    unsigned long int cnt = 0;
    SSPCON2bits.SEN = 1;
    while(SSPCON2bits.SEN);
    PIR1bits.SSPIF = 0;
    SSPBUF = addr;
    for(cnt = 0; PIR1bits.SSPIF == 0; cnt++)
        if(cnt > TIMEOUT)
            return 1;
    PIR1bits.SSPIF = 0;
    if(SSPCON2bits.ACKSTAT)
        return 1;
    for(sum = 0; sum < num; sum++){
        SSPBUF = data[sum];
        for(cnt = 0; PIR1bits.SSPIF == 0; cnt++)
        if(cnt > TIMEOUT)
            return 1;
        PIR1bits.SSPIF = 0;
        if(SSPCON2bits.ACKSTAT)
            return 1;
    }
    SSPCON2bits.PEN = 1;
    //SSPCON2bits.RSEN = 1; // order slave to resend
    while(SSPCON2bits.PEN);
    INTCONbits.GIE = 1;
    return 0;
}

char i2c_read(int addr, char *data, char num){
    INTCONbits.GIE = 0;
    char sum = 0;
    unsigned long cnt = 0;
    SSPCON2bits.SEN = 1;
    while(SSPCON2bits.SEN);
    PIR1bits.SSPIF = 0;
    SSPBUF = addr;
    for(cnt = 0; PIR1bits.SSPIF == 0; cnt++)
        if(cnt > TIMEOUT)
            return 1;
    PIR1bits.SSPIF = 0;
    if(SSPCON2bits.ACKSTAT)
        return 1;
    for(sum = 0; sum < num; sum++){
        SSPCON2bits.RCEN = 1;
        for(cnt = 0; SSPSTATbits.BF == 0; cnt++)
            if(cnt > TIMEOUT)
                return 1;
        if(sum == (num - 1))
            SSPCON2bits.ACKDT = 1;
        else
            SSPCON2bits.ACKDT = 0;
        SSPCON2bits.ACKEN = 1;
        while(SSPCON2bits.ACKEN);
        data[sum] = SSPBUF;
        PIR1bits.SSPIF = 0;
    }
    SSPCON2bits.RCEN = 0;
    SSPCON2bits.PEN = 1;
    //SSPCON2bits.RSEN = 1; // order slave to resend
    while(SSPCON2bits.PEN);
    INTCONbits.GIE = 1;
    return 0;
}

/* IIC SLAVE */
void set_addr(char addr){
    if(which_am_i())
        SSPADD = addr;
}

// UART
void serial_init(){
    RCSTA = 0b10010000; // UART
    TXSTA = 0b00100100;
    TRISBbits.RB5 = 1; // RX
    TRISBbits.RB7 = 0; // TX
    BAUDCON = 0b00001000; // HI-16Bit
    SPBRGH = 1249 >> 8; // 9600
    SPBRG = 1249;
    IPR1bits.RCIP = 1;
    PIE1bits.RCIE =1;
}

void serial_baud(unsigned long baudrate){
	switch(baudrate){
		case 9600:
			SPBRGH = 0x04;
            SPBRG = 0xE1;
			break;		
		case 19200:
			SPBRGH = 0x02;
            SPBRG = 0x70;
			break;	
		case 57600:
			SPBRGH = 0;
            SPBRG = 207;
			break;
		case 115200:
			SPBRGH = 0;
            SPBRG = 103;
			break;
	}	
}	

void serial_write(char *data, char num){
    char i;
    for(i = 0; i < num; i++){  // Transmit a byte
        TXREG = data[i];
        while (!PIR1bits.TXIF);
    }
}