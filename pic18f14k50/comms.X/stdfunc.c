/*
 * File:   stdfunc.c
 * Author: Kinari
 *
 * Created on 2018/05/11, 14:20
 */

#include <pic18f14k50.h>

#include "stdfunc.h"

char freq = 0;

void intr_init(){
    INTCONbits.GIE = 0;
    RCONbits.IPEN = 0;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
}

void init(){
    ANSEL = 0;
    ANSELH = 0;
    TRISC = 0;
    LATC = 0;
    TRISB = 0;
    OSCCON2bits.PRI_SD = 1;
    OSCCON = 0;
    intr_init();
}

void init_for_MD(char active){ // PIN init state
    ANSEL = 0;
    ANSELH = 0;
    TRISC = 0;
    if(active){
        LATCbits.LC5 = 0;
        LATCbits.LC4 = 0;
        LATCbits.LC3 = 0;
        LATCbits.LC2 = 0;
    }
    else{
        LATCbits.LC5 = 1;
        LATCbits.LC4 = 1;
        LATCbits.LC3 = 1;
        LATCbits.LC2 = 1;
    }
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
        SSPCON1 = 0b00110110; // 7bit addres, interrupt enable
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
    SSPBUF = addr << 1 + 0;
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
    SSPBUF = addr << 1 + 1;
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
        SSPADD = addr << 1;
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

// FULL BRIDGE PWM
void full_bridge_pwm_init(char frequency){
    TRISCbits.RC5 = 0; // P1A is output
    TRISCbits.RC4 = 0; // P1B is output
    TRISCbits.RC3 = 0; // P1C is output
    TRISCbits.RC2 = 0; // P1D is output
    CCP1CON = 0b01001111; // active low
    T2CON = 0b00000111;
    switch(frequency){
        // PR2 = 48,000,000 / (4 * frequency[Hz] * 16) - 1
        case 0: // 10 kHz
            PR2 = 74;
            frequency = 0;
            break;
        case 1: // 50 kHz
            PR2 = 14;
            frequency = 1;
            break;
        case 2: // 75 kHz
            PR2 = 10;
            frequency = 2;
            break;
        default: // 10kHz
            PR2 = 74;
            frequency = 0;
            break;
    }
    CCPR1L = 0;
    CCPR1H = 0;
    TMR2 = 0;
    TMR2ON = 1;
    freq = frequency;
}

void restart_pwm_full(){
    full_bridge_pwm_init(freq);
}

void fin_pwm(){
    T2CONbits.TMR2ON = 0;
    CCP1CONbits.CCP1M = 0;
}

void brake(){
    fin_pwm();
    LATCbits.LC5 = 0; // P1A out 0
    LATCbits.LC4 = 1; // P1B out 1
    LATCbits.LC3 = 0; // P1C out 0
    LATCbits.LC2 = 1; // P1D out 1
}

void  change_rotate(char s){ // 1 : cw, 0 : ccw
    if(s){
        CCP1CONbits.P1M0 = 1;
        CCP1CONbits.P1M1 = 0;
    }
    else{
        CCP1CONbits.P1M0 = 1;
        CCP1CONbits.P1M1 = 1;
    }
}

void set_duty_full(char duty, char turn){ // duty is from 0 to 100[%]
    if(T2CONbits.TMR2ON == 0)
        restart_pwm_full();
    change_rotate(turn);
//     DUTY   : (CCPR1L<7:0>:CCP1CON<5:4>) = 48,000,000 / 16 * PulseWidth[s]
//     PERIOD : 1[us] = 1E-6[s]
//     DUTY   : (CCPR1L<7:0>:CCP1CON<5:4>) = 48 / 16 * PulseWidth[us]
    unsigned pw = 48 / 16 * duty; // 10 kHz
    if(duty < 0)
        duty = 0;
    switch(freq){
        case 0: // 10 kHz
            if(duty > LIMIT_10K)
                duty = LIMIT_10K;
            pw = 48 / 16 * duty; // 10 kHz
            break;
        case 1: // 50 kHz
            if(duty > LIMIT_50K)
                duty = LIMIT_50K;
            pw = 48 / 16 * ((20 * duty) * 0.01); // 50 kHz
            break;
        case 2: // 75 kHz
            if(duty > LIMIT_75K)
                duty = LIMIT_75K;
            pw = 48 / 16 * ((13 * duty) * 0.01); // 75 kHz
            break;
    }
    CCPR1L = (pw >> 2) & 0xFF;
    CCP1CONbits.DC1B = pw & 0x03;
}

// HALF BRIDGE PWM
void set_deadhand(){
    PWM1CONbits.PDC = 10;
}

void half_bridge_pwm_init(char frequency){
    TRISCbits.RC5 = 0; // P1A is output
    TRISCbits.RC4 = 0; // P1B is output
    TRISCbits.RC3 = 0; // P1C is output
    TRISCbits.RC2 = 0; // P1D is output
    CCP1CON = 0b10001111; // active low
    T2CON = 0b00000111;
    set_deadhand();
    switch(frequency){
        // PR2 = 48,000,000 / (4 * frequency[Hz] * 16) - 1
        case 0: // 10 kHz
            PR2 = 74;
            frequency = 0;
            break;
        case 1: // 50 kHz
            PR2 = 14;
            frequency = 1;
            break;
        case 2: // 75 kHz
            PR2 = 10;
            frequency = 2;
            break;
        default: // 10kHz
            PR2 = 74;
            frequency = 0;
            break;
    }
    CCPR1L = 0;
    CCPR1H = 0;
    TMR2 = 0;
    TMR2ON = 1;
    freq = frequency;
}

void restart_pwm_half(){
    half_bridge_pwm_init(freq);
}

void set_duty_half(char duty){ // duty is from 0 to 100[%]
    if(T2CONbits.TMR2ON == 0)
        restart_pwm_half();
//     DUTY   : (CCPR1L<7:0>:CCP1CON<5:4>) = 48,000,000 / 16 * PulseWidth[s]
//     PERIOD : 1[us] = 1E-6[s]
//     DUTY   : (CCPR1L<7:0>:CCP1CON<5:4>) = 48 / 16 * PulseWidth[us]
    unsigned pw = 48 / 16 * duty; // 10 kHz
    if(duty < 0)
        duty = 0;
    switch(freq){
        case 0: // 10 kHz
            if(duty > LIMIT_10K)
                duty = LIMIT_10K;
            pw = 48 / 16 * duty; // 10 kHz
            break;
        case 1: // 50 kHz
            if(duty > LIMIT_50K)
                duty = LIMIT_50K;
            pw = 48 / 16 * ((20 * duty) * 0.01); // 50 kHz
            break;
        case 2: // 75 kHz
            if(duty > LIMIT_75K)
                duty = LIMIT_75K;
            pw = 48 / 16 * ((13 * duty) * 0.01); // 75 kHz
            break;
    }
    CCPR1L = (pw >> 2) & 0xFF;
    CCP1CONbits.DC1B = pw & 0x03;
}