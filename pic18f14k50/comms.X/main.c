#include <xc.h>
#include "config.h"
#include "stdfunc.h"

/* !CAUTION! */
// i2c_write/read() : Master mode. 'addr' is slave address without RWbit. 7bit
// set_addr() : Slave mode. 'addr' is slave address without RWbit. 7bit
#define SLAVE_ADDR 0x10 >> 1
#define NODE 2 
#define SPECIFIC_ID 4
#define RSSI 2
#define DATA 8

char data_i2c = 0;
//char data_serial = 0;
char rec[SIZE_RECV_UART] = {0};
char cmp[SIZE_RECV_UART] = {0};
char cnt = 0;

void interrupt intr(){
    // EEPROM
    if(PIR2bits.EEIF){
        // writing ended
        PIR2bits.EEIF = 0;
    }
    
    // ADC
    if(PIR1bits.ADIF){
        // ADconvert ended
        PIR1bits.ADIF = 0;
    }
    
    // I2C
    if(PIR1bits.SSPIF){
        // master : received ack
        // i2c_write/read disable interrupt
        if(which_am_i() == 0){
            PIR1bits.SSPIF = 0;
        }
        // slave : received from master
        if(which_am_i()){
            if(SSPSTATbits.P){
                PIR1bits.SSPIF = 0;
                SSPCON1bits.CKP = 1;
                return;
            }
            char mem = SSPBUF;
            if(SSPSTATbits.DA){
                data_i2c = mem;
            }
            else if(SSPSTATbits.RW){
                SSPBUF = data_i2c;
            }
            PIR1bits.SSPIF = 0;
            SSPCON1bits.CKP = 1;
        }
    }
    
    // UART
    if(PIR1bits.RCIF){ // serial receive
        rec[cnt] = RCREG;
        cnt++;
        if(cnt >= SIZE_RECV_UART){
            cnt = 0;
            for(char i = 0; i < SIZE_RECV_UART; i++)
                cmp[i] = rec[i];
        }
        PIR1bits.RCIF = 0;
    }
}

void asciicon(char c[]) {
	char hex[SIZE_RECV_UART] = {0}, i;
	for(i = 0; SIZE_RECV_UART >= i; i++){
		if(c[i] >= '0' && c[i] <= '9')
            hex[i] = c[i] - '0';
        else if(c[i] >= 'A' && c[i] <= 'F')
            hex[i] = c[i] - 'A' + 10;
        else
            hex[i] = 0;
    }
}

void main(void){
    serial_init();
    serial_baud(19200);
    i2c_init(1);
    set_addr(SLAVE_ADDR);
    unsigned long i = 0;
    while(1){
        
    }
}