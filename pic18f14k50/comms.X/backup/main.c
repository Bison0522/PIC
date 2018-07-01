#include <xc.h>
#include "config.h"
#include "stdfunc.h"

/* !CAUTION! */
// i2c_write/read() : addr is slave address contains RWbit. 8bit
// set_addr() : addr is slave address contains RWbit(0). 8bit
char data_i2c = 0;
char data_serial = 0;

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
        data_serial = RCREG;
        PIR1bits.RCIF = 0;
    }
}

void main(void){
    init();
    intr_init();
    serial_init();
    i2c_init(1);
    //i2c_init(0);
    set_addr(0x10);
    while(1){
        data_i2c = 'E';
        LATCbits.LC5 = !LATCbits.LC5;
        LATCbits.LC4 = !LATCbits.LC4;
        LATCbits.LC3 = !LATCbits.LC3;
        LATCbits.LC2 = !LATCbits.LC2;
//        i2c_read(0x11, &data_i2c, 1);
//        serial_write(&data_i2c, 1);
//        __delay_ms(200);
    }
}