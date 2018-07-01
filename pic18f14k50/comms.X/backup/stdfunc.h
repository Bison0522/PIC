#ifndef STDCOM_H
#define STDCOM_H

#include <xc.h>

#define _XTAL_FREQ 48000000
#define EESIZE 256
#define TIMEOUT 2000

void intr_init();
void init();

void EE_init();
unsigned char EE_read(int adrs);
void EE_write(int adrs, unsigned char data);
void EE_clear();

void ADC_init();
char ADC_read();

void i2c_init(char select);
/* !CAUTION! */
// addr is slave address contains RWbit. 8bit
char i2c_write(int addr, char *data, char num);
char i2c_read(int addr, char *data, char num);
void set_addr(char addr);

void serial_init();
char which_am_i();
void serial_baud(unsigned long baud);
void serial_write(char *data, char num);

#endif