#ifndef STDCOM_H
#define STDCOM_H

#include <xc.h>

#define _XTAL_FREQ 48000000 // Tosc 83.333... ns
#define EESIZE 256
#define TIMEOUT 2000
#define DEADHAND 83 //ns
#define LIMIT_10K 150
#define LIMIT_50K 150
#define LIMIT_75K 150 // Tperiod 13 us = 13000 ns

void intr_init();
void init();
void init_for_MD(char active);

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

void full_bridge_pwm_init(char frequency);
void restart_pwm_full();
void fin_pwm();
void brake();
void change_rotate(char s);
void set_duty_full(char duty, char rotate);

void set_deadhand();
void half_bridge_pwm_init(char frequency);
void restart_pwm_half();
void set_duty_half(char duty);

#endif