/*******************************************************************
 * Name: Lab5_atmega48
 * Author: Jesse Ulibarri
 * Date: 11/21/16
 * Class: ECE473
 ******************************************************************/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "mega48_uart_functions.h"
#include "lm73_functions_skel.h"
#include "twi_master.h"

char temperature[2] = {'3', '5'};
uint8_t status;
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];
uint16_t lm73_temp;
char lm73_char_temp[8];


ISR(USART_RX_vect) {
    status = uart_getc();
    if(status == 0xF0)
        uart_puts(lm73_char_temp);
}

int main() {

init_twi();
uart_init();
sei();

lm73_wr_buf[0] = 0x00;
twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 2);

while(1) {

twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2);
_delay_ms(2);
//format temperature array
lm73_temp = (lm73_rd_buf[0] << 8) | (lm73_rd_buf[1]);
lm73_temp = lm73_temp >> 7;
itoa(lm73_temp, lm73_char_temp, 10);
_delay_ms(500);

}
}//main
