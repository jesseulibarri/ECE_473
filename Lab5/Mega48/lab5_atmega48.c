/*******************************************************************
 * Name: Lab5_atmega48
 * Author: Jesse Ulibarri
 * Date: 11/21/16
 * Class: ECE473
 ******************************************************************/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "mega48_uart_functions.h"
#include "twi_master.h"

char temperature[2] = {'3', '5'};
uint8_t status;

ISR(USART_RX_vect) {
    status = uart_getc();
    if(status == 0xF0)
        uart_puts(temperature);
}

int main() {

uart_init();
sei();

while(1) {


}
}//main
