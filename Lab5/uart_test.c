/*********************************
 * uart tester program
 *
 * ******************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>
#include "uart_functions.h"
#include "hd44780.h"

char lcd_string[32] = "Here's my string and line two   ";

void spi_init(void) {
    DDRB = DDRB | 0x07;
    SPCR |= (1 << SPE) | (1 << MSTR);
    SPSR |= (1 << SPI2X);
}

ISR(USART0_RX_vect) {
    PORTC |= (1 << PC4);
    //memcpy(lcd_string, "I got into the interrupt!       ", 32);
    //lcd_string[30] = 'Y';
    
    lcd_string[30] = uart_getc();

    PORTC &= ~(1 << PC4);
}

int main() {

DDRC |= (1 << PC4);

uint8_t i = 0;
char uart_msg;
uart_init();
spi_init();
lcd_init();

sei();

clear_display();
cursor_home();

while(1) {

    refresh_lcd(lcd_string);
    //uart_msg = uart_getc();
    lcd_string[31] = 'Z';

    _delay_ms(10);

}
}//main
