// lab1_code.c
// R. Traylor
// 7.21.08

//This program increments a binary display of the number of button pushes on switch
//S0 on the mega128 board.

#include <avr/io.h>
#include <util/delay.h>

//Variables
uint8_t count = 0;

//*******************************************************************************
//                            debounce_switch
// Adapted from Ganssel's "Guide to Debouncing
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed.
// Function returns a 1 only once per debounced button push so a debounce and toggle
// function can be implemented at the same time.  Expects active low pushbutton on
// Port D bit zero.  Debounce time is determined by external loop delay times 12.
//*******************************************************************************
int8_t debounce_switch() {
  static uint16_t state = 0; //holds present state
  state = (state << 1) | (! bit_is_clear(PIND, 0)) | 0xE000;
  if (state == 0xF000) return 1;
  return 0;
}


//*******************************************************************************
// 				dec_to_cbd
// Function to convert our decimal count to "binary coded decimal". Funtion takes
// an 8 bit number and divides by 10 to isolate the tens place digit. It then
// shifts the number by four places so that it is displayed in the upper 4 bits
// of the BCD. It then adds the number mod 10 to the end. This will capture the
// ones place in the lower 4 bits.
//*******************************************************************************
uint8_t dec_to_bcd(uint8_t num) {
  return((num/10 << 4) + (num % 10));
}

//*******************************************************************************
// Check switch S0.  When found low for 12 passes of "debounc_switch(), increment
// PORTB.  This will make an incrementing count on the port B LEDS.
//*******************************************************************************
int main()
{
DDRB = 0xFF;  //set port B to all outputs
while(1){     //do forever
 if(debounce_switch()) {
	count++;
    //bound the count
    if(count > 99)
        count = 0;
    //send count information to LEDs on PORTB
	PORTB = dec_to_bcd(count);

}  //if switch true for 12 passes, increment port B
  _delay_ms(2);                    //keep in loop to debounce 24ms
  } //while
} //main
