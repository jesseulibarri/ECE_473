/ lab2_skel.c 
// R. Traylor
// 9.12.08

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define OFF     0xFF
#define ZERO    0xC0
#define ONE     0xF9
#define TWO     0xA4
#define THREE   0xB0
#define FOUR    0x99
#define FIVE    0x92
#define SIX     0x82
#define SEVEN   0xF8
#define EIGHT   0x80
#define NINE    0x90

#define SEL_DIGIT_1 0x40
#define SEL_DIGIT_2 0x30
#define SEL_DIGIT_3 0x10
#define SEL_DIGIT_4 0x00
#define SEL_COLON   0x20
#define ENABLE_TRISTATE 0x70    // tristate is enabled by Y7 decoder output.
                                // ENABLE_TRISTATE are the bits on PORTB that
                                // need to be set to get a low output on Y7.
#define DISABLE_TRISTATE 0x60

#define MAX_NUM 1023;

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[10] = {ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE};

//array that holds the segment codes
uint8_t segment_codes[5] = {SEL_DIGIT_4, SEL_DIGIT_3, SEL_COLON, SEL_DIGIT_2, SEL_DIGIT_1};

//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//
    
    uint16_t state[8] = {0,0,0,0,0,0,0,0};
uint8_t chk_buttons(uint8_t button) {
   // int8_t chk_buttons(int8_t button) {
   // static uint16_t state = 0; //holds present state
    state[button] = (state[button] << 1) | (!bit_is_clear(PINA, button)) | 0xE000;
    if (state[button] == 0xF000) return 1;
    return 0;
    }

                                                 
//******************************************************************************

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {

  // variables needed for this function
    int num_of_digits;
    //int i;
  //determine how many digits there are
    if(sum < 10) num_of_digits = 1;
    else if (sum < 100) num_of_digits = 2;
    else if (sum < 1000) num_of_digits = 3;
    else num_of_digits = 4;

  //break up decimal sum into 4 digit-segments
    //***** General equation is num % 10 -> num /= 10 and repeat *****
    segment_data[0] = dec_to_7seg[sum % 10]; // This holds the ones
    //sum /= 10;
    segment_data[1] = dec_to_7seg[(sum / 10) % 10]; // This holds the tens
    //sum /= 10;
    // there is no segment_data[2] because that holds the colon
    segment_data[3] = dec_to_7seg[(sum / 100) % 10]; // This holds the hundreds
    //sum /= 10;
    segment_data[4] = dec_to_7seg[(sum / 1000) % 10]; // This holds the thousands

  //blank out leading zero digits
        if(num_of_digits == 1)
        {
            segment_data[1] = OFF;
            segment_data[2] = OFF;
            segment_data[3] = OFF;
            segment_data[4] = OFF;

        }
        else if(num_of_digits == 2)
        {
            segment_data[2] = OFF;
            segment_data[3] = OFF;
            segment_data[4] = OFF;
        }
        else if(num_of_digits == 3)
        {
            segment_data[2] = OFF;
            segment_data[4] = OFF;
        }
        else
            segment_data[2] = OFF;
  //now move data to right place for misplaced colon position
}//segment_sum
//***********************************************************************************


//***********************************************************************************


//******************************************************************************

//***********************************************************************************
//                                   get_button_input
//
// This routine will get any input from the button board and load the information
// into the segment_data array.


void get_button_input() {
    // define index integer 
    int i;
    static uint16_t summed_value = 0;

    // make port A input with pull-ups
    DDRA = 0x00;
    PORTA = 0xFF;

    // enable the button tristate buffer
    PORTB = ENABLE_TRISTATE;

    // wait for ports to be set
    _delay_us(500);

    // loop throught the buttons and check for a push
    for(i = 0; i < 8; i++) {
        if(chk_buttons(i))
            summed_value += (1 << i);
    }

    // disable the tristate buffer
    PORTB = DISABLE_TRISTATE;

    // bound the count
    if(summed_value > 1023)
        summed_value = summed_value % MAX_NUM;

    // put all the numbers into the array in the correct
    // place to be displayed
    segsum(summed_value);

}


void update_LEDs() {
    // define loop index
    int num_digits;

    // make port A an output
    DDRA = 0xFF;

    // make sure that port has changed direction
    _delay_us(500);

    // loop and update each LED number
    for(num_digits = 0; num_digits < 5; num_digits++) {
        // send 7 segment code to LED segments
        PORTA = segment_data[num_digits];
        // send PORTB the digit to desplay
        PORTB = segment_codes[num_digits];
        // wait a moment
        _delay_ms(0.5);
    }
}

ISR(TIMER0_OVF_vect) {
    PORTE = 0x01;
    get_button_input();
    update_LEDs();
    PORTE = 0x00;
}

int main()
{

//set port bits 4-7 B as outputs
DDRB = 0xF0;
DDRE = 0x01;
// set up timer and interrupti
TCCR0 |= (1 << CS01) | (1 << CS02); // set timer mode (normal, 128 prescalar)
TIMSK |= (1 << TOIE0); // turn on timer interrupts

sei(); // enable global interrupts

while(1){ }//while

return 0;
}//main
