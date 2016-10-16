// lab2_skel.c 
// R. Traylor
// 9.12.08

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//
//             ***** LED_GRAPH_BOARD *****
//  PORTB bit 0 (SS_n) goes to REGLCK on graph board
//  PORTB bit 1 (SCLK) goes to SRCLK on graph board
//  PORTB bit 2 (MOSI) goes to SDIN on graph board
//      OE_N goes to ground on AVR
//      GND goes to ground on AVR
//      VDD goes to VCC on AVR
//      SD_OUT is not connected
//
//             ***** ENCODER_BOARD *****
//
// 
//             ***** BUTTON_BOARD *****
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

volatile uint8_t display_count = 0x01;
volatile uint8_t index = 0;

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
    
uint8_t chk_buttons(uint8_t button) {
    static uint16_t state[8] = {0,0,0,0,0,0,0,0};
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
    segment_data[1] = dec_to_7seg[(sum / 10) % 10]; // This holds the tens
    // there is no segment_data[2] because that holds the colon
    segment_data[3] = dec_to_7seg[(sum / 100) % 10]; // This holds the hundreds
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
//                                   SPI send information
//
void SPI_send(uint8_t message) {
    SPDR = message; // write message to SPI data register
    while(bit_is_clear(SPSR, SPIF)) {} // wait for data to send

}//SPI_send



//******************************************************************************

//***********************************************************************************
//                                   SPI read information
//
uint8_t SPI_read() { 
    SPDR = 0x00; // send junk to initialize SPI return
    while(bit_is_clear(SPSR, SPIF)) {} // wait until data is recieved
    return SPDR; // return data from device
}//SPI_receive

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
    __asm__ __volatile__ ("nop");

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


//******************************************************************************

//***********************************************************************************
//                                   update_LEDs
//
void update_LEDs() {
    // define loop index
    int num_digits;

    // make port A an output
    DDRA = 0xFF;

    // make sure that port has changed direction 
    __asm__ __volatile__ ("nop");
    __asm__ __volatile__ ("nop");

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


//******************************************************************************

//***********************************************************************************
//                                   update_bar_graph
//
//
void update_bar_graph() {
    SPI_send(display_count); // send data to bar graph

    PORTB |= 0x01;      // move data from shift to storage reg.
    PORTB &= ~0x01;     // change 3-state back to high Z

    display_count = display_count << 1; // move light across graph

    if(display_count == 0x00)
        display_count = 0x01; // rap around

    _delay_us(200);


}//update_bar_graph



//******************************************************************************

//***********************************************************************************
//                                   Interrupt Routine
//
ISR(TIMER0_OVF_vect) {
    PORTE = 0x00;
    get_button_input();
    update_LEDs();
    index++;
    //update_bar_graph();
    PORTE = 0x01;
}//ISR







//***********************************************************************************
//***********************************************************************************
//                                   MAIN

int main()
{

// set port bits 4-7 B as outputs
// set port bits 0-3 B as outputs (output mode for SS, MOSI, SCLK)
DDRB = 0xF7;
DDRE = 0x01; // set as output for debugging

// set up timer and interrupt
TCCR0 |= (1 << CS01) | (1 << CS02); // set timer mode (normal, 128 prescalar)
TIMSK |= (1 << TOIE0); // turn on timer interrupts

// set up SPI (master mode, clk low on idle, leading edge sample)
SPCR = (1 << SPE) | (1 << MSTR) | (0 << CPOL) | (0 << CPHA);
SPSR = (1 << SPI2X);
sei(); // enable global interrupts

while(1){ 
    if(index == 255) {
        update_bar_graph();
        index = 0;
    }
}//while

return 0;
}//main

