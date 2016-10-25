//Author: Jesse Ulibarri
//Date: 10/17/16
//Class: ECE 473
//Assignment: Lab3

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
//  PORTB bit 1 (SCLK) goes to SCK on encoder board
//  PORTB bit 3 (MISO) goes to SER_OUT on encoder board
//  PORTE bit 0 goes to SH/LD on encoder board
//  PORTE bit 1 goes to CLK_INH on encoder board
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

#define MAX_NUM 1023

// Define different modes
#define ADD_ONE 0xFF
#define ADD_TWO 0xFE
#define ADD_FOUR 0xFD
#define NO_ADD 0xFC

volatile int16_t summed_value = 0;
volatile uint8_t current_mode = ADD_ONE;

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[10] = {ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE};

//array that holds the segment codes
uint8_t segment_codes[5] = {SEL_DIGIT_4, SEL_DIGIT_3, SEL_COLON, SEL_DIGIT_2, SEL_DIGIT_1};

//look up table to determine what direction the encoders are turning
int8_t enc_lookup[16] = {0,0,0,0,0,0,0,1,0,0,0,-1,0,0,0,0};


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
    static uint16_t state[3] = {0};
    state[button] = (state[button] << 1) | (!bit_is_clear(PINA, button)) | 0xE000;
    if (state[button] == 0xF000) return 1;
    return 0;

}i//chk_buttons

                                                 
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
    switch(num_of_digits)
    {
        case 1:
            segment_data[1] = OFF;
            segment_data[2] = OFF;
            segment_data[3] = OFF;
            segment_data[4] = OFF;
            break;
        case 2:
            segment_data[2] = OFF;
            segment_data[3] = OFF;
            segment_data[4] = OFF;
            break;
        case 3:
            segment_data[2] = OFF;
            segment_data[4] = OFF;
            break;
        case 4:
            segment_data[2] = OFF;
            break;
    }//switch
}//segment_sum
//***********************************************************************************

//***********************************************************************************
//                                   Set Boundaries
//
// This function bounds the current count to within the max limit. It then calls the 
// segsum function which will format our value into the segment data array.

void bound_format_count() {

    //bound count
    if(summed_value > MAX_NUM)
        summed_value -= MAX_NUM;
    if(summed_value < 0)
        summed_value += MAX_NUM + 1;

    segsum(summed_value);

}//bound_format_count


//******************************************************************************

//***********************************************************************************
//                                   SPI send information
// Function will take in a message to send through SPI. It will write the data to the
// SPI data register and then wait for the message to send before returning.
//
// NOT IN USE

void SPI_send(uint8_t message) {
    SPDR = message; // write message to SPI data register
    while(bit_is_clear(SPSR, SPIF)) {} // wait for data to send

}//SPI_send



//******************************************************************************

//***********************************************************************************
//                                   SPI read information
// Function will read any data coming through the SPI communication bus. It will write 
// a garbage value to the SPI data register to initialize communication and then wait
// for the data to be sent. At this time, any incoming data has entered the SPI data
// register. The function now returns the read data.
//
// NOT IN USE

uint8_t SPI_read() { 
    PORTE = 0x00; //shift data into encoder register
    __asm__ __volatile__ ("nop");
    __asm__ __volatile__ ("nop");
    PORTE = 0x01; //end shift

    SPDR = 0x00; // send junk to initialize SPI return
    while(bit_is_clear(SPSR, SPIF)) {} // wait until data is recieved
    return SPDR; // return data from device

}//SPI_receive

//******************************************************************************

//***********************************************************************************
//                                   get_button_input
//
// Function will get any input from the button board and load the information
// into the segment_data array.


void get_button_input() {
    // define index integer 
    int i;

    // make port A input with pull-ups
    DDRA = 0x00;
    PORTA = 0xFF;

    // enable the button tristate buffer
    PORTB = ENABLE_TRISTATE;

    // wait for ports to be set
    __asm__ __volatile__ ("nop");

    // loop throught the buttons and check for a push
    for(i = 0; i < 3; i++) {
        if(chk_buttons(i))
            current_mode ^= (1 << i);
    }

    // disable the tristate buffer
    PORTB = DISABLE_TRISTATE;

}//get_button_input


//******************************************************************************

//***********************************************************************************
//                                   update_LEDs
//
// Function will send the data in the segment data array to the 7-segment board and 
// then wait 0.5 ms on each value to allow the LED to be on long enough to produce 
// a bright output.
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
        // send PORTB the digit to desplay
        PORTB = segment_codes[num_digits];

        //__asm__ __volatile__ ("nop");

        // send 7 segment code to LED segments
        PORTA = segment_data[num_digits];

        // wait a moment
        _delay_ms(0.5);
    }
    PORTA = OFF; // turn off port to keep each segment on the same amount of time
    __asm__ __volatile__ ("nop");
    __asm__ __volatile__ ("nop");

}//update_LEDs


//******************************************************************************

//***********************************************************************************
//                                   update_bar_graph
// Function will send the inverted value of the current mode variable to the graph bar.
//
// NOT IN USE

void update_bar_graph() {
    SPI_send(~current_mode); // send data to bar graph

    PORTB |= 0x01;      // move data from shift to storage reg.
    PORTB &= ~0x01;     // change 3-state back to high Z

    _delay_us(200);


}//update_bar_graph


//******************************************************************************

//***********************************************************************************
//                                   Encoder 1
// 
// Function will receive the raw data brought in from the encoder board, interperate
// the data, and add the correct value to the sum variable based on the recieved 
// encoder status and current mode.
//

void encoder1_instruction(uint8_t encoder1_val) {

    static uint8_t encoder1_hist = 0;
    int8_t add;

    encoder1_hist = encoder1_hist << 2; // shift the encoder history two places
    encoder1_hist = encoder1_hist | (encoder1_val & 0b0011); // or the history with new value
    switch(current_mode) 
    {
        case ADD_ONE:
            add = enc_lookup[encoder1_hist & 0b1111] << 0; //add one
            summed_value += add; // add number to sum
            break;
        case ADD_TWO:
            add = enc_lookup[encoder1_hist & 0b1111] << 1; //add two
            summed_value += add; // add number to sum
            break;
        case ADD_FOUR:
            add = enc_lookup[encoder1_hist & 0b1111] << 2; //add four
            summed_value += add; // add number to sum
            break;
        case NO_ADD:
             //do not add anything
            break;
        default:
            break;

    }//switch

}//get_encoder1


//******************************************************************************

//***********************************************************************************
//                                   Encoder 2
// 
// This function is the same as the encoder1 function except that it will interperate
// the data coming from encoder 2.
//

void encoder2_instruction(uint8_t encoder2_val) {

    static uint8_t encoder2_hist = 0;
    int8_t add;

    encoder2_hist = encoder2_hist << 2; // shift the encoder history two places
    encoder2_hist = encoder2_hist | (encoder2_val & 0b0011); // or the history with new value
    switch(current_mode) 
    {
        case ADD_ONE:
            add = enc_lookup[encoder2_hist & 0b1111] << 0; //add one
            summed_value += add; // add number to sum
            break;
        case ADD_TWO:
            add = enc_lookup[encoder2_hist & 0b1111] << 1; //add two
            summed_value += add; // add number to sum
            break;
        case ADD_FOUR:
            add = enc_lookup[encoder2_hist & 0b1111] << 2; //add four
            summed_value += add; // add number to sum
            break;
        case NO_ADD:
             //do not add anything
            break;
        default:
            break;

    }//switch

}//encoder2


//******************************************************************************

//***********************************************************************************
//                                   SPI Total Functionallity
//
// Function will send the current mode data to the graph board and receive data from
// the encoder at the same time. It will then call the encoders 1 and 2 functions 
// to interperate the encoder data.
//

void SPI_function() {
    uint8_t data;
    
    //************ Encoder Portion *******************
    PORTE = 0x00; //shift encoder data into register
    __asm__ __volatile__ ("nop");
    __asm__ __volatile__ ("nop");
    PORTE = 0x01; //end shift

    //*********** Send and Receive SPI Data **********
    SPDR = (~current_mode | 0x03); // send the bar graph the current status
    while(bit_is_clear(SPSR, SPIF)) {} // wait until encoder data is recieved
    data = SPDR;

    //********** Bar Graph Portion *******************
    PORTB |= 0x01;      // move graph data from shift to storage reg.
    PORTB &= ~0x01;     // change 3-state back to high Z

    //********** Pass Encoder Info to Functions ******
    encoder1_instruction(data);
    encoder2_instruction(data >> 2);

}//SPI_function


//******************************************************************************

//***********************************************************************************
//                                   Interrupt Routine
//
ISR(TIMER0_OVF_vect) {
    
    PORTF = 0x00;
    uint8_t old_DDRA = DDRA;
    uint8_t old_PORTA = PORTA;
    uint8_t old_PORTB = PORTB;

    get_button_input();
    SPI_function();

    DDRA = old_DDRA;
    PORTA = old_PORTA;
    PORTB = old_PORTB;
    PORTF = 0x01;

}//ISR


//***********************************************************************************
//***********************************************************************************
//                                   MAIN

int main()
{

// set port bits 4-7 B as outputs
// set port bits 0-3 B as outputs (output mode for SS, MOSI, SCLK)
DDRB = 0xF7;



// encoder is on PORTE
DDRE = 0x03;
PORTE = 0xFD;
DDRF = 0x01; // set as output for debugging

// set up timer and interrupt
TCCR0 |= (1 << CS00) | (1 << CS02); // set timer mode (normal, 128 prescalar)
TIMSK |= (1 << TOIE0); // turn on timer interrupts

// set up SPI (master mode, clk low on idle, leading edge sample)
SPCR = (1 << SPE) | (1 << MSTR) | (0 << CPOL) | (0 << CPHA);
SPSR = (1 << SPI2X);
sei(); // enable global interrupts

while(1){

    bound_format_count();
    update_LEDs();
    
}//while

return 0;
}//main

