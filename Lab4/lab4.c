

/**********************************************************************
 * Copywrite: NONE
 * Original Author(s): Jesse Ulibarri
 * Original Date: 10/17/16
 * Version: Lab4.1
 * Description: ATMega128 will track real time to be displayed on the
 *  seven-segment, five-digit board. Eight-button board will receive
 *  user input and change the system's state. Based on the state,
 *  users will be able to change the current time and alarm time by
 *  using the encoder board. Current system state will be displayed 
 *  on the  LED graph board.
 *********************************************************************/

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
#include "hd44780.c"

#define FALSE   0
#define TRUE    1

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
#define COLON_ON   0xFC
#define COLON_OFF  0xFF

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
#define NORMAL              0xFF
#define TOGGLE_CLK_FORMAT   0x7F
#define SET_CLK             0xBF
#define SET_ALARM           0xDF

#define VOL_COMPARE vol_duty_cycle / 100 * 0x8000

//volatile int16_t summed_value = 0;
uint8_t current_mode = NORMAL;
int8_t hrs = 10;
int8_t min = 5;
uint8_t sec = 0;
int8_t alarm_hrs = 1;
int8_t alarm_min = 0;
uint8_t alarm_AM = FALSE;
uint8_t Colon_Status = FALSE;
uint8_t AM = FALSE;
uint8_t twelve_hr_format = TRUE;
uint8_t alarm_on = FALSE;

//variable defines what duty cycle to run the volume at. Do not go above 60
uint16_t vol_duty_cycle = 10;

uint8_t alarm_msg[16] = {'A', 'L', 'A', 'R', 'M', ' ',  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[10] = {ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE};

//array that holds the segment codes
uint8_t segment_codes[5] = {SEL_DIGIT_4, SEL_DIGIT_3, SEL_COLON, SEL_DIGIT_2, SEL_DIGIT_1};

//look up table to determine what direction the encoders are turning
int8_t enc_lookup[16] = {0,0,0,0,0,0,0,1,0,0,0,-1,0,0,0,0};

//******************************************************************************
//                            Clock_Init
//
//******************************************************************************
// External clock runs at 32kHz.
// Elapsed time = 32,768Hz / (256 * 128) = 1 sec

void real_clk_init() {
    
    // Follow procedures in the datasheet to select the external clock.
    TIMSK &= ~((1 << OCIE0) | (1 << TOIE0)); //clear interrupts
    ASSR |= (1 << AS0);                     //enable external clock
    TCCR0 = (0 << WGM01) | (0 << WGM00) | \
            (1 << CS02) | (1 << CS00);      //normal mode, 128 prescale
    while(!((ASSR & 0b0111) == 0)) {}       //spin till registers finish updating
    TIFR |= (1 << OCF0) | (1 << TOV0);      //clear interrupt flags
    TIMSK |= (1 << TOIE0);                  //enable overflow interrupt

}//real_clk_intit


/******************************************************************************
* Function: chk_buttons
*******************************************************************************
* Parameters: uint8_t "button"
* Return: True if button pushed
* Description: Checks the state of the button number passed to it. It shifts in ones till   
*   the button is pushed. Function returns a 1 only once per debounced button    
*   push so a debounce and toggle function can be implemented at the same time.  
*   Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
*   Expects active low pushbuttons on PINA port.  Debounce time is determined by 
*   external loop delay times 12. 
*******************************************************************************/
    
uint8_t chk_buttons(uint8_t button) {
    static uint16_t state[8] = {0};
    state[button] = (state[button] << 1) | (!bit_is_clear(PINA, button)) | 0xE000;
    if (state[button] == 0xF000) return 1;
    return 0;

}//chk_buttons

                                                 

//***********************************************************************************
//                             format_clk_array
//
//******************************************************************************
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|

void format_clk_array(uint8_t hours, uint8_t minutes) {


  //break up decimal sum into 4 digit-segments
    segment_data[0] = dec_to_7seg[minutes % 10]; // This holds the ones
    segment_data[1] = dec_to_7seg[(minutes / 10) % 10]; // This holds the tens
    // there is no segment_data[2] because that holds the colon
    segment_data[3] = dec_to_7seg[hours % 10]; // This holds the hundreds
    segment_data[4] = dec_to_7seg[(hours / 10) % 10]; // This holds the thousands

    // Determine if the colon needs to be on
    if(TCNT0 == 128) 
        Colon_Status = TRUE;

    if(current_mode == SET_ALARM && !alarm_AM && twelve_hr_format)
        segment_data[0] &= ~(1 << 7);

    // Determine if it is AM or PM
    else if(!AM && twelve_hr_format)
        segment_data[0] &= ~(1 << 7); //turn on last DP

    if(alarm_on)
        segment_data[4] &= ~(1 << 7);


    switch(Colon_Status)
    {
        case TRUE:
            segment_data[2] = COLON_ON;
            break;
        case FALSE:
            segment_data[2] = COLON_OFF;
            break;
    }//switch
}//segment_sum


//***********************************************************************************
//                                   Set Boundaries
//
//******************************************************************************
// This function bounds the current count to within the max limit. It then calls the 
// segsum function which will format our value into the segment data array.

void clk_boundary() {

    switch(twelve_hr_format) 
    {
        case TRUE:
            if(sec == 60) {
                min += 1;
                sec = 0;
            }
            if(min == 60) {
                hrs += 1;
                min = 0;
            }
            if(hrs == 13) {
                hrs = 1;
                AM ^= TRUE; //Toggle every time hrs rap around
            }
            break;

        case FALSE:
            if(sec == 60) {
                min += 1;
                sec = 0;
            }
            if(min == 60) {
                hrs += 1;
                min = 0;
            }
            if(hrs == 24) {
                hrs = 0;
            }
            break;
        }//switch
}//clk_boundary

//***********************************************************************************
//                                   Step Time Forward
//
//******************************************************************************

void step_time() {

    Colon_Status = FALSE;   // part of colon "one-shot". Turn colon OFF every interrupt
    sec += 1;               // increment the second count

    clk_boundary();

}//step_time


//***********************************************************************************
//                                   Change Format
//
//******************************************************************************

void twelve_to_twfour() {
    if(!AM)
        hrs += 12;
}//twelve_to_twfour

void twfour_to_twelve() {
    if(hrs > 12) {
        hrs -= 12;
        AM = FALSE;
    }
}//twfour_to_twelve


//***********************************************************************************
//                                   SPI send information
//
//******************************************************************************
// Function will take in a message to send through SPI. It will write the data to the
// SPI data register and then wait for the message to send before returning.
//
// NOT IN USE

void SPI_send(uint8_t message) {
    SPDR = message; // write message to SPI data register
    while(bit_is_clear(SPSR, SPIF)) {} // wait for data to send

    PORTB |= 0x01;      // move data from shift to storage reg.
    PORTB &= ~0x01;     // change 3-state back to high Z

}//SPI_send



//***********************************************************************************
//                                   SPI read information
//
//******************************************************************************
// Function will read any data coming through the SPI communication bus. It will write 
// a garbage value to the SPI data register to initialize communication and then wait
// for the data to be sent. At this time, any incoming data has entered the SPI data
// register. The function now returns the read data.
//
// NOT IN USE

uint8_t SPI_read() {

    // Here is an example of internal commenting used to describe several lines 
    // of code.
    if(1) {
        PORTE = 0x00; //shift data from encoder into it's internal register
        __asm__ __volatile__ ("nop");
        __asm__ __volatile__ ("nop");
        PORTE = 0x01; //end shift

        SPDR = 0x00; // send junk to initialize SPI return
        while(bit_is_clear(SPSR, SPIF)) {} // wait until data is recieved
    }//if
    return SPDR;

}//SPI_read


//***********************************************************************************
//                                   get_button_input
//
//******************************************************************************
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
    __asm__ __volatile__ ("nop");

    // loop throught the buttons and check for a push
    switch(current_mode)
    {
        case NORMAL:
            for(i = 7; i > 4; i--) {
                if(chk_buttons(i))
                    current_mode &= ~(1 << i);
            }
            break;

        case SET_CLK:

            switch(twelve_hr_format)
            {
                case TRUE:
                    if(chk_buttons(7))
                        AM ^= TRUE;
                    if(chk_buttons(6))
                        current_mode = NORMAL;
                    break;
                case FALSE:
                    if(chk_buttons(6))
                        current_mode = NORMAL;
                        //TCCR0 |= (1 << CS02) | (1 << CS00); //turn clock back on
                    break;
            }

            break;


        case SET_ALARM:

            /*switch(twelve_hr_format)
            {
                case TRUE:
                    if( chk_buttons(0))
                        AM ^= TRUE;
                    if(chk_buttons(2)) 
                        current_mode = NORMAL;
                    if(ckh_buttons(7)
                        alarm_on ^= TRUE;
                    break;
                case FALSE:
                    if(chk_buttons(2))
                        current_mode = NORMAL;
                    break;
            }

            break; */
            if(twelve_hr_format && chk_buttons(0))
    //       {
    //            case TRUE:
    //                if( chk_buttons(0))
    //                    AM ^= TRUE;
    //                if(chk_buttons(2)) 
    //                    current_mode = NORMAL;
    //                if(ckh_buttons(7)
    //                    alarm_on ^= TRUE;
    //                break;
    //            case FALSE:
    //                if(chk_buttons(2))
    //                    current_mode = NORMAL;
    //                break;
    //        }

    //        break;
            if(twelve_hr_format && chk_buttons(6))
                alarm_AM ^= TRUE;
            if(chk_buttons(5))
                current_mode = NORMAL;
            if(chk_buttons(0))
                alarm_on ^= TRUE;
            break;
            
    }//switch

    // disable the tristate buffer
    PORTB = DISABLE_TRISTATE;

}//get_button_input



//***********************************************************************************
//                                   update_LEDs
//
//******************************************************************************
// Function will send the data in the segment data array to the 7-segment board and 
// then wait 0.5 ms on each value to allow the LED to be on long enough to produce 
// a bright output.
//

void update_LEDs() {
    // define loop index
    int num_digits;

    // make port A an output
    DDRA = 0xFF;
    //TCCR2 |= (1 << COM20) | (1 << COM21); 
    // make sure that port has changed direction 
    __asm__ __volatile__ ("nop");
    __asm__ __volatile__ ("nop");

    // loop and update each LED number
    for(num_digits = 0; num_digits < 5; num_digits++) {
        // send PORTB the digit to desplay
        PORTB = segment_codes[num_digits];

        // send 7 segment code to LED segments
        PORTA = segment_data[num_digits];

        // wait a moment
        _delay_ms(0.5);
    }
    PORTA = OFF; // turn off port to keep each segment on the same amount of time
    //TCCR2 &= ~((1 << COM20) | (1 << COM21));
    __asm__ __volatile__ ("nop");
    __asm__ __volatile__ ("nop");

}//update_LEDs



//***********************************************************************************
//                                   Encoder 1
// 
//******************************************************************************
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
        case NORMAL:
            //do not do anything - shouldn't ever get here
            break;
        case SET_CLK:
            add = enc_lookup[encoder1_hist & 0b1111]; //add one
            min += add; // add number to min

            //bound the new minute setting
            if(min > 59)
                min = 0;
            if(min < 0)
                min = 59;

            break; //SET_CLK
        case SET_ALARM:
            add = enc_lookup[encoder1_hist & 0b1111]; //add four
            alarm_min += add; // add number to sum

            //bound the new minute setting
            if(alarm_min > 59)
                alarm_min = 0;
            if(alarm_min < 0)
                alarm_min = 59;

            break; //SET_ALARM

        default:
            break;

    }//switch

}//get_encoder1


//***********************************************************************************
//                                   Encoder 2
// 
//***********************************************************************************
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
        case NORMAL:
            //do not do anything - shouldn't ever get here
            break;
        case SET_CLK:
            add = enc_lookup[encoder2_hist & 0b1111]; //add one
            hrs += add; // add number to hrs

            //bound the new hours setting
            switch(twelve_hr_format)
            {
                case TRUE:
                    if(hrs > 12)
                        hrs = 1;
                    if(hrs < 1)
                        hrs = 12;
                    break;
                case FALSE:
                    if(hrs > 23)
                        hrs = 0;
                    if(hrs < 0)
                        hrs = 23;
                    break;
            }//switch
            break; //SET_CLK

        case SET_ALARM:
            add = enc_lookup[encoder2_hist & 0b1111];
            alarm_hrs += add;

            //bound the new hours setting
            switch(twelve_hr_format)
            {
                case TRUE:
                    if(alarm_hrs > 12)
                        alarm_hrs = 1;
                    if(alarm_hrs < 1)
                        alarm_hrs = 12;
                    break;
                case FALSE:
                    if(alarm_hrs > 23)
                        alarm_hrs = 0;
                    if(alarm_hrs < 0)
                       alarm_hrs = 23;
                    break;
            }//switch
        break; //SET_ALARM

        default:
            break;

    }//switch

}//encoder2


//***********************************************************************************
//                                   SPI Total Functionallity
//
//***********************************************************************************
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
    SPDR = (~current_mode); // send the bar graph the current status
    while(bit_is_clear(SPSR, SPIF)) {} // wait until encoder data is recieved
    data = SPDR;

    //********** Bar Graph Portion *******************
    PORTB |= 0x01;      // move graph data from shift to storage reg.
    PORTB &= ~0x01;     // change 3-state back to high Z

    //********** Pass Encoder Info to Functions ******
    encoder1_instruction(data);
    encoder2_instruction(data >> 2);

}//SPI_function


//***********************************************************************************
//                                   mode_handler
//
//***********************************************************************************
void mode_handler() {

    switch(current_mode)
    {

//********************************* NORMAL MODE **************************************
//***********************************************************************************
        case NORMAL:
            //do not do anything
            break;

//*************************** TOGGLE CLOCK FORMAT MODE *******************************
//***********************************************************************************
        case TOGGLE_CLK_FORMAT:
            twelve_hr_format ^= TRUE; //if change format button is pushed, toggle
            
            switch(twelve_hr_format)
            {
                case FALSE:
                    twelve_to_twfour();
                    break;
                case TRUE:
                    twfour_to_twelve();
                    break;
            }//switch

            current_mode = NORMAL;

            break;

//***************************** SET CLOCK MODE *************************************
//***********************************************************************************
        case SET_CLK:

            TCCR0 = (0 << CS00) | (0 << CS01) | (0 << CS02); //disable real clock
            TCNT0 = 0x00; //reset counter
            sec = 0;
            Colon_Status = TRUE; //turn colon on

            SPI_function();

            // TODO: could put this line the "get button input" function when
            // changing back to NORMAL mode.
            TCCR0 |= (1 << CS02) | (1 << CS00); //turn clock back on
            break;

//***************************** SET ALARM MODE *************************************
//***********************************************************************************
        case SET_ALARM:
            
            format_clk_array(alarm_hrs, alarm_min);
            Colon_Status = TRUE;

            SPI_function();

            if(alarm_on)
                send_lcd(0x00, 0x0C);
            else
                send_lcd(0x00, 0x08);

            break;
        default:
            break;

    }//switch
}//mode_handler



//***********************************************************************************
//                                   Interrupt Routine
//
//***********************************************************************************
ISR(TIMER0_OVF_vect) {
    
    PORTC &= ~(1 << 0);
    step_time();
    PORTC |= (1 << 0);

}//Timer0 overflow ISR

ISR(TIMER3_OVF_vect) {

   // PORTC &= ~(1 << 4);
    // Start ADC conversion
    ADCSRA |= (1 << ADSC);

    PORTC &= ~(1 << 4);
    uint8_t old_DDRA = DDRA;
    uint8_t old_PORTA = PORTA;
    uint8_t old_PORTB = PORTB;

    get_button_input();
    mode_handler();
    SPI_send(~current_mode);

    DDRA = old_DDRA;
    PORTA = old_PORTA;
    PORTB = old_PORTB;

   // PORTC |= (1 << 4);

}//Timer2 overflow ISR


ISR(TIMER2_COMP_vect) {
    PORTC &= ~(1 << 3);
    PORTF ^= (1 << 0);
    PORTC |= (1 << 3);
}//Timer3 overflow ISR


/*ISR(TIMER3_COMPA_vect) {
    PORTC ^= (1 << 0);
}*/

ISR(ADC_vect) {
    OCR2 = ADCH;
}//ADC conversion ISR


//***********************************************************************************
//***********************************************************************************
//                                   MAIN
//***********************************************************************************

int main()
{

// set port bits 4-7 B as outputs
// set port bits 0-3 B as outputs (output mode for SS, MOSI, SCLK)
DDRB = 0xF7;
PINB = 0b1000;

// initialize the real time clock and initial clock display
real_clk_init();
format_clk_array(hrs, min);


// encoder is on PORTE
// volume is tied to OC3A on PE3
DDRE = 0xFF;
DDRC = 0xFF;
//DDRF = 0xFF; // make pin 0 GND, and pin 1 HIGH
//PORTF = 0b0010;



//setup timer counter 1 to run in CTC mode. 
TCCR1A |= (0<<COM1A1) | (0<<COM1A0);                // disable compare output pins 
TCCR1B |= (0<<WGM13) | (1<<WGM12) | (1<<CS10);      //use OCR1A as source for TOP, use clk/1
TCCR1C = 0x00;          //no forced compare 
OCR1A = 0x8000;         //clear at 0x8000. 16MHz/0x8000 = 488.28Hz = 0.002 Sec
TIMSK |= (1 << OCIE1A); // enable interrupt when timer resets


//setup timer counter 3 as the interrupt source, 30 interrupts/sec
// (16,000,000)/(32,768) = 488 cycles/sec
TCCR3A |= (1 << COM3B1) | (1 << WGM30) |  (1 << WGM31); //fast PWM mode, non-inverting
TCCR3B |= (1 << WGM32) | (1 << WGM33) | (1 << CS30); //fast PWM and clk/1  (488hz)  
//TCCR3C = 0X00;          //no forced compare
OCR3A = 0x8000;          //define TOP of counter
OCR3B = 0x2AAA; //define the volume dc in the compare register
//ETIMSK = (1 << TOIE3);   //enable interrupt on overflow and compare,
                         //check buttons and get new duty cycle, 

// set up timer and interrupt (16Mhz / 256 = 62,500Hz = 16uS)
// OC2 will pulse PB7 which is what the LED board PWM pin is connected to
TCCR2 |= (1 << WGM21) | (1 << WGM20) | (1 << COM20) \
         | (1 << COM21) | (1 << CS21); // set timer mode (PWM, no prescalar, inverting)
OCR2 = 0xF9;
//TIMSK |= (1 << OCIE2); // turn on timer interrupts

// set up ADC
DDRF  &= ~(_BV(DDF7)); //make port F bit 7 is ADC input  
PORTF &= ~(_BV(PF7));  //port F bit 7 pullups must be off
ADMUX = (1 << ADLAR) | (1 << REFS0) | (1 << MUX0) | (1 << MUX1) \
        | (1 << MUX2); // set reference voltage to external 5V
ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) \
        | (1 << ADPS1) | (1 << ADPS2); // enable ADC, enable interrupts, enable ADC0
                                      // 128 prescaler (16,000,000/128 = 125,000)

// set up SPI (master mode, clk low on idle, leading edge sample)
SPCR = (1 << SPE) | (1 << MSTR) | (0 << CPOL) | (0 << CPHA);
SPSR = (1 << SPI2X);

lcd_init(); // initialize the lcd screen
string2lcd(alarm_msg); // sent msg
send_lcd(0x00, 0x08); // turn diplay off
sei(); // enable global interrupts

while(1){
    switch(current_mode)
    {
        case NORMAL:
            format_clk_array(hrs, min);
            break;
        case SET_CLK:
            format_clk_array(hrs, min);
            break;
        case SET_ALARM:
            format_clk_array(alarm_hrs, alarm_min);
            break;
    }//switch
    update_LEDs();
}//while

return 0;
}//main

