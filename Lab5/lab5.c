
/**********************************************************************
 * Copywrite: NONE
 * Original Author(s): Jesse Ulibarri
 * Original Date: 10/17/16
 * Version: Lab5.1
 * Description: ATMega128 will track real time to be displayed on the
 *  seven-segment, five-digit board. Eight-button board will receive
 *  user input and change the system's state. Based on the state,
 *  users will be able to change the current time and alarm time by
 *  using the encoder board. Current system state will be displayed 
 *********************************************************************/

/**********************************************************************
* Class: ECE 473
* Assignment: Lab5
*
*  HARDWARE SETUP:
*  PORTA is connected to the segments of the LED display. and to the pushbuttons.
*  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
*  
*             ***** LED_GRAPH_BOARD *****
*  PORTB bit 0 (SS_n) goes to REGLCK on graph board
*  PORTB bit 1 (SCLK) goes to SRCLK on graph board
*  PORTB bit 2 (MOSI) goes to SDIN on graph board
*      OE_N goes to ground on AVR
*      GND goes to ground on AVR
*      VDD goes to VCC on AVR
*      SD_OUT is not connected
*
*             ***** ENCODER_BOARD *****
*  PORTB bit 1 (SCLK) goes to SCK on encoder board
*  PORTB bit 3 (MISO) goes to SER_OUT on encoder board
*  PORTE bit 2 goes to SH/LD on encoder board
*  PORTE bit 3 goes to CLK_INH on encoder board
* 
*             ***** BUTTON_BOARD *****
*  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
*  PORTB bit 7 goes to the PWM transistor base.
*********************************************************************/

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"
#include "twi_master.h"
#include "lm73_functions.h"
#include "uart_functions.h"

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

//For debugging
#define SHOW_INTERRUPTS TRUE
#define TCNT0_ISR       0x01
#define TCNT1_ISR       0x02
#define TCNT3_ISR       0x03
#define ADC_ISR         0x04
#define TWI_ISR         0x05
#define USART0_ISR      0x06
#define NOT_IN_ISR      0xF8

//Select digit codes
#define SEL_DIGIT_1 0x40 
#define SEL_DIGIT_2 0x30
#define SEL_DIGIT_3 0x10
#define SEL_DIGIT_4 0x00
#define SEL_COLON   0x20
#define ENABLE_TRISTATE 0x70    // tristate is enabled by Y7 decoder output.
                                // ENABLE_TRISTATE are the bits on PORTB that
                                // need to be set to get a low output on Y7.
#define DISABLE_TRISTATE 0x60

// Define different modes
#define NORMAL              0xFF
#define TOGGLE_CLK_FORMAT   0x7F
#define SET_CLK             0xBF
#define SET_ALARM           0xDF

// Declare init functions. They are at bottom out of the way
void real_clk_init();
void timer1_init();
void timer2_init();
void timer3_init();
void SPI_init();
void ADC_init();

uint8_t current_mode = NORMAL;

int8_t hrs = 12;
int8_t min = 0;
uint8_t sec = 0;
uint8_t AM = TRUE;

int8_t alarm_hrs = 12;
int8_t alarm_min = 0;
int8_t alarm_sec = 0;
uint8_t alarm_AM = TRUE;

uint8_t Colon_Status = FALSE;
uint8_t twelve_hr_format = TRUE;
uint8_t alarm_on = FALSE;
uint8_t alarm_going_off = FALSE;

// TWI arrays
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];
uint16_t lm73_temp;
char lm73_char_temp[8];
char remote_temp;
uint8_t remote_byte = 1;

char mode_text[16] = "Normal Mode     ";
char temp_text[16] = "In:   C Out:   C";
//char lcd_display[32] = lcd_line1 + lcd_line2;
char lcd_display[32];
//= {'N', 'O', 'R', 'M', 'A', 'L',  ' ', 'M', 'O', 'D', 'E', ' ', ' ', ' ', ' ', ' ',\
            ' ', ' ', ' ', ' ', ' ', ' ',  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[10] = {ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE};

//array that holds the segment codes
uint8_t segment_codes[5] = {SEL_DIGIT_4, SEL_DIGIT_3, SEL_COLON, SEL_DIGIT_2, SEL_DIGIT_1};

//look up table to determine what direction the encoders are turning
int8_t enc_lookup[16] = {0,0,0,0,0,0,0,1,0,0,0,-1,0,0,0,0};


/******************************************************************************
* Function: chk_buttons
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


/***********************************************************************************
* Function: format_clk_array
* Parameters: hours holds current hour, minutes holds current minutes.
* Return: none
* Description: Takes a 16-bit binary input value and places the appropriate 
*   equivalent 4 digit BCD segment code in the array segment_data for display. 
*   Array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
*******************************************************************************/

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
    else if(current_mode != SET_ALARM && !AM && twelve_hr_format)
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

/******************************************************************************
* Function: clk_boundary
* Parameter: none
* Return: none
* Description: This function bounds the current count to within the max limit. 
*   It then calls the segsum function which will format our value into the 
*   segment data array.
*******************************************************************************/

void clk_boundary() {

    switch(twelve_hr_format) 
    {
        case TRUE:
            if(sec == 60) { min += 1; sec = 0; }
            if(min == 60) { hrs += 1; min = 0; }
            if(hrs == 13) { hrs = 1; AM ^= TRUE; }
            break;

        case FALSE:
            if(sec == 60) { min += 1; sec = 0; }
            if(min == 60) { hrs += 1; min = 0; }
            if(hrs == 24) { hrs = 0; }
            break;
        }//switch
}//clk_boundary

/***********************************************************************************
* Function: step_time
* Parameters: none
* Return: none
* Description:
*
*******************************************************************************/

void step_time() {


    Colon_Status = FALSE;   // part of colon "one-shot". Turn colon OFF every interrupt
    sec += 1;               // increment the second count

    clk_boundary();


    //check if alarm should go off
    if(alarm_on) {

        //toggle the clk to produce a beep
        if(alarm_going_off) { TCCR1B ^= (1 << CS10); }
        else if((hrs == alarm_hrs) && (min == alarm_min) && (sec == alarm_sec) && (AM == alarm_AM))
            alarm_going_off = TRUE;
    }

}//step_time


/***********************************************************************************
* Functions: twelve_to_twfour, twfour_to_twelve
* Parameters: none
* Return: none
* Description: Converts the clock from twelve hour time to military time
*   or vise versa.
*******************************************************************************/

void twelve_to_twfour() {
    if(!AM) hrs += 12;
}//twelve_to_twfour

void twfour_to_twelve() {
    if(hrs > 12) { hrs -= 12; AM = FALSE; }
}//twfour_to_twelve


/******************************************************************************
* Function: SPI_send
* Parameters: message var holds int to be sent
* Return: none
* Description: Function will take in a message to send through SPI. It will 
*   write the data to the SPI data register and then wait for the message to 
*   send before returning.
*******************************************************************************/

void SPI_send(uint8_t message) {
    PORTE &= ~(1 << PE5); // enable bar graph

    SPDR = message; // write message to SPI data register
    while(bit_is_clear(SPSR, SPIF)) {} // wait for data to send

    PORTE |= (1 << PE6);      // move data from shift to storage reg.
    PORTE &= ~(1 << PE6);     // change 3-state back to high Z

    PORTE |= (1 << PE5); // disable bar graph

}//SPI_send


/***********************************************************************************
* Function: SPI_read
* Parameters: none
* Return: 8 bit int from SPI peripherial
* Description: Function will read any data coming through the SPI communication bus.
*   It will write a garbage value to the SPI data register to initialize 
*   communication and then wait for the data to be sent. At this time, any incoming 
*   data has entered the SPI data register. The function now returns the read data.
*
* NOT IN USE
*******************************************************************************/

uint8_t SPI_read() {

    // Here is an example of internal commenting used to describe several lines 
    // of code.
    if(1) {
        PORTE |= (1 << PE2); //shift data from encoder into it's internal register
        __asm__ __volatile__ ("nop");
        __asm__ __volatile__ ("nop");
        PORTE &= ~(1 << PE2); //end shift

        SPDR = 0x00; // send junk to initialize SPI return
        while(bit_is_clear(SPSR, SPIF)) {} // wait until data is recieved
    }//if
    return SPDR;

}//SPI_read


/***********************************************************************************
* Function: get_button_input
* Parameters: none
* Return: none
* Description: Function will get any input from the button board and load the
*   information into the segment_data array.
*******************************************************************************/

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
                if(chk_buttons(i)) { current_mode &= ~(1 << i); }
            }
            
            if(alarm_going_off) {
                //snooze function
                if(chk_buttons(3)) {
                    TCCR1B &= ~(1 << CS10);
                    alarm_going_off = FALSE;
                    alarm_sec = sec + 10;
                }
                //turn alarm off
                if(chk_buttons(2)) {
                    TCCR1B &= ~(1 << CS10);
                    alarm_going_off = FALSE;
                    alarm_on = FALSE;
                    alarm_sec = 0;
                    memcpy(mode_text, "Normal Mode     ", 16);
                }
            }//if alarm_going_off
            break;

        case SET_CLK:

            switch(twelve_hr_format)
            {
                case TRUE:
                    if(chk_buttons(7)){ AM ^= TRUE; }
                    break;
                case FALSE:
                    break;
            }
            
            // exit SET_CLK mode
            if(chk_buttons(6)) {
                current_mode = NORMAL;
                TCCR0 |= (1 << CS02) | (1 << CS00); //turn clock back on
            }

            break;

        case SET_ALARM:
            
            switch(twelve_hr_format)
            {
                case TRUE:
                    if(chk_buttons(7)) { alarm_AM ^= TRUE; }
                    break;
                case FALSE:
                    break;
            }
            if(chk_buttons(0)) {
                alarm_on ^= TRUE;
                    if(alarm_on) { memcpy(mode_text, "Alarm Armed     ", 16); }
                    else { memcpy(mode_text, "Normal Mode     ", 16); }
            
            }
            // exit SET_ALARM mode
            if(chk_buttons(5)) { current_mode = NORMAL; }
            break;
            
    }//switch

    // disable the tristate buffer
    PORTB = DISABLE_TRISTATE;
//    DDRA = 0xFF; //set PORTA back to output

}//get_button_input


/***********************************************************************************
* Function: update_LEDs
* Parameters: none
* Return: none
* Description: Function will send the data in the segment data array to the 
*   7-segment board and then wait 0.5 ms on each value to allow the LED to be 
*   on long enough to produce a bright output.
*******************************************************************************/

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

        PORTB = segment_codes[num_digits]; // send PORTB the digit to desplay
        PORTA = segment_data[num_digits];  // send 7 segment code to LED segments

        // wait a moment
        _delay_ms(0.1);
    }//for
    PORTA = OFF; // turn off port to keep each segment on the same amount of time
    __asm__ __volatile__ ("nop");
    __asm__ __volatile__ ("nop");

}//update_LEDs


/***********************************************************************************
* Function: encoder1_instructions
* Parameters: encoder1_val is the binary value coming from encoder 1
* Return: none
* Description: Function will receive the raw data brought in from the encoder 
*   board, interperate the data, and add the correct value to the sum variable 
*   based on the recieved encoder status and current mode.
*******************************************************************************/

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
            if(min > 59) min = 0;
            if(min < 0) min = 59;

            break; //SET_CLK
        case SET_ALARM:
            add = enc_lookup[encoder1_hist & 0b1111]; //add four
            alarm_min += add; // add number to sum

            //bound the new minute setting
            if(alarm_min > 59) alarm_min = 0;
            if(alarm_min < 0) alarm_min = 59;

            break; //SET_ALARM

        default:
            break;

    }//switch

}//get_encoder1


/***********************************************************************************
* Function: encoder2_instruction
* Parameters: encoder2_val is the binary value coming from encoder 2
* Return: none
* Description: This function is the same as the encoder1 function except that
*   it will interperate the data coming from encoder 2.
*******************************************************************************/

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
                    if(hrs > 12) hrs = 1;
                    if(hrs < 1) hrs = 12;
                    break;
                case FALSE:
                    if(hrs > 23) hrs = 0;
                    if(hrs < 0) hrs = 23;
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
                    if(alarm_hrs > 12) alarm_hrs = 1;
                    if(alarm_hrs < 1) alarm_hrs = 12;
                    break;
                case FALSE:
                    if(alarm_hrs > 23) alarm_hrs = 0;
                    if(alarm_hrs < 0) alarm_hrs = 23;
                    break;
            }//switch
        break; //SET_ALARM

        default:
            break;

    }//switch

}//encoder2


/***********************************************************************************
* Function: SPI_function
* Parameters: none
* Return: none
* Description: Function will send the current mode data to the graph board and 
*   receive data from the encoder at the same time. It will then call the encoders 
*   1 and 2 functions to interperate the encoder data.
*******************************************************************************/

void SPI_function() {
    uint8_t data;

    //************ Encoder Portion *******************
    PORTE &= ~(1 << PE5); // enable bar graph
    PORTE &= ~(1 << PE2); //shift encoder data into register
    __asm__ __volatile__ ("nop");
    __asm__ __volatile__ ("nop");
    PORTE |= (1 << PE2); //end shift

    //*********** Send and Receive SPI Data **********
    SPDR = (~current_mode); // send the bar graph the current status
    while(bit_is_clear(SPSR, SPIF)) {} // wait until encoder data is recieved
    data = SPDR;
    
    //********** Bar Graph Portion *******************
    PORTE |= (1 << PE6);      // move graph data from shift to storage reg.
    PORTE &= ~(1 << PE6);     // change 3-state back to high Z
    PORTE |= (1 << PE5);      // disable bar graph


    //********** Pass Encoder Info to Functions ******
    encoder1_instruction(data);
    encoder2_instruction(data >> 2);

}//SPI_function


/***********************************************************************************
* Function: mode_handler
* Parameters: none
* Return: none
* Description: Mode handler will determine what functions to execute on each
*   interrupt depending on the mode of the machine. Possible modes are: NORMAL,
*   TOGGLE_CLK_FORMAT, SET_CLK, or SET_ALARM. 
*******************************************************************************/
void mode_handler() {

    switch(current_mode)
    {

/********************************* NORMAL MODE **************************************
************************************************************************************/
        case NORMAL:
            //Do not do anything
            break;

/*************************** TOGGLE CLOCK FORMAT MODE *******************************
************************************************************************************/
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

/***************************** SET CLOCK MODE *************************************
************************************************************************************/
        case SET_CLK:

            TCCR0 = (0 << CS00) | (0 << CS01) | (0 << CS02); //disable real clock
            TCNT0 = 0x00; //reset counter
            sec = 0;
            Colon_Status = TRUE; //turn colon on

            SPI_function();

            break;

/***************************** SET ALARM MODE *************************************
************************************************************************************/
        case SET_ALARM:
            
            format_clk_array(alarm_hrs, alarm_min);
            Colon_Status = TRUE;

            SPI_function();

            break;
        default:
            break;

    }//switch
}//mode_handler


/***********************************************************************************
************************************************************************************
*                                   Interrupt Routines                             *
************************************************************************************
***********************************************************************************/


/***********************************************************************************
* Description: Interrupts every second to track real time.
***********************************************************************************/
ISR(TIMER0_OVF_vect) {

    static int8_t i;
    PORTC |= (1 << PC5);
    
    step_time();

    //format temp array
    lm73_temp = (lm73_rd_buf[0] << 8) | (lm73_rd_buf[1]);
    lm73_temp = lm73_temp >> 7;
    itoa(lm73_temp, lm73_char_temp, 10);
    for(i = 0; i < 2; i++)
        temp_text[i+4] = lm73_char_temp[i];

    //begin a new temp request
    twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2);
    
    //request remote temp through uart
    //uart_putc(0xF0);
    while(!(UCSR0A & (1 << UDRE0)));
    UDR0 = 0xF0;


    PORTC &= ~(1 << PC5);

}//Timer0 overflow ISR


/***********************************************************************************
* Description: Interrupt drives the alarm tone.
*  PWM into input of OPAMP.
***********************************************************************************/

ISR(TIMER1_COMPB_vect) {

    PORTC ^= (1 << 0);

}//Timer1 compare B ISR


/***********************************************************************************
* Description: Interrupt drives the alarm tone.
*  PWM into input of OPAMP.
***********************************************************************************/
ISR(TIMER2_OVF_vect) {

    //PORTC |= (1 << PC4);

    // Start ADC conversion (get light input)
    ADCSRA |= (1 << ADSC);
    
    uint8_t old_DDRA = DDRA;
    uint8_t old_PORTA = PORTA;
    uint8_t old_PORTB = PORTB;

    get_button_input();         //any user input to change mode?
    mode_handler();             //call correct functions depending on mode
    SPI_send(~current_mode);    //send mode to the bar graph

    DDRA = old_DDRA;
    PORTA = old_PORTA;
    PORTB = old_PORTB;

    refresh_lcd(lcd_display);

    //PORTC &= ~(1 << PC4);

}//Timer2 overflow ISR


/***********************************************************************************
* Description: Interrupt occurs when an ADC conversion is complete.
*   On each interrupt, the brightness of the LED display is updated.
***********************************************************************************/

ISR(ADC_vect) {

    OCR2 = ADCH;

}//ADC converter ISR


/***********************************************************************************
* Description: Received a temp byte from the ATMega48
***********************************************************************************/

ISR(USART0_RX_vect) {

    PORTC |= (1 << PC4);

    remote_temp = uart_getc();
    //first or second byte?
    if(remote_byte == 1) {
        temp_text[13] = remote_temp;
        remote_byte++;
    }
    else {
        temp_text[14] = remote_temp;
        remote_byte = 1;
    }

    PORTC &= ~(1 << PC4);

}//USART0 receive ISR



/***********************************************************************************
************************************************************************************
*                                   MAIN                                           *
************************************************************************************
***********************************************************************************/

int main()
{
uint8_t i;
// set port bits 4-7 B as outputs
// set port bits 0-2 B as outputs (output mode for SS, MOSI, SCLK)
// set port bit 3 as input (MISO) with pull-ups
DDRB = 0xF7;
PINB = (1 << PB3);
// Alarm tone is generated on PC0
// Logic timing on PC4
DDRC = (1 << PC0) | (1 << PC4) | (1 << PC5) | (1 << PC6);
// encoder is on PE2 and PE3
// bar graph ~OE is on PE5
// volume is tied to OC3A on PE3
DDRE = (1 << PE2) | (1 << PE3) | (1 << PE4) | (1 << PE5) | (1 << PE6);
// For debugging
DDRG |= (1 << PG0) | (1 << PG1) | (1 << PG2);

// initialize the real time clock and initial clock display
real_clk_init();
timer1_init();
timer2_init();
timer3_init();
ADC_init();
SPI_init();
format_clk_array(hrs, min);
lcd_init();             // initialize the lcd screen
clear_display();
init_twi();
uart_init();

lm73_wr_buf[0] = 0x00; //load lm72_wr_buf[0] with temp pointer address
twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 2); //start the "set-up" write

sei();                  // enable global interrupts

while(1){
    
    //format the led display
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

    //format what is sent to the lcd display 
    for(i = 0; i < 16; i++) {
        lcd_display[i] = mode_text[i];
        lcd_display[i+16] = temp_text[i];
    }

    update_LEDs();

}//while

return 0;
}//main



/******************************************************************************
* Function: real_clk_init
* Parameters: none
* Return: none
* Description: This function initializes timer 0 to track real time. The
*   timer uses the 32kHz external clock. There are specific procedures
*   found in the datasheet that initializes this oscillator.
*   External clock runs at ~32kHz.
*   Elapsed time = 32,768Hz / (256 * 128) = 1 sec
*******************************************************************************/

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


void timer1_init() {
	//setup timer counter 1 to run in Fast PWM mode. Timer generates the alarm tone 
	TCCR1A |= (1 << WGM10) | (1 << WGM11);               //fast PWM mode, OC pin disabled 
	TCCR1B |= (1 << WGM12) | (1 << WGM13) | (0 << CS10); //use OCR1A as source for TOP, use clk/1
	TCCR1C = 0x00;          //no forced compare 
	OCR1A = 0x8000;         //clear at 0x8000. 16MHz/0x8000 = 488.28Hz = 0.002 Sec
	OCR1B = 0x4000;         //create DC of tone
	TIMSK |= (1 << OCIE1B); // enable interrupt when timer resets
}//timer1_init


void timer2_init() {
	// set up timer and interrupt (16Mhz /(8*256) = 7,813Hz = 128uS)
	// OC2 will pulse PB7 which is what the LED board PWM pin is connected to
	TCCR2 |= (1 << WGM21) | (1 << WGM20) | (1 << COM20) \
 	        | (1 << COM21) | (1 << CS21) | (1 << CS20); // set timer mode (PWM, no prescalar, inverting)
	OCR2 = 0xF9;
	TIMSK |= (1 << TOIE2);
}//timer2_init


void timer3_init() {
	// timer 3 controls frequency of checking buttons as well as volume control
	// (16,000,000)/(16,384) = 976 cycles/sec = 1.024mS
	TCCR3A |= (1 << COM3B1) | (1 << WGM30) |  (1 << WGM31); //fast PWM mode, non-inverting
	TCCR3B |= (1 << WGM32) | (1 << WGM33) | (1 << CS30); //fast PWM and clk/1 (976Hz)  
	//TCCR3C = 0X00;         //no forced compare
	OCR3A = 0x2000;          //define TOP of counter
	OCR3B = 0x1000;          //define the volume dc in the compare register
	//ETIMSK = (1 << TOIE3);   //enable interrupt on overflow and compare,
                         //check buttons and get new duty cycle, 
}//timer3_init


void SPI_init() {
	// set up SPI (master mode, clk low on idle, leading edge sample)
	SPCR = (1 << SPE) | (1 << MSTR) | (0 << CPOL) | (0 << CPHA);
	SPSR = (1 << SPI2X);
}//SPI_init


void ADC_init() {
	// set up ADC (get light level)
	DDRF  &= ~(_BV(DDF7)); //make port F bit 7 is ADC input  
	PORTF &= ~(_BV(PF7));  //port F bit 7 pullups must be off
	ADMUX = (1 << ADLAR) | (1 << REFS0) | (1 << MUX0) | (1 << MUX1) \
	        | (1 << MUX2); // set reference voltage to external 5V
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) \
	        | (1 << ADPS1) | (1 << ADPS2); // enable ADC, enable interrupts, enable ADC0
                                       // 128 prescaler (16,000,000/128 = 125,000)	
}//ADC_init
