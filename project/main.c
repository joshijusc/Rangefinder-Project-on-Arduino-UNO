/********************************************
 *
 *  Name: Jason Joshi
 *  Email: joshij@usc.edu
 *  Section: 11am Friday
 *  Assignment: Project - Rangefinder
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <stdlib.h>     /* abs */
#include <avr/eeprom.h>
#include "lcd.h"
#include "timers.h"
#include "debounce.h"

#define FOSC 16000000 // Clock frequency
#define BAUD 9600 // Baud rate used
#define MYUBRR (FOSC/16/BAUD-1) // Value for UBRR0

// Main functions
char rx_char();
void tx_char(char ch);
void play_note(uint16_t freq);

// Frequencies for natural notes from middle C (C4)
// up one octave to C5.
uint16_t frequency[8] =
    { 262, 294, 330, 349, 392, 440, 494, 523 };

//Global variables
volatile uint8_t echoChanged = 0;  // Flag for sensor change
volatile uint8_t new_state_loc, old_state_loc;
volatile uint8_t new_state_rem, old_state_rem;
volatile uint8_t a, b;
volatile int16_t countLoc = 0;		// Count to display
volatile int16_t countRem = 0;		// Count to display
volatile int16_t isrCount = 0;		// Count for ISR
volatile int16_t isrInvoked = 0;		// Count for ISR
volatile int16_t isrInvokedTimer0 = 0;		// Count for ISR
volatile uint16_t pulse_count = 0;		// Count for pulse ISR
volatile long distance = 5000; // Rangefinder measurement
volatile long decimal;
volatile int16_t started;           // Flag to make sure changed isn't altered when in the Exceeded Range
volatile long width; // Slope of range for servo motor
volatile int16_t pwmTimer1 = 0;		// PWM for Timer 1
volatile uint16_t rotaryChangedLoc = 0;
volatile uint16_t rotaryChangedRem = 0;
volatile char rangeString[6];
volatile char receivedString[5]; // Stores received data from the other breadboard
volatile int remoteDistInt = 0;
volatile uint16_t dataFullyReceived = 0; // Flag to see if ">" has been read in
volatile uint16_t dataStarted = 0; // Flag to see if "<" has been read in
volatile uint16_t numCharactersRead = 0; // Counter to see how characters are read in
volatile uint16_t noteChanged = 0;
enum states {LOCAL, REMOTE};
uint8_t state = LOCAL;

int main(void) {
    UBRR0 = MYUBRR; // Set baud rate
    UCSR0B |= (1 << TXEN0 | 1 << RXEN0); // Enable RX and TX
    UCSR0C |= (3 << UCSZ00); // Async., no parity,
    UCSR0B  |=  (1 << RXCIE0); // USART ISR setup
    // 1 stop bit, 8 data bits

    // Timer and LCD initializations
    timer0_init();
	timer1_init();
	timer2_init();
    lcd_init();

	// Setting up the interrupt handlers
    PCICR |= (1<<PCIE1) | (1<<PCIE2);
    PCMSK1 |= ((1<<PCINT9)|(1<<PCINT10));
    PCMSK2 |= ((1<<PCINT18)|(1<<PCINT19));

    // Initialize the global interrupt
    sei();

    // Initialize DDR and PORT registers and LCD
	PORTB |= (1 << PB4); // Acquire button pull-up resistor
    DDRC |= (1 << PC1) | (1 << PC4) | (1 << PC5); // Set LED, Buzzer, Rangefinder trigger to output
    DDRC &= ~(1 << PC0); // Set adjust button to input
    DDRB |= (1 << PB3) | (1 << PB5); // Set servo and LED output to output
   	PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD0); // Rotary encoder pull-up resistors
    DDRC |= (1 << PC3); // Tri-state buffer enable signal set to output
    PORTC &= ~(1 << PC3); // Tri-state buffer enable signal set to output

    // Write a spash screen to the LCD
	char buf[20];
    char* title = "EE109 Project";
    snprintf(buf, 15, "%s", title);
    lcd_moveto(0, 0);
    lcd_stringout(buf);
    char* name = "Jason Joshi";
    snprintf(buf, 15, "%s", name);
    lcd_moveto(1, 0);
    lcd_stringout(buf);
    _delay_ms(1000);
    lcd_writecommand(1);

    // Rotary setup
	a = (PIND & (1<<PD2));
    b = (PIND & (1<<PD3));
    if (!b && !a) old_state_rem = 0;
    else if (!b && a) old_state_rem = 1;
    else if (b && !a) old_state_rem = 2;
    else old_state_rem = 3;
    new_state_rem = old_state_rem;
    if (!b && !a) old_state_loc = 0;
    else if (!b && a) old_state_loc = 1;
    else if (b && !a) old_state_loc = 2;
    else old_state_loc = 3;
    new_state_rem = old_state_loc;
    
    // Set LED to blue initially
    PORTC |= (1<<PC4);
    PORTB |= (1<<PB5);

    countLoc = eeprom_read_word((void *) 100); // Local address 100
    countRem = eeprom_read_word((void *) 200); // Remote address 200
    // Set back to default if eeprom out of range initially
    if (countLoc < 1 || countLoc > 400) countLoc = 1; // Set back to default
    if (countRem < 1 || countRem > 400) countRem = 1; // Set back to default


    while (1) {                 // Loop forever
        if (dataFullyReceived == 1){ // Received data
            // convert the range data from from a
            // string of ASCII characters to a single fixed-point number
            remoteDistInt;
            sscanf(receivedString, "%d", &remoteDistInt);
            // Print remote distance to LCD
            char remoteDistBuf[6];
            snprintf(remoteDistBuf, 7, "%d.%d", remoteDistInt/10, remoteDistInt%10);
            lcd_moveto(1, 9);
            lcd_stringout(remoteDistBuf);
            lcd_stringout("     ");
            dataFullyReceived = 0;
            if (remoteDistInt/10 <= countRem && state != LOCAL){
                play_note(500);
            }
        }
        if ((PINB & (1<<PB4)) == 0) {
            started = 0;
            debounceB(PB4); // Generate trigger pulse
            PORTC |= (1<<PC1);
            _delay_us(10);
            PORTC &= ~(1<<PC1);
        }
        if (isrInvoked == 1){ // Out of range case
            isrInvoked = 0;
            echoChanged = 0;
            lcd_moveto(0, 9);
            lcd_stringout(">400  ");
            // LED to blue
            PORTC |= (1<<PC4);
            PORTB |= (1<<PB5);
            isrInvoked = 0;
            echoChanged = 0;
            distance = 5000; // Set to large value to keep out of range case working
            noteChanged = 0;
        } else {
            if (echoChanged) { // New ultrasonic sensor reading
                distance = ((long)pulse_count*10) / (58*2); // Calculate the distance
                decimal = distance % 10; // Get the mod value to avoid the floating point numbers
                width = ((35*100000) - (distance * 575))/100000;
                char distBuf[12];
                snprintf(distBuf, 12, "%4ld.%ld", (distance/10), (decimal));
                lcd_moveto(0, 9); // Distance location
                lcd_stringout(distBuf);
                lcd_moveto(1, 0);
                lcd_stringout("Min= ");
                OCR2A = width;
                echoChanged = 0;
                if ((distance/10) <= 400 && (distance/10) >= 0){ // Distance is within range
                    snprintf(rangeString, 7, "<%ld>", distance);
                }
                int i = 0;
                while (rangeString[i] != '\0') {    // Loop until next charater is NULL byte
                    tx_char(rangeString[i]);  // Send the character
                    i++;
                }
            }
            if ((distance/10) <= 400 && (distance/10) >= 0){ // If reading is within range
                // Reprint on the screen if rotary is changed
                if (rotaryChangedLoc && state == LOCAL) {
                    lcd_moveto(1, 4);
                    char buf4[12];
                    snprintf(buf4, 12, "%03d", countLoc);
                    lcd_stringout(buf4);
                    rotaryChangedLoc = 0;
                    eeprom_update_word((void *) 100, countLoc);
                    if ((distance/10) < countLoc){ // Flash LED
                        //Green
                        PORTC &= ~(1<<PC4);
                        PORTB |= (1<<PB5);
                    } else {
                        // Red
                        PORTC |= (1<<PC4);
                        PORTB &= ~(1<<PB5);
                    }
                } else if (rotaryChangedRem && state == REMOTE) {
                    lcd_moveto(1, 4);
                    char buf5[12];
                    snprintf(buf5, 12, "%03d", countRem);
                    lcd_stringout(buf5);
                    rotaryChangedRem = 0;
                    eeprom_update_word((void *) 200, countRem);
                    noteChanged = 1;
                    if ((remoteDistInt/10) <= countRem){ // Flash LED
                        //Green
                        PORTC &= ~(1<<PC4);
                        PORTB |= (1<<PB5);
                    } else {
                        // Red
                        PORTC |= (1<<PC4);
                        PORTB &= ~(1<<PB5);
                    }
                }
                // Keeps the words Remote and Local printed on the screen
                if (state == REMOTE) {
                    lcd_moveto(0,0);
                    lcd_stringout("Remote");
                    lcd_moveto(1, 4);
                    char buf5[12];
                    snprintf(buf5, 12, "%03d", countRem);
                    lcd_stringout(buf5);
                    if ((remoteDistInt/10) <= countRem){ // Flash LED
                        //Green
                        PORTC &= ~(1<<PC4);
                        PORTB |= (1<<PB5);
                    } else {
                        // Red
                        PORTC |= (1<<PC4);
                        PORTB &= ~(1<<PB5);
                    }

                } else if (state == LOCAL) {
                    lcd_moveto(0,0);
                    lcd_stringout("Local ");
                    lcd_moveto(1, 4);
                    char buf4[12];
                    snprintf(buf4, 12, "%03d", countLoc);
                    lcd_stringout(buf4);
                    if ((distance/10) <= countLoc){ // Flash LED
                        //Green
                        PORTC &= ~(1<<PC4);
                        PORTB |= (1<<PB5);
                    } else {
                        // Red
                        PORTC |= (1<<PC4);
                        PORTB &= ~(1<<PB5);
                    }
                    noteChanged = 0;
                }

                if ((PINC & (1<<PC0)) == 0) { //Adjust button pressed
                    debounceC(PC0);
                    if (state == LOCAL){
                        lcd_moveto(0,0);
                        lcd_stringout("Remote"); // Go from local to remote
                        state = REMOTE;
                        lcd_moveto(1, 4);
                        char buf5[12];
                        snprintf(buf5, 12, "%03d", countRem);
                        lcd_stringout(buf5);
                        if ((remoteDistInt/10) <= countRem){  // Flash LED
                            //Green
                            PORTC &= ~(1<<PC4);
                            PORTB |= (1<<PB5);
                        } else {
                            // Red
                            PORTC |= (1<<PC4);
                            PORTB &= ~(1<<PB5);
                        }
                    } else { //if (state == REMOTE) {
                        lcd_moveto(0,0);
                        lcd_stringout("Local "); // Go from remote to local
                        state = LOCAL;
                        lcd_moveto(1, 4);
                        char buf4[12];
                        snprintf(buf4, 12, "%03d", countLoc);
                        lcd_stringout(buf4);
                        if ((remoteDistInt/10) <= countLoc){ // Flash LED
                            //Green
                            PORTC &= ~(1<<PC4);
                            PORTB |= (1<<PB5);
                        } else {
                            // Red
                            PORTC |= (1<<PC4);
                            PORTB &= ~(1<<PB5);
                        }
                        noteChanged = 0;
                    }
                }
            }
        }
	}
}

char rx_char()
{
    // Wait for receive complete flag to go high
    while ( !(UCSR0A & (1 << RXC0)) ) {}
        return UDR0;
}

void tx_char(char ch)
{
    // Wait for transmitter data register empty
    while ((UCSR0A & (1<<UDRE0)) == 0) {}
        UDR0 = ch;
}

/*
  Play a tone at the frequency specified for one second
*/
void play_note(uint16_t freq)
{
	isrCount = freq/2;
	isrInvokedTimer0 = 0;
	OCR0A = 62500 / (2 * freq);
    TCCR0B |= (0b100 << CS00);
    TCNT0 = 0;
}

/*
  ISR for serial interface
*/
ISR(USART_RX_vect)
{
    char receivedChar = UDR0;
    if (receivedChar == '<'){
        dataFullyReceived = 0;
        dataStarted = 1;
        numCharactersRead = 0;
    } else if (receivedChar == '>' && numCharactersRead > 0){
        dataFullyReceived = 1;
        dataStarted = 0;
        receivedString[numCharactersRead] = '\0';
        numCharactersRead = 0;
    } else if (receivedChar < '0' || receivedChar > '9' || (receivedChar == '>' && numCharactersRead == 0)){ // Invalid character case
        dataStarted = 0;
        int i;
        for (i=0; i<5; i++){ // Reset receivedString to null
            receivedString[i] = '\0';
        }
    }
    else { // Normal digit adding to buffer
        receivedString[numCharactersRead] = receivedChar;
        numCharactersRead++;
    }
}

/*
    ISR for rangefinder
*/
ISR(PCINT1_vect)
{
    if ((PINC & (1<<PC2)) != 0){ //Echo signal produced
        TCNT1 = 0;
        echoChanged = 0;
        TCCR1B |= (1 << CS11);    //Prescalar 8
        started = 1;
    } else if (started) {
        echoChanged = 1;        // Reset changed flag
        TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10)); // Set prescalar of 0
        pulse_count = TCNT1;
        started = 0;        
    }
}

/*
  ISR for rotary encoder 
*/
ISR(PCINT2_vect)
{
    // threshold values are stored in eeprom
    uint8_t input = PIND;
	a = input & (1 << PD2);
	b = input & (1 << PD3);
	if (a != 0) a = 1;
	if (b != 0) b = 1;

    if (state == LOCAL){ // Handle states for Local
        if (old_state_loc == 0) {
            // Handle A and B inputs for state 0
            if (a==1) {new_state_loc = 1; countLoc++;}
            if (b==1) {new_state_loc = 2; countLoc--;}
            if (a==0 && b==0) new_state_loc = 0;
        }
        else if (old_state_loc == 1) {
            // Handle A and B inputs for state 1
            if (a==0) {new_state_loc = 0; countLoc--;}
            if (b==1) {new_state_loc = 3; countLoc++;}
            if (a==1 && b==0) new_state_loc = 1;
        }
        else if (old_state_loc == 2) {
            // Handle A and B inputs for state 2
            if (a==1) {new_state_loc = 3; countLoc--;}
            if (b==0) {new_state_loc = 0; countLoc++;}
            if (a==0 && b==1) new_state_loc = 2;
        }
        else {   // old_state = 3
            // Handle A and B inputs for state 3
            if (a==0) {new_state_loc = 2; countLoc++;}
            if (b==0) {new_state_loc = 1; countLoc--;}
            if (a==1 && b==1) new_state_loc = 3;
        }
        if (new_state_loc != old_state_loc) {
            rotaryChangedLoc = 1;
            old_state_loc = new_state_loc;
        }
        if (countLoc > 400) countLoc = 400;
        else if (countLoc < 1) countLoc = 1;
    } else if (state == REMOTE) { // Handle states for remote
        if (old_state_rem == 0) {
            // Handle A and B inputs for state 0
            if (a==1) {new_state_rem = 1; countRem++;}
            if (b==1) {new_state_rem = 2; countRem--;}
            if (a==0 && b==0) new_state_rem = 0;
        }
        else if (old_state_rem == 1) {
            // Handle A and B inputs for state 1
            if (a==0) {new_state_rem = 0; countRem--;}
            if (b==1) {new_state_rem = 3; countRem++;}
            if (a==1 && b==0) new_state_rem = 1;
        }
        else if (old_state_rem == 2) {
            // Handle A and B inputs for state 2
            if (a==1) {new_state_rem = 3; countRem--;}
            if (b==0) {new_state_rem = 0; countRem++;}
            if (a==0 && b==1) new_state_rem = 2;
        }
        else {   // old_state = 3
            // Handle A and B inputs for state 3
            if (a==0) {new_state_rem = 2; countRem++;}
            if (b==0) {new_state_rem = 1; countRem--;}
            if (a==1 && b==1) new_state_rem = 3;
        }
        if (new_state_rem != old_state_rem) {
            rotaryChangedRem = 1;
            old_state_rem = new_state_rem;
        }
        if (countRem > 400) countRem = 400;
        else if (countRem < 1) countRem = 1;
    }
}

/*
  Timer 1 ISR
*/
ISR(TIMER1_COMPA_vect)
{
    TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10)); // Set prescalar of 0
    isrInvoked = 1;
    started = 0;
    TCNT1 = 0;
}

/*
  Timer 0 ISR
*/
ISR(TIMER0_COMPA_vect)
{
    isrInvokedTimer0++;
    PORTC ^= (1 << PC5);
    if (isrInvokedTimer0 >= isrCount){		
        TCCR0B &= ~(0b100 << CS00);
        isrInvokedTimer0 = 0;
        TCNT0 = 0;
    }
}