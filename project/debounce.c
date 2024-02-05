/********************************************
 *
 *  Name: Jason Joshi
 *  Email: joshij@usc.edu
 *  Section: 11am Friday
 *  Assignment: Project - Rangefinder
 *
 ********************************************/
#include "debounce.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <stdlib.h>     /* abs */
#include "lcd.h"
#include <avr/eeprom.h>

// Debounces bit on Port B
void debounceB(size_t bit)
{
    // Add code to debounce input "bit" of PINC
    // assuming we have sensed the start of a press.
    char pressed = (PINB & (1 << bit));
    if(pressed == 0){
        _delay_ms(5);
        while((PINB & (1 << bit)) == 0) { }
        _delay_ms(5);
    }
}

// Debounces bit on Port C
void debounceC(size_t bit)
{
    // Add code to debounce input "bit" of PINC
    // assuming we have sensed the start of a press.
    char pressed = (PINC & (1 << bit));
    if(pressed == 0){
        _delay_ms(5);
        while((PINC & (1 << bit)) == 0) { }
        _delay_ms(5);
    }
}