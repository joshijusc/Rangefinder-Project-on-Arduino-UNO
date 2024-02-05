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
#include "lcd.h"
#include <avr/eeprom.h>
#include "timers.h"

void timer0_init()
{
    // Set to CTC mode
    TCCR0A |= (1 << WGM01);

    // Enable Timer Interrupt
    TIMSK0 |= (1 << OCIE0A);
    TCNT0 = 1;
}

void timer1_init()
{
    TCCR1B |= (1 << WGM12);
    OCR1A = 46400; // with prescalar 8

    // Enable Timer Interrupt
    TIMSK1 |= (1 << OCIE1A);
}

void timer2_init(void)
{
    TCCR2A |= (0b11 << WGM20);  // Fast PWM mode, modulus = 256
    TCCR2A |= (0b10 << COM2A0); // Turn D11 on at 0x00 and off at OCR2A
    OCR2A = 35;                // Initial pulse duty cycle of 50%
    TCCR2B |= (0b111 << CS20);  // Prescaler = 1024 for 16ms period
}