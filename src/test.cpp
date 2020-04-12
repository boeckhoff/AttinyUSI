/* test script to tune frequency*/

#ifdef __INTELLISENSE__
#define TEST
#endif

#ifdef TEST

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Arduino.h"

void setup() {
    pinMode(PB3, OUTPUT)
    OSCCAL -= 5;                // User calibration
    TCCR1 = 0;
    TCCR1 |= (1 << CTC1);  // clear timer on compare match
    TCCR1 |= (1 << CS12) | (1 << CS11) | (1 << CS10); //clock prescaler 8192
    OCR1C = 10; // compare match value 
    TIMSK |= (1 << OCIE1A); // enable compare match interrupt
}

ISR(TIMER1_COMPA_vect)
{
    PORTB ^= (1 << PB3);  // toggle PB3 for debug
}


void loop() {
}

#endif TEST