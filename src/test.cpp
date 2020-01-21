#ifdef __INTELLISENSE__
#define TEST
#endif

#ifdef TEST

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Arduino.h"

void setup() {
    pinMode(PB2, OUTPUT);
    //OSCCAL -= 8;                // User calibration
}
 
void loop() {
    digitalWrite(PB2, HIGH);
    delay(1000);
    digitalWrite(PB2, LOW);
    delay(1000);
}

#endif TEST