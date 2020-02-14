
#ifdef __INTELLISENSE__
#define MASTER
#endif

#ifdef MASTER

#include "Arduino.h"
#include <SoftwareSerial.h>
#include <stdlib.h>

SoftwareSerial mySerial(10, 11); //RX, TX

const uint8_t MESSAGE_LENGTH = 5;
char message[MESSAGE_LENGTH];

char messages[4][MESSAGE_LENGTH] = {
    {'R',0,0,0,0},
    {'T',0,0,0,0},
    {'T',1,0,0,0},
    {'T',2,0,0,0},
};

uint8_t num_devices;

uint8_t message_index = 0;

uint8_t prevButtonRead = HIGH;

void setup()
{
    Serial.begin(9600);
    mySerial.begin(9600);

    pinMode(3, INPUT_PULLUP);

    delay(100);

    message[0] = 'R';
    message[1] = 0;
    message[2] = 0;
    message[3] = 0;
    message[4] = 0;
    /*
    mySerial.write(message, MESSAGE_LENGTH);

    delay(100);

    message[0] = 'T';
    message[1] = 0;
    message[2] = 0;
    message[3] = 0;
    message[4] = 0;
    mySerial.write(message, MESSAGE_LENGTH);
    */
}

void loop() {
    if(digitalRead(3) == LOW && prevButtonRead == HIGH) {
        prevButtonRead = LOW;
        mySerial.write(messages[message_index],MESSAGE_LENGTH);
        message_index++;
        delay(300);
    }
    else {
        prevButtonRead = HIGH;
    }

    /*
    if(Serial.available()) {
        Serial.print((char)Serial.read());
    }
    */
}

#endif