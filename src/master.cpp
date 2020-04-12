
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

#define BAUD_RATE 9600
#define F_CPU 8000000

#define CYCLES_PER_BIT 833


// declare the messages to be send
char messages[7][MESSAGE_LENGTH] = {
    {'R',0,0,0,0}, // register all units with IDs
    {'T',0,0,0,0}, // get byte timing information from unit 0
    {'T',1,0,0,0}, // get byte timing information from unit 0
    {'T',2,0,0,0}, // get byte timing information from unit 0
    {'T',3,0,0,0}, // get byte timing information from unit 0
    {'L',3,0,0,0}, // light up unit 3
    {'L',0,0,0,0}, // light up unit 0
};

uint16_t clock_cycles[4] = {0};

uint8_t num_devices;

uint8_t message_index = 0;
bool new_data = false;

uint8_t prevButtonRead = HIGH;

void setup()
{
    Serial.begin(9600);
    mySerial.begin(9600);

    pinMode(3, INPUT_PULLUP);

    delay(100);
}

char incomingByte;

void loop() {

    if(digitalRead(3) == LOW && prevButtonRead == HIGH) {
        prevButtonRead = LOW;
        if(message_index != 5) {
            mySerial.write(messages[message_index],MESSAGE_LENGTH);
        }
        else { //timed message
            double d = 0.0;

            double prevByteTime = 10.0/(double)BAUD_RATE;
            double curByteTime = 0.0;
            double curByteBelief = 0.0;
            
            for(uint8_t i = 0; i<messages[message_index][1]; ++i) {
                curByteBelief = (double)clock_cycles[i]/(double)F_CPU;
                curByteTime = prevByteTime + (((prevByteTime - curByteBelief)/curByteBelief) * prevByteTime);
                prevByteTime = curByteTime;
                d += curByteTime*(double)MESSAGE_LENGTH;
            }

            d -= (10.0/(double)BAUD_RATE)*MESSAGE_LENGTH;

            mySerial.write(messages[message_index],MESSAGE_LENGTH);
            delay(d*1000.0);
            mySerial.write(messages[message_index+1],MESSAGE_LENGTH);


            /*
            for(uint8_t i = 0; i<messages[message_index][1]; ++i) {
                Serial.print("clock cycles: ");
                d+= (10.0/9600.0)*(8000.0/(float)clock_cycles[i])*MESSAGE_LENGTH;
                Serial.print(clock_cycles[i]);
            }
            d -= (10.0/9600.0)*MESSAGE_LENGTH;
            */

        }

        message_index++;
        delay(300);
    }
    else {
        prevButtonRead = HIGH;
    }

    if(mySerial.available()) {
        // read the incoming byte:
        message[0] = mySerial.read();
        message[1] = mySerial.read();
        message[2] = mySerial.read();
        message[3] = mySerial.read();
        message[4] = mySerial.read();
        new_data = true;
    }

    if(new_data) {
        new_data = false;
        Serial.print((uint8_t)message[0]);
        Serial.print(" ");
        Serial.print((uint8_t)message[1]);
        Serial.print(" ");
        Serial.print((uint8_t)message[2]);
        Serial.print(" ");
        Serial.print((uint8_t)message[3]);
        Serial.print(" ");
        Serial.print((uint8_t)message[4]);
        Serial.println(" ");
        if(message[0] == 82) {
            num_devices = message[1];
        }
        if(message[0] == 84) {
            clock_cycles[message[1]] = message[2]*243+message[3];
            Serial.print((uint8_t)message[1]);
            Serial.print(" ");
            Serial.println(clock_cycles[message[1]]);
        }

    }
}

#endif