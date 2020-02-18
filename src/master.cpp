
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

#define CYCLES_PER_BIT 833

char messages[7][MESSAGE_LENGTH] = {
    {'R',0,0,0,0},
    {'T',0,0,0,0},
    {'T',1,0,0,0},
    {'T',2,0,0,0},
    {'T',3,0,0,0},
    {'L',3,0,0,0},
    {'L',0,0,0,0},
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

char incomingByte;

void loop() {

    if(digitalRead(3) == LOW && prevButtonRead == HIGH) {
        prevButtonRead = LOW;
        if(message_index != 5) {
            mySerial.write(messages[message_index],MESSAGE_LENGTH);
        }
        else { //timed message
            Serial.println("timed message");
            double d = 0.0;

            double prevByteTime = 10.0/9600.0;
            double curByteTime = 0.0;
            double curByteBelief = 0.0;
            
            for(uint8_t i = 0; i<messages[message_index][1]; ++i) {
                Serial.println("i");

                Serial.println("clockCycles");
                Serial.println((float)clock_cycles[i]);
                curByteBelief = (double)clock_cycles[i]/8000000.0;
                Serial.println("curByteBelief");
                Serial.println(curByteBelief, 8);

                Serial.println("ratio");
                Serial.println(((prevByteTime - curByteBelief)/curByteBelief), 8);

                curByteTime = prevByteTime + (((prevByteTime - curByteBelief)/curByteBelief) * prevByteTime);

                Serial.println("curByteTime");
                Serial.println(curByteTime, 8);
                
                prevByteTime = curByteTime;
                Serial.println("adding:");
                Serial.println(curByteTime*(double)MESSAGE_LENGTH, 8);

                Serial.println("cur d:");
                Serial.println(d, 8);
                d += curByteTime*(double)MESSAGE_LENGTH;
            }

            d -= (10.0/9600.0)*MESSAGE_LENGTH;

            Serial.print("delay ");
            Serial.println(d*1000.0);

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
            clock_cycles[message[1]] = 8300;//message[2]*243+message[3];
            Serial.print((uint8_t)message[1]);
            Serial.print(" ");
            Serial.println(clock_cycles[message[1]]);
        }

    }
}

#endif