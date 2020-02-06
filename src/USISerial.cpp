#ifdef __INTELLISENSE__
#define USISERIAL
#endif

#ifdef USISERIAL

#include <USISerial.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Arduino.h"
#include <sw_fifo.h>

USISerial* s;

static uint8_t reverse_byte(uint8_t x)
{
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;
}

USISerial::USISerial(uint8_t _num_rbytes, uint8_t _num_sbytes, long _rgap, void (*_receive_handler)()) { 
    num_rbytes = _num_rbytes;
    num_sbytes = _num_sbytes;
    rgap = _rgap;
    receive_handler = _receive_handler;
    state = READY;
}

uint8_t USISerial::send(uint8_t nbytes, char *buffer, long gap) {
    switch(state) {
        case GAP:
            return 0;
            break;
        case RECEIVING:
            break;
            return 1;
        case SENDING_START:
        case SENDING_MIDDLE:
        case SENDING_END:
        case SENDING_WRAPUP:
            return 2;
            break;
    }
    // we have verified that send state is READY
    send_buffer = buffer;
    bytes_left_to_send = nbytes;
    state = SENDING_START;
    check_send();
    return 3;
}

void USISerial::check_send() {
    //is triggered by send() or interrupt that indicates we have send a buffer

    switch(state) {

    case SENDING_START:

        #ifdef ARDUINO
            oldTCCR0B = TCCR0B;
            oldTCCR0A = TCCR0A;
            oldTCNT0 = TCNT0;
        #endif

        // Configure Timer0
        TCCR0A = 2<<WGM00;                      // CTC mode
        TCCR0B = CLOCKSELECT;                   // Set prescaler to clk or clk /8
        GTCCR |= 1 << PSR0;                     // Reset prescaler
        OCR0A = FULL_BIT_TICKS;                 // Trigger every full bit width
        TCNT0 = 0;                              // Count up from 0 

        // Configure USI to send low start bit and 7 bits of data

        USIDR = 0x00 |                                         // Start bit (low)
                reverse_byte(*send_buffer) >> 1;               // followed by first 7 bits of serial data
        USICR = (1 << USIOIE) |                                // Enable USI Counter OVF interrupt.
                (0 << USIWM1) | (1 << USIWM0) |                // Select three wire mode to ensure USI written to PB1
                (0 << USICS1) | (1 << USICS0) | (0 << USICLK); // Select Timer0 Compare match as USI Clock source.
        DDRB |= (1 << PB1);                                    // Configure USI_DO as output.
        USISR = 1 << USIOIF |                                  // Clear USI overflow interrupt flag
                (16 - 8);                                      // and set USI counter to count 8 bits

        bytes_left_to_send -= 1;

        if(bytes_left_to_send) {
            state = SENDING_MIDDLE;
        }
        else {
            state = SENDING_END;
        }
        break;

    if(bits_left) {
        USIDR = reverse_byte(*send_buffer) << (8-bits_left);
        USIDR |= (1 >> (6 - bits_left));
        if(bytes_left_to_send) {
            send_buffer++;
            bytes_left_to_send -= 1;
            USIDR |= reverse_byte(*send_buffer) >> (bits_left+2);
        }
    }
    else {
    }
    
    case SENDING_MIDDLE:
        // USIDR: [last bit][stop bit (high)][start bit(low)][first 5 bits of next byte]
        USIDR = reverse_byte(*send_buffer) << 7; // last bit of previous byte
        send_buffer++;
        bytes_left_to_send -=1;
        USIDR |= 0x40; // followed by stop bit (high)
        USIDR |= reverse_byte(*send_buffer) >> 3; // followed by first 5 bits of next byte
        USISR = 1 << USIOIF |       // Clear USI overflow interrupt flag
                (16 - 8);           // and set USI counter to count 8 bits

        if(!bytes_left_to_send) {
            state = SENDING_END;
        }
        break;

    case SENDING_:
        // USIDR: [last 3 bits][stop bit (high)][start bit(low)][first 5 bits of next byte]


    case SENDING_END:    
        USIDR = reverse_byte(*send_buffer) << 5 // Send last 3 bits of data
                | 0x7F;                      // and stop bits (high)
        USISR = 1 << USIOIF |                // Clear USI overflow interrupt flag
                (16 - (1 + (STOPBITS)));     // Set USI counter to send last data bit and stop bits
        
        state = SENDING_WRAPUP;
        break;

    case SENDING_WRAPUP:
        PORTB |= 1 << PB1;    // Ensure output is high
        DDRB |= (1 << PB1);   // Configure USI_DO as output.
        USICR = 0;            // Disable USI.
        USISR |= 1 << USIOIF; // clear interrupt flag

        //Restore old timer values for Arduino
        #ifdef ARDUINO
            TCCR0A = oldTCCR0A;
            TCCR0B = oldTCCR0B;
            // Note Arduino millis() and micros() will lose the time it took us to send a byte
            // Approximately 1ms at 9600 baud
            TCNT0 = oldTCNT0;
        #endif

        state = READY;
        break;
    
    default:
        return;
    }
}

void receive_handler() {

}

void setup() {
    pinMode(1,HIGH);                // Configure USI_DO as output.
    digitalWrite(1,HIGH);           // Ensure serial output is high when idle
    delay(1000);

    s = new USISerial(3,3, 1, &receive_handler);
    char b[13] = "UUU456789112";
    s->send(2, b, 1);
}

void loop() {

}

ISR(USI_OVF_vect)
{
    s->check_send();
}

#endif