/*  USI UART daisy-chain communication for ATtiny85 */

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
char received_byte; 
uint8_t byte_time;

static uint8_t reverse_byte(uint8_t x)
{
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;
}

USISerial::USISerial(uint8_t _num_rbytes, uint8_t _num_sbytes, uint8_t _rgap, void (*_receive_handler)()) { 
    num_rbytes = _num_rbytes;
    num_sbytes = _num_sbytes;
    rgap = (_rgap*1024)/CYCLES_PER_BIT; // user gives rgap in BIT_LENGTH time,
                                        // we use this value to set an interrupt
                                        // for a timer1 with 1024 prescaler
    receive_handler = _receive_handler;
    state = READY;
    initialize_USI();
}

// Initialize USI for UART reception.
void USISerial::initialize_USI()
{
    oldTCCR0B = TCCR0B;
    oldTCCR0A = TCCR0A;
    DDRB &= ~(1 << DDB0); // Set pin 0 to input
    PORTB |= 1 << PB0;    // Enable internal pull-up on pin PB0
    USICR = 0;            // Disable USI. GIFR = 1 << PCIF;     // Clear pin change interrupt flag.
    GIMSK |= 1 << PCIE;   // Enable pin change interrupts
    PCMSK |= 1 << PCINT0; // Enable pin change on pin PB0

    // initialize timer1 for measuing byte-receive time
    // TODO move this to when its actually needed + save & restore old state
    oldTCCR1 = TCCR1;
    oldTCNT1 = TCNT1;

    TCCR1 = 0;                  //stop the timer
    TCNT1 = 0;                  //zero the timer
    GTCCR = _BV(PSR1);          //reset the prescaler
    OCR1A = 243;                //set the compare value
    OCR1C = 243;
    TCCR1 = _BV(CTC1) | _BV(CS10);

    // TODO count overflows instead of compare matches
}

uint8_t USISerial::send(uint8_t nbytes, char *buffer, long gap) {
    switch(state) {
        case GAP:
            return 1; //sending not allowed due to user-defined gap
        case RECEIVING: 
            return 2;
        case SENDING_START:
        case SENDING_MIDDLE:
        case SENDING_END:
        case SENDING_WRAPUP:
            return 3;
    }

    // we have verified that send state is READY
    send_buffer = buffer;
    bytes_left_to_send = nbytes;
    state = SENDING_START;

    #ifdef ARDUINO
        oldTCCR0B = TCCR0B;
        oldTCCR0A = TCCR0A;
        oldTCNT0 = TCNT0;
    #endif

    // Configure Timer0
    TCCR0A = 2 << WGM00;    // CTC mode
    TCCR0B = CLOCKSELECT;   // Set prescaler to clk or clk /8
    GTCCR |= 1 << PSR0;     // Reset prescaler
    OCR0A = FULL_BIT_TICKS; // Trigger every full bit width
    TCNT0 = 0;              // Count up from 0
    
    // Configure timer1 to fix spike
    oldTCCR1 = TCCR1;
    oldTCNT1 = TCNT1;

    steps = 0;
    TCCR1 = 0;                  //stop the timer
    TCNT1 = 0;                  //zero the timer
    GTCCR = _BV(PSR1);          //reset the prescaler
    OCR1A = FULL_BIT_TICKS;                //set the compare value
    OCR1C = FULL_BIT_TICKS;
    //TIMSK = _BV(OCIE1A);        //interrupt on Compare Match A
    //start timer, ctc mode, prescaler clk/16384   
    TCCR1 = _BV(CTC1) | _BV(CS10);
    TIMSK |= (1 << OCIE1A); // enable compare match interrupt

    // Configure USI to send low start bit and 7 bits of data
    USIDR_buffer = 0x00 |                                         // Start bit (low)
            reverse_byte(*send_buffer) >> 1;                              // followed by first 7 bits of serial data
    USICR = (1 << USIOIE) |                                // Enable USI Counter OVF interrupt.
            (0 << USIWM1) | (1 << USIWM0) |                // Select three wire mode to ensure USI written to PB1
            (0 << USICS1) | (1 << USICS0) | (0 << USICLK); // Select Timer0 Compare match as USI Clock source.
    DDRB |= (1 << PB1);                                    // Configure USI_DO as output.
    USISR_buffer = 1 << USIOIF |                                  // Clear USI overflow interrupt flag
            (16 - 8);                                      // and set USI counter to count 8 bits

    bits_left_to_send = 1;

    state = SENDING_MIDDLE;
    on_USI_overflow();

    return 0; //return 0 for sending success
}

void USISerial::on_USI_overflow() {

    switch(state) {

    case SENDING_START:
        break;

    case SENDING_MIDDLE:
        USIDR = USIDR_buffer;
        USISR = USISR_buffer;

        last_bit = USIDR_buffer & 0x01;
        USICR |= (1 << USIWM0); // Select three wire mode to ensure USI written to PB1

        steps = 0;
        TIMSK |= (1 << OCIE1A); // enable compare match interrupt
        //PORTB ^= (1 << PB3);  // toggle PB3 for debug

        if(next_wrapup) {
            state = SENDING_WRAPUP;
            break;
        }

        if(bits_left_to_send) {
            USIDR_buffer = reverse_byte(*send_buffer) << (8-bits_left_to_send) // send leftover bits
                      | (1 << (7-bits_left_to_send));               // send stop bit(high)

            bytes_left_to_send -= 1;

            if(bytes_left_to_send) {
                send_buffer++;
                USIDR_buffer |= (reverse_byte(*send_buffer) >> (bits_left_to_send + 2)); // send start bit and first n bits

                if(bits_left_to_send == 7) {
                    bits_left_to_send = 0;
                }
                else {
                    bits_left_to_send += 2;
                }
                USISR_buffer |= (16 - 8);           // set USI counter to count 8 bits
            }
            else {
                USIDR_buffer |= (1 << (6-bits_left_to_send));
                //USIDR |= (255 >> (bits_left_to_send));               // send stop bit(high)
                USISR_buffer |= (16 - (bits_left_to_send + 1)); // set USI counter to count leftover bits + stopbit
                bits_left_to_send = 0;
                next_wrapup = true;
            }
        }
        else {
            USIDR_buffer = 0x00 |                        // Start bit (low)
                reverse_byte(*send_buffer) >> 1;  // followed by first 7 bits of serial data
            USISR_buffer |= (16 - 8);           // set USI counter to count 8 bits
            bits_left_to_send = 1;
        }
        break;

    case SENDING_WRAPUP:
        //reset timer1
        TIMSK &= ~(1 << OCIE1A); // Disable COMPA interrupt

        //restore old timer state
        TCCR1 = oldTCCR1;
        TCNT1 = oldTCNT1;

        PORTB |= 1 << PB1;    // Ensure output is high
        DDRB |= (1 << PB1);   // Configure USI_DO as output.
        USICR = 0;            // Disable USI.
        USISR |= 1 << USIOIF; // clear interrupt flag

        if(rgap != 0) {
            TCCR1 = 0;                  //stop the timer
            TCNT1 = 0;                  //zero the timer
            GTCCR = _BV(PSR1);          //reset the prescaler
            OCR1A = rgap;                //set the compare value
            OCR1C = rgap;

            TIMSK = _BV(OCIE1A);        //interrupt on Compare Match A
            //start timer, ctc mode, prescaler clk/16384   
            TCCR1 = _BV(CTC1) | _BV(CS10) | _BV(CS11) | _BV(CS13); //set prescaler to 1024
        }

        //Restore old timer values for Arduino
        #ifdef ARDUINO
            TCCR0A = oldTCCR0A;
            TCCR0B = oldTCCR0B;
            // Note Arduino millis() and micros() will lose the time it took us to send a byte
            // Approximately 1ms at 9600 baud
            TCNT0 = oldTCNT0;
        #endif
        
        state = READY;
        next_wrapup = false;
        break;

    case RECEIVING:
        received_byte = reverse_byte(USIBR);
        USICR = 0; // Disable USI

        //Restore old timer values
        TCCR0A = oldTCCR0A;
        TCCR0B = oldTCCR0B;
        // Note Arduino millis() and micros() will loose the time it took us to receive a byte
        // Approximately 1ms at 9600 baud
        TCNT0 = oldTCNT0;

        receive_handler();

        GIFR = 1 << PCIF;   // Clear pin change interrupt flag.
        GIMSK |= 1 << PCIE; // Enable pin change interrupts again
        // We are still in the middle of bit 7 and if it is low we will get a pin change event
        // for the stop bit, but we will ignore it because it is high
        state = READY;
        break;
    
    default:
        return;
    }
}

void USISerial::on_start_bit() {
    // enable timer1 interrupt to start measuring byte time
    if(just_received) {
        just_received = false;
        return;
    }
    if(started_measuring) {
        TIMSK &= ~(1 << OCIE1A); // Disable COMPA interrupt

        //restore old timer state
        TCCR1 = oldTCCR1;
        TCNT1 = oldTCNT1;
        measurement[1] = TCNT1;
    }
    else {
        TIMSK |= (1 << OCIE1A); // enable compare match interrupt
        started_measuring = true;
    }

    // Prepare for receiving a byte
    state = RECEIVING;

    oldTCNT0 = TCNT0;      // Save old timer counter
    GIMSK &= ~(1 << PCIE); // Disable pin change interrupts
    TCCR0A = 2 << WGM00;   // CTC mode
    TCCR0B = CLOCKSELECT;  // Set prescaler to clk or clk /8
    GTCCR |= 1 << PSR0;    // Reset prescaler
    OCR0A = TIMER_TICKS;   // Set compare value to middle of start bit
    TCNT0 = 0;             // Count up from 0
    TIFR = 1 << OCF0A;     // Clear output compare interrupt flag
    TIMSK |= 1 << OCIE0A;  // Enable output compare interrupt
}

void USISerial::on_timer_comp() {
    // COMPA interrupt indicates middle of bit 0

    // TODO: determine if we need to check if we are in state receiving
    TIMSK &= ~(1 << OCIE0A); // Disable COMPA interrupt

    if(state != RECEIVING) {
        return;
    }

    byte_time += TCNT0;
    TCNT0 = 0;               // Count up from 0
    OCR0A = FULL_BIT_TICKS;  // Shift every bit width
    // Enable USI OVF interrupt    
    TIMSK &= ~(1 << OCIE0A); // Disable COMPA interrupt, and select Timer0 compare match as USI Clock source:
    USICR = 1 << USIOIE | 0 << USIWM0 | 1 << USICS0;
    // Clear Start condition interrupt flag, USI OVF flag, and set counter
    USISR = 1 << USIOIF | /*1<<USISIF |*/ 8;
}


void receive_handler() {
    // CAUTION: is called in ISR, process byte directly or store in buffer
    // NO DELAYS
    message[index] = received_byte;
    index++;

    if(index == MESSAGE_LENGTH) {
        new_data = true;
        just_received = true;
        index = 0;
    }
}

void setup() {
    pinMode(1,HIGH);         // Configure USI_DO as output.
    pinMode(PB4, OUTPUT);
    pinMode(PB3, OUTPUT);
    digitalWrite(1,HIGH);    // Ensure serial output is high when idle
    digitalWrite(PB4,LOW);   // Ensure serial output is high when idle
    delay(1000);

    s = new USISerial(4,3, 1, &receive_handler);
}

void loop() {
    if(new_data) {
        new_data = false;
        switch(message[0]) {
            case 'R':
                // Register -> save ID and pass on ID+1  
                my_id = message[1];
                message[1] += 1;
                s->send(MESSAGE_LENGTH, message, 1);
                break;
            case 'T':
                // Time -> if ID matches send timing information for byte
                if(message[1] == my_id) { 
                    message[2] = measurement[0];
                    message[3] = measurement[1];
                }
                s->send(MESSAGE_LENGTH, message, 1);
                break;
            case 'L':
                // Light -> if ID matches toggle LED
                if(message[1] == my_id) {
                    PORTB ^= (1 << PB3);
                }
                else {
                    s->send(MESSAGE_LENGTH, message, 1);
                }
                break;
            default:
                // pass on all other messages
                    s->send(MESSAGE_LENGTH, message, 1);
                break;
        }
    }
}

ISR(USI_OVF_vect)
{
    USISR = 1 << USIOIF; // clear interrupt flag
    s->on_USI_overflow();
}

ISR(PCINT0_vect)
{
    uint8_t pinbVal = PINB;
    if (!(pinbVal & 1 << PINB0)) // Trigger only if DI is Low
    {
        s->on_start_bit();
    }
}

ISR(TIMER0_COMPA_vect)
{
    s->on_timer_comp();
}

ISR(TIMER1_COMPA_vect)
{
    if(s->state == RECEIVING) {
        measurement[0] += 1;
        return;
    }
    if(s->state == GAP) {
        s->state = READY;
        return;
    }

    if(steps == TIMER_CYCLES_TO_USI_OVERFLOW) {
        //PORTB ^= (1 << PB3);  // toggle PB3 for debug
        TIMSK &= ~(1 << OCIE1A); // disable compare match interrupt

        if(last_bit) {
            PORTB|= (1 << PB1);  // pull pb0 high
        }
        else {
            PORTB&= ~(1 << PB1);  // pull pb0 low
        }
        USICR&= ~(1 << USIWM1);
        USICR&= ~(1 << USIWM0);
        }
    steps ++;
}

#endif