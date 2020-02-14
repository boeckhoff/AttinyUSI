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

const bool FIRST_UNIT = false;

volatile bool new_data = false;

volatile bool started_measuring = false;

volatile char measurement[2] = {0};

volatile bool just_received = false;

const uint8_t MESSAGE_LENGTH = 5;
char message[MESSAGE_LENGTH] = {0};
volatile uint8_t index = 0;

uint8_t my_id = 255;
volatile bool next_wrapup = false;

volatile char USISR_buffer;
volatile char USIDR_buffer;

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
    //TIMSK = _BV(OCIE1A);        //interrupt on Compare Match A
    //start timer, ctc mode, prescaler clk/16384   
    TCCR1 = _BV(CTC1) | _BV(CS10);

    // TODO count overflows instead of compare matches
    //TIFR = 1 << OCF0A;     // Clear output compare interrupt flag
}

uint8_t USISerial::send(uint8_t nbytes, char *buffer, long gap) {
    switch(state) {
        case GAP:
            return 0;
        case RECEIVING: 
            return 1;
        case SENDING_START:
        case SENDING_MIDDLE:
        case SENDING_END:
        case SENDING_WRAPUP:
            return 2;
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

    return 3;
}

void USISerial::on_USI_overflow() {

    switch(state) {

    case SENDING_START:
        break;

    case SENDING_MIDDLE:
        USIDR = USIDR_buffer;
        USISR = USISR_buffer;
        if(next_wrapup) {
            state = SENDING_WRAPUP;
            break;
        }

        if(bits_left_to_send) {
            // TODO maybe this is takin too long (the bit shifting)

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
    // enable timer1 interrupt to start measuing byte time
    if(just_received && !FIRST_UNIT) {
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
    OCR0A = TIMER_TICKS;   // Delay to the middle of start bit accounting for interrupt startup and code execution delay before timer start
    TCNT0 = 0;             // Count up from 0
    TIFR = 1 << OCF0A;     // Clear output compare interrupt flag
    TIMSK |= 1 << OCIE0A;  // Enable output compare interrupt
}

void USISerial::on_timer_comp() {
    // COMPA interrupt indicates middle of bit 0

    // TODO: determine if we need to check if we are in state receiving
    TIMSK &= ~(1 << OCIE0A); // Disable COMPA interrupt

    if(s->state != RECEIVING) {
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
    // CAUTION: is called in interrupt routine, process byte directly or store in buffer
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
    // Tweak clock speed for 5V, comment out if running ATtiny at 3V
    OSCCAL += 3;

    pinMode(1,HIGH);                // Configure USI_DO as output.
    pinMode(PB4, OUTPUT);
    pinMode(PB3, OUTPUT);
    digitalWrite(1,HIGH);           // Ensure serial output is high when idle
    digitalWrite(PB4,LOW);           // Ensure serial output is high when idle
    delay(1000);

    s = new USISerial(4,3, 1, &receive_handler);
    char b[13] = "U23456789112";
    /*
    b[0] = 255;
    b[1] = 0xC0;
    b[2] = 255;
    b[3] = 0xC0;
    */
    //s->send(8, b, 1);
}

void loop() {
    /*
    if(new_data && !send) {
        send = true;
        new_data = false;
        digitalWrite(PB4, HIGH);
        delay(100);
        digitalWrite(PB4, LOW);
        delay(100);
        //s->send(1, &received_byte, 1);
        s->send(2, measurement, 1);
    }
    */
    if(new_data) {
        new_data = false;
        switch(message[0]) {
            case 'R':
                my_id = message[1];
                message[1] += 1;
                break;
            case 'T':
                if(message[1] == my_id) {
                    message[2] = measurement[0];
                    message[3] = measurement[1];
                }
                break;
            case 'L':
                if(message[1] == my_id) {
                    PORTB ^= (1 << PB3);  // toggle PB3 for debug
                }
        }
        s->send(MESSAGE_LENGTH, message, 1);
    }
}

ISR(USI_OVF_vect)
{
    //PORTB ^= (1 << PB3);  // toggle PB3 for debug
    USISR = 1 << USIOIF; // clear interrupt flag
    s->on_USI_overflow();
}

ISR(PCINT0_vect)
{
    uint8_t pinbVal = PINB;      // Read directly as Arduino digitalRead is too slow
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
    measurement[0] += 1;
    //PORTB ^= (1 << PB3);  // toggle PB3 for debug
}

#endif