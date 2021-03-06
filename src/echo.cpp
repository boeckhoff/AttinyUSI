/* 
basic USI Serial echo implementation with fifo buffer based on
https://github.com/MarkOsborne/becomingmaker/USISerialSend  (send)
https://github.com/MarkOsborne/becomingmaker/USISerial      (receive)
aswell as Atmel app note AVR307
http://www.atmel.com/Images/doc4300.pdf
and a template fifo buffer implementation by digikey
https://www.digikey.com/eewiki/display/microcontroller/Software+FIFO+Buffer+for+UART+Communication
*/

#ifdef __INTELLISENSE__
#define ECHO
#endif

#ifdef ECHO

/*
ATTiny85 Hookup

RESET -|1 v 8|- Vcc
    PB3 -|2   7|- PB2/SCK
    PB4 -|3   6|- PB1/MISO
    GND -|4 _ 5|- PB0/MOSI/SDA

ATTiny85 PB0/MOSI/SDA -> Serial UART Rx, connect to Tx of serial input device
ATTiny85 PB1/MISO/DO = Serial UART Tx -> connect to Rx of serial output device
*/

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Arduino.h"
#include <sw_fifo.h>

/* Supported combinations:
*   F_CPU 1000000   BAUDRATE 1200, 2400 
*   F_CPU 8000000   BAUDRATE 9600, 19200
*   F_CPU 16000000  BAUDRATE 9600, 19200, 28800, 38400
*/

// Set your baud rate and here
#define BAUDRATE 9600
#define STOPBITS 1
// F_CPU defined by Arduino, e.g. 1000000, 8000000, 16000000

// If bit width in cpu cycles is greater than 255 then  divide by 8 to fit in timer
// Calculate prescaler setting
#define CYCLES_PER_BIT (F_CPU / BAUDRATE)
#if (CYCLES_PER_BIT > 255)
#define DIVISOR 8
#define CLOCKSELECT 2
#else
#define DIVISOR 1
#define CLOCKSELECT 1
#endif
#define FULL_BIT_TICKS (CYCLES_PER_BIT / DIVISOR)
#define HALF_BIT_TICKS (FULL_BIT_TICKS / 2)

// Number of code CPU cycles from from pin change to starting USI timer
#define START_DELAY (99)

// Number of CPU cycles delay after setting COMPA timer until global interrupt is enabled
#define COMPA_DELAY 42
#define TIMER_MIN (COMPA_DELAY / DIVISOR)

#define TIMER_START_DELAY (START_DELAY / DIVISOR)
#if (HALF_BIT_TICKS - TIMER_START_DELAY) > 0
#define TIMER_TICKS (HALF_BIT_TICKS - TIMER_START_DELAY)
#if (TIMER_TICKS < TIMER_MIN)
#warning TIMER_TICKS too low, USI bit sample will after center of bit
#endif
#else
#error "TIMER_TICKS invalid, choose different values for F_CPU, BAUDRATE and START_DELAY"
#define TIMER_TICKS 1
#endif

volatile uint8_t uart_rx_fifo_not_empty_flag = 0;
volatile uint8_t uart_rx_fifo_full_flag      = 0;
volatile uint8_t uart_rx_fifo_ovf_flag       = 0; 
volatile uint8_t uart_tx_fifo_full_flag      = 0;
volatile uint8_t uart_tx_fifo_ovf_flag       = 0;
volatile uint8_t uart_tx_fifo_not_empty_flag = 0;

extern sw_fifo_typedef rx_fifo;
extern sw_fifo_typedef tx_fifo;

// Old timer values
#ifdef ARDUINO
volatile static uint8_t oldTCCR0A;
volatile static uint8_t oldTCCR0B;
volatile static uint8_t oldTCNT0;
#endif

// Serial
volatile bool serialDataReady = false;
volatile uint8_t serialInput;

volatile bool receiving = false;

bool prevButVal = HIGH;

//---- Send -----
// USISerial send state variable and accessors
enum USISERIAL_SEND_STATE
{
    AVAILABLE,
    FIRST,
    SECOND
};
static volatile enum USISERIAL_SEND_STATE usiserial_send_state = AVAILABLE;
static inline enum USISERIAL_SEND_STATE usiserial_send_get_state(void)
{
    return usiserial_send_state;
}
static inline void usiserial_send_set_state(enum USISERIAL_SEND_STATE state)
{
    usiserial_send_state = state;
}
bool usiserial_send_available()
{
    return usiserial_send_get_state() == AVAILABLE;
}

// Transmit data persistent between USI OVF interrupts
static volatile uint8_t usiserial_tx_data;
static inline uint8_t usiserial_get_tx_data(void)
{
    return usiserial_tx_data;
}
static inline void usiserial_set_tx_data(uint8_t tx_data)
{
    usiserial_tx_data = tx_data;
}

static uint8_t reverse_byte(uint8_t x)
{
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;
}

void usiserial_send_byte(uint8_t data)
{
    while (usiserial_send_get_state() != AVAILABLE)
    {
        // Spin until we finish sending previous packet
    };
    usiserial_send_set_state(FIRST);
    usiserial_set_tx_data(reverse_byte(data));

    // Save current Arduino timer state
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
    USIDR = 0x00 |                                         // Start bit (low)
            usiserial_get_tx_data() >> 1;                  // followed by first 7 bits of serial data
    USICR = (1 << USIOIE) |                                // Enable USI Counter OVF interrupt.
            (0 << USIWM1) | (1 << USIWM0) |                // Select three wire mode to ensure USI written to PB1
            (0 << USICS1) | (1 << USICS0) | (0 << USICLK); // Select Timer0 Compare match as USI Clock source.
    DDRB |= (1 << PB1);                                    // Configure USI_DO as output.
    USISR = 1 << USIOIF |                                  // Clear USI overflow interrupt flag
            (16 - 8);                                      // and set USI counter to count 8 bits
}

// Call from main loop to read from serial
// returns true if data was read
// data placed in variable at pData address
bool readSerialData(uint8_t *pData)
{
    if (serialDataReady)
    {
        *pData = serialInput;
        serialDataReady = false;
        return true;
    }
    return false;
}

// Initialize USI for UART reception.
void initializeUSI()
{
    oldTCCR0B = TCCR0B;
    oldTCCR0A = TCCR0A;
    DDRB &= ~(1 << DDB0); // Set pin 0 to input
    PORTB |= 1 << PB0;    // Enable internal pull-up on pin PB0
    USICR = 0;            // Disable USI. GIFR = 1 << PCIF;     // Clear pin change interrupt flag.
    GIMSK |= 1 << PCIE;   // Enable pin change interrupts
    PCMSK |= 1 << PCINT0; // Enable pin change on pin PB0
}

void onSerialPinChange()
{
    // Prepare for receiving a byte
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

// Will fire for all enabled pin change interrupt pins
ISR(PCINT0_vect)
{
    uint8_t pinbVal = PINB;      // Read directly as Arduino digitalRead is too slow
    if (!(pinbVal & 1 << PINB0)) // Trigger only if DI is Low
    {
        receiving = true;
        //digitalWrite(PB2, receiving);
        onSerialPinChange();
    }
}

ISR(TIMER0_COMPA_vect)
{
    if(!receiving) {
        return;
    }
    // COMPA interrupt indicates middle of bit 0
    TIMSK &= ~(1 << OCIE0A); // Disable COMPA interrupt
    TCNT0 = 0;               // Count up from 0
    OCR0A = FULL_BIT_TICKS;  // Shift every bit width
    // Enable USI OVF interrupt, and select Timer0 compare match as USI Clock source:
    USICR = 1 << USIOIE | 0 << USIWM0 | 1 << USICS0;
    // Clear Start condition interrupt flag, USI OVF flag, and set counter
    USISR = 1 << USIOIF | /*1<<USISIF |*/ 8;
}

void serialReceived(uint8_t data)
{
    serialDataReady = true;
    //ticksUnitlClearReceive = micros() + (float)(FULL_BIT_TICKS/(F_CPU/1000000);
    receiving = false;
    //digitalWrite(PB2, receiving);
    uart_byte_received(data);
}

// Reverse USI byte
uint8_t ReverseByte(uint8_t x)
{
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;
}

// USI overflow interrupt indicates we've received or sent a buffer
ISR(USI_OVF_vect)
{
    if (receiving)
    {
        //digitalWrite(PB2, receiving);
        uint8_t temp = USIBR;
        USICR = 0; // Disable USI

        //Restore old timer values
        TCCR0A = oldTCCR0A;
        TCCR0B = oldTCCR0B;
        // Note Arduino millis() and micros() will loose the time it took us to receive a byte
        // Approximately 1ms at 9600 baud
        TCNT0 = oldTCNT0;

        serialReceived(ReverseByte(temp));

        GIFR = 1 << PCIF;   // Clear pin change interrupt flag.
        GIMSK |= 1 << PCIE; // Enable pin change interrupts again
        // We are still in the middle of bit 7 and if it is low we will get a pin change event
        // for the stop bit, but we will ignore it because it is high
    }
    if (!receiving)
    // we have sent a buffer (1 start bit and 7 data bits)
    {
        if (usiserial_send_get_state() == FIRST)
        {
            usiserial_send_set_state(SECOND);
            USIDR = usiserial_get_tx_data() << 7 // Send last 1 bit of data
                    | 0x7F;                      // and stop bits (high)
            USISR = 1 << USIOIF |                // Clear USI overflow interrupt flag
                    (16 - (1 + (STOPBITS)));     // Set USI counter to send last data bit and stop bits
        }
        else
        {
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

            usiserial_send_set_state(AVAILABLE);
        }
    }
}

const uint8_t ledPin = 4;

void setup()
{
    // Tweak clock speed for 5V, comment out if running ATtiny at 3V
    OSCCAL += 3;

    initializeUSI();

    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, 0);

    pinMode(PB2, OUTPUT);

    // Send
    pinMode(1, HIGH);      // Configure USI_DO as output.
    digitalWrite(1, HIGH); // Ensure serial output is high when idle
    pinMode(PB2, INPUT_PULLUP);
    pinMode(PB4, OUTPUT);
}

void loop()
{
    if(digitalRead(PB2) == LOW) {

        if(prevButVal == HIGH) {
            for(uint16_t i = rx_fifo.i_first; i<rx_fifo.i_last; ++i) {
                analogWrite(PB4, rx_fifo.data_buf[i]);
                while (!usiserial_send_available())
                {
                    // Wait for last send to complete
                }
                usiserial_send_byte(rx_fifo.data_buf[i]);
            }
            rx_fifo.num_bytes = 0;
            rx_fifo.i_last = 0;
        }
        prevButVal = LOW;
    }
    else {
        prevButVal = HIGH;
    }
}
#endif