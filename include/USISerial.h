#include <stdint.h>

#define TIMER_CYCLES_TO_USI_OVERFLOW 56

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

enum USISERIAL_STATE {
    READY,
    RECEIVING,
    SENDING_START,
    SENDING_MIDDLE,
    SENDING_END,
    SENDING_WRAPUP,
    GAP = 6,
};

class USISerial {

    private:
        uint8_t num_rbytes;
        uint8_t num_sbytes;
        long rgap;
        void (*receive_handler)();

        volatile uint8_t bytes_left_to_send;
        volatile uint8_t bits_left_to_send;
        volatile char *send_buffer;
        volatile uint8_t cur_byte_to_send;


        #ifdef ARDUINO
            volatile uint8_t oldTCCR0A;
            volatile uint8_t oldTCCR0B;
            volatile uint8_t oldTCNT0;

            volatile uint8_t oldTCCR1;
            volatile uint8_t oldTCNT1;
        #endif

    public:
        //TODO make privat
        volatile USISERIAL_STATE state;
        USISerial(uint8_t _num_rbytes, uint8_t _num_sbytes, long _r_gap, void (*_receive_handler)());
        void initialize_USI();
        uint8_t send(uint8_t nbytes, char *buffer, long gap);
        void block(long time);
        void on_USI_overflow();
        void on_start_bit();
        void on_timer_comp();
};