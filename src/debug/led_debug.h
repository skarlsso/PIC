#ifndef LED_DEBUG_H
#define LED_DEBUG_H

// FIXME: Hard coded for now.
#include <p30F2011.h>
#define DEBUG_BIT_0 LATCbits.LATC13
#define DEBUG_BIT_1 LATCbits.LATC15

inline static void debug_init(void) {
    // Set to output pins.
    TRISCbits.TRISC13 = 0;
    TRISCbits.TRISC15 = 0;
}

// Hang and blink a led.
void debug_blink(char bit_num, unsigned long delay_on_ms, unsigned long delay_off_ms);

#endif // LED_DEBUG_H