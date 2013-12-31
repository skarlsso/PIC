#ifndef LED_DEBUG_H
#define LED_DEBUG_H

#include "xc.h"

// FIXME: Hard coded for now.
#define MICROSTICK2

#ifdef MICROSTICK2
#define DEBUG_BIT_0 LATAbits.LATA0
#define DEBUG_BIT_1 LATAbits.LATA0
#else
#define DEBUG_BIT_0 LATCbits.LATC13
#define DEBUG_BIT_1 LATCbits.LATC14
#endif

inline static void debug_init(void) {
    // Set to output pins.
#ifdef MICROSTICK2
    TRISAbits.TRISA0 = 0;
#else
    TRISCbits.TRISC13 = 0;
    TRISCbits.TRISC14 = 0;
#endif
}

// Hang and blink a led.
void debug_blink(char bit_num, unsigned long delay_on_ms, unsigned long delay_off_ms);

#endif // LED_DEBUG_H