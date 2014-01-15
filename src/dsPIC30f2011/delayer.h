#ifndef DELAYER_H
#define DELAYER_H

// FIXME: Hard coded for dsPIC30F2011 with _FOSC(CSW_FSCM_OFF & FRC_PLL16)
// FOCS = 7.37 MHZ           // FRC - Fast Internal RC Oscillator 
// PPL = 16                  // 
// FCY = (FOCS * PLL) / 4   // 4 clocks per instruction
#if !(FCY == 29840000)
# warning "This file assumes FCY == 29840000"
#endif

#include <xc.h>

// __delay32() gives unsatble behavior and sets RCONbits.IOPUWR,
// so provide some inaccurate delay functions instead.

// Nanos per instruction: 1 / FCY ~= 34 nanos
// Instructions per micro second: 1000 / 34 ~= 30
inline static void delay_us() {
    Nop(); Nop(); Nop(); Nop(); Nop();
    Nop(); Nop(); Nop(); Nop(); Nop();
    Nop(); Nop(); Nop(); Nop(); Nop();
    Nop(); Nop(); Nop(); Nop(); Nop();
    Nop(); Nop(); Nop(); Nop(); Nop();
    Nop(); Nop(); Nop(); Nop(); Nop();
}

// The offset is used to delay some extra if the accuracy is too bad. 
inline static void delay_ns_with_offset(unsigned long offset, unsigned long nanos) {
    delay_us((offset + nanos) / 1000);
}

inline static void delay_ns(unsigned long nanos) {
    delay_ns_with_offset(0, nanos);
}

inline static void delay_ms(unsigned long millis) {
    unsigned int i;
    for (; millis != 0; --millis) {
        // Just some guesses and some tuning.
        // 6000 ~= 1000000 / 5 * 34
        for (i = 6000; i != 0; i--) {
            // READ F -> W
            // ADDLW 
            // STOR W -> F
            // COMP
            // JMP
        }
    }
}

// 30MHz => 1 instruction 100/3 ns
// 30 * (100/3 ns) = 1000 ns = 1 us
#define delay_us()    asm("repeat #%0" :: "i"(30)); Nop()
#define delay_100us() asm("repeat #%0" :: "i"(3000)); Nop()

#endif // DELAYER_H