#ifndef LED_DEBUG_H
#define LED_DEBUG_H

#include <xc.h>

#include "time/delayer.h"

#define pulse_forever2(LAT, on_ms, off_ms, clear_watchdog) \
     do {                                                  \
        LAT = 1;                                           \
        delay_ms(on_ms);                                   \
        LAT = 0;                                           \
        delay_ms(off_ms);                                  \
        if (clear_watchdog) {                              \
            ClrWdt();                                      \
        }                                                  \
    } while (1)

#define pulse_forever(LAT, on_ms, off_ms) \
    pulse_forever2(LAT, on_ms, off_ms, 0)


#endif // LED_DEBUG_H