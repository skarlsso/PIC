#ifndef WS2812B_H
#define	WS2812B_H

// Each bit is encoded as:
//  0: High(0.4 us +- 150 ns) then Low(0.85 us +- 150ns)
//  1: High(0.8 us +- 150 ns) then Low(0.45 us +- 150ns)
//
// The timing is very sensitive.
// The Nop() lengths were timed with the help of an oscilloscope.

// FIXME: Use different config depending on generated code, device and frequency.

#if 0
// main_led_ws2812b.c config
#define HIGH_LONG()  Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop()
#define HIGH_SHORT() Nop(); Nop(); Nop()
#define LOW_LONG()   Nop(); Nop(); Nop(); Nop(); Nop(); Nop()
#define LOW_SHORT()
#endif

#define Nop5() Nop(); Nop(); Nop(); Nop(); Nop()
#define Nop10() Nop5(); Nop5()

#define HIGH_LONG()  Nop10(); Nop5(); Nop(); Nop(); Nop(); Nop();
#define HIGH_SHORT() Nop5(); Nop(); Nop()
#define LOW_LONG()   Nop10(); Nop5(); Nop(); Nop(); Nop()
#define LOW_SHORT()  Nop5()

#define init_ws2812b(PIN) \
    PIN = 0 /* Start state */

#define SEND_AND_SHIFT_BIT(PIN, value) \
    PIN = 1;                           \
    if (value & 0b10000000) {          \
        HIGH_LONG();                   \
    } else {                           \
        HIGH_SHORT();                  \
    }                                  \
    PIN = 0;                           \
    if (value & 0b10000000) {          \
        LOW_SHORT();                   \
    } else {                           \
        LOW_LONG();                    \
    }                                  \
    value <<= 1;

#define ws2812b_send_byte(PIN, value_variable) \
    SEND_AND_SHIFT_BIT(PIN, value_variable)    \
    SEND_AND_SHIFT_BIT(PIN, value_variable)    \
    SEND_AND_SHIFT_BIT(PIN, value_variable)    \
    SEND_AND_SHIFT_BIT(PIN, value_variable)    \
    SEND_AND_SHIFT_BIT(PIN, value_variable)    \
    SEND_AND_SHIFT_BIT(PIN, value_variable)    \
    SEND_AND_SHIFT_BIT(PIN, value_variable)    \
    SEND_AND_SHIFT_BIT(PIN, value_variable)

// TODO: Check the ASM to see if the do/while adds overhead.
#define ws2812b_send_rgb(PIN, r, g, b)               \
    do {                                             \
        unsigned char _ws2812b_r_variable = r;       \
        unsigned char _ws2812b_g_variable = g;       \
        unsigned char _ws2812b_b_variable = b;       \
        ws2812b_send_byte(PIN, _ws2812b_g_variable); \
        ws2812b_send_byte(PIN, _ws2812b_r_variable); \
        ws2812b_send_byte(PIN, _ws2812b_b_variable); \
    } while (0)

#endif // WS2812B_H
