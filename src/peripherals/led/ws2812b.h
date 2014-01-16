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

typedef struct {
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RGB;

#define RGB_equals(value0, value1) \
    (value0.r == value1.r && value0.g == value1.g && value0.b == value1.b)

#define ws2812b_send_RGB(PIN, rgb) \
    ws2812b_send_rgb(PIN, rgb.r, rgb.g, rgb.b)

#define ws2812b_send_RGB16(PIN, rgb_array_16) \
do {                                          \
    ws2812b_send_RGB(PIN, rgb_array_16[0]);   \
    ws2812b_send_RGB(PIN, rgb_array_16[1]);   \
    ws2812b_send_RGB(PIN, rgb_array_16[2]);   \
    ws2812b_send_RGB(PIN, rgb_array_16[3]);   \
    ws2812b_send_RGB(PIN, rgb_array_16[4]);   \
    ws2812b_send_RGB(PIN, rgb_array_16[5]);   \
    ws2812b_send_RGB(PIN, rgb_array_16[6]);   \
    ws2812b_send_RGB(PIN, rgb_array_16[7]);   \
    ws2812b_send_RGB(PIN, rgb_array_16[8]);   \
    ws2812b_send_RGB(PIN, rgb_array_16[9]);   \
    ws2812b_send_RGB(PIN, rgb_array_16[10]);  \
    ws2812b_send_RGB(PIN, rgb_array_16[11]);  \
    ws2812b_send_RGB(PIN, rgb_array_16[12]);  \
    ws2812b_send_RGB(PIN, rgb_array_16[13]);  \
    ws2812b_send_RGB(PIN, rgb_array_16[14]);  \
    ws2812b_send_RGB(PIN, rgb_array_16[15]);  \
} while (0)

#endif // WS2812B_H
