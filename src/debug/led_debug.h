#ifndef LED_DEBUG_H
#define LED_DEBUG_H

// FIXME: Hard coded for now.
#define DEBUG_BIT_0 _LATC13
#define DEBUG_BIT_1 _LATC15

// Hang and blink a led.
void debug_blink(char bit_num, unsigned long delay_on_ms, unsigned long delay_off_ms);

#endif // LED_DEBUG_H