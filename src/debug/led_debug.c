#include "led_debug.h"
#include "delayer.h"

void debug_blink(char bit_num, unsigned long delay_on_ms, unsigned long delay_off_ms) {
    while (1) {
        if (bit_num == 0) {
            DEBUG_BIT_0 = 1;
            delay_ms(delay_on_ms);
            DEBUG_BIT_0 = 0;
            delay_ms(delay_off_ms);
        } else if(bit_num == 1) {
            DEBUG_BIT_1 = 1;
            delay_ms(delay_on_ms);
            DEBUG_BIT_1 = 0;
            delay_ms(delay_off_ms);
        } // else nothing
    }
}
