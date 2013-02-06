#include "delayer.h"
#include "led_debug.h"

#include <p30f2011.h> 
#include <libpic30.h>

_FOSC(CSW_FSCM_OFF & FRC_PLL16); // FCY=29.48MHz & FOSC=16x7.37MHz  
_FWDT(WDT_OFF);                  // Watchdog timer off 
_FBORPOR(MCLR_DIS & PWRT_64);    // Disable reset pin & Power-up timer

#include "lm16a211.h"

static struct tagRCONBITS RCONbits_saved;

static void init(void) {
    // Save RCON for later debugging.
    RCONbits_saved = RCONbits;
    RCON = 0;
}

int main(void) {
    init();

    // The pin setup has been placed in PRE_LCD_INIT, which is called from lcd_init().
    lcd_init(); 
    
#if !NO_DEBUG
    // Have had problems where __delay32 resets with IOPUWR. Show the bit.
    if (RCONbits_saved.IOPUWR) {
        DEBUG_BIT_1 = 1;
    } else {
        DEBUG_BIT_1 = 0;
    }
#endif


    lcd_send_str("0123456789ABCDEF");
    while (1) {
        int i;
        for (i = 0; i < 10; i++) {
            lcd_shift_display_right(1);
            delay_ms(500);
        }    
        for (i = 0; i < 10; i++) {
            lcd_shift_display_left(1);
            delay_ms(500);
        }
    }

    return 0; 
} 
