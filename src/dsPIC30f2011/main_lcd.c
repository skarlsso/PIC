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

    // Change to digital pins.
    //ADPCFG = 0b01101111;
    ADPCFG = 0b11111111;
    // TODO: Assumes pin setup.

    // Set as outputs: RB0, RB1, RB2, RB3, RB6, RB7
    //TRISB = 0b1111111100110000;
    TRISB = 0b1111111100110000;
    // Set as outputs: RC13, RC14, RC15 
    TRISC = 0b0001111111111111;

    // Clear the output bits.
    LATB = 0;
    LATC = 0;

#if !NO_DEBUG
    // Have had problems where __delay32 resets with IOPUWR. Show the bit.
    if (RCONbits_saved.IOPUWR) {
        LATCbits.LATC15 = 1;
    } else {
        LATCbits.LATC15 = 0;
    }
#endif
}

int main(void) {
    init();
    lcd_init(); 
    
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
