// Setup UART so that printf sends to the serial port
// ==================================================

#include "xc.h"

#include "delayer.h"
#include "lm16a211.h"
#include "led_debug.h"

#include <stdio.h>

// Doc: xc16/v1.11/docs/config_docs/30F2011.html
#pragma config FCKSMEN=CSW_FSCM_OFF /* Clock switching and monitor off */
#pragma config FOSFPR=FRC_PLL16     /* FCY=29.48MHz & FOSC=16x7.37MHz */
#pragma config WDT=WDT_OFF          /* Wathdog timer off */
#pragma config MCLRE=MCLR_DIS       /* Disable reset pin */
#pragma config FPWRT=PWRT_16        /* Power-up timer */

int main(void) {

    debug_init();
    lcd_init();
    lcd_send_str("Hello");

    // Baud Rate Generator calculation
    // Baud_rate = Fcy / (16 * (BRG + 1))
    // BRG       = Fcy / (16 * Baud_rate) - 1
    // Fcy       = 29840000
    // Baud_rate = 9600
    // BRG = 29840000 / (16 * 9600) - 1
    // BRG ~= 193

    U1BRG = 193;

    U1MODE = 0;
    U1MODEbits.UARTEN = 1; // Enable UART
    U1MODEbits.ALTIO  = 1; // Use alternative UART pins on the PIC.
    U1MODEbits.PDSEL  = 0; // 8-bit data, no parity
    U1MODEbits.STSEL  = 0; // 1 stop bit
    U1MODEbits.ABAUD  = 0; // No auto baud
    U1MODEbits.LPBACK = 0; // No loopback mode
    U1STA = 0;
    U1STAbits.UTXEN   = 1; // Enable transmit

    while (1) {
        printf("Hello World!\n");
        fflush(stdout);
        delay_ms(100);
    }
    return 0;
}
