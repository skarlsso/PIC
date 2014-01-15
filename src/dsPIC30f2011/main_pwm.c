// Example code for simple Pulse Width Modulation (PWM)
// ====================================================

#include <xc.h>

// Doc: xc16/v1.11/docs/config_docs/30F2011.html
#pragma config FCKSMEN=CSW_FSCM_OFF /* Clock switching and monitor off */
#pragma config FOSFPR=FRC_PLL16     /* FCY=29.48MHz & FOSC=16x7.37MHz */
#pragma config WDT=WDT_OFF          /* Wathdog timer off */
#pragma config MCLRE=MCLR_DIS       /* Disable reset pin */
#pragma config FPWRT=PWRT_16        /* Power-up timer */

#include "time/delayer.h"

int main(void) {
    debug_init();

    // 1) Set PR1 to choose PWM period
    //     PWM period = (PR1 + 1) * Tcy * (TMR1 prescale value)
    //     PR1 = PWM period / (Tcy * (TMR1 prescale value)) - 1
    //     PR1 = 1 / ((PWM frequency) * TCY * (TMR1 prescale value)) - 1
    //     PR1 = Fcy / ((PWM frequency) * (TMR1 prescale value)) - 1
    //      Fcy           = 29840000
    //      PWM frequency = 2
    //      TMR1          = 256
    //     PR1 = 29840000 / (2 * 256) - 1
    PR2 = 58280;

    // The PWM signal is output on the OC1 pin

    // 2) Set PWM duty cycle to 25% of the PWM period
    OC1RS = (PR2 >> 2);
    
    // 3) Set OC1R with the initial duty cycle value
    OC1R = OC1RS;

    // Timer2 is used to control the PWM period

    // 4) Enable Timer2 interrupts
    _T2IP = 1; // Interrput priority flag
    _T2IF = 0; // Interrupt flag
    _T2IE = 1;

    // 5) Choose PWM mode
    OC1CON = 0;
    OC1CONbits.OCTSEL = 0; // Use timer2
    OC1CONbits.OCM0   = 0; // No faulting on compare input pins
    OC1CONbits.OCM1   = 1;
    OC1CONbits.OCM2   = 1;

    // 6) Set prescale value and enable timer
    T2CON = 0;
    T2CONbits.TCKPS = 3; // 1:256 prescaler
    T2CONbits.TCS   = 0; // Use internal timer
    T2CONbits.TON   = 1; // Start timer

    while (1);

    return 0;
}

// The interrupts are not strictily necessary to get the PWM to work,
// but it helps during debugging.
void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(void) {
    // Test that we get the interrupts.
    DEBUG_BIT_0 = 1 - DEBUG_BIT_0;

    _T2IF = 0;
}