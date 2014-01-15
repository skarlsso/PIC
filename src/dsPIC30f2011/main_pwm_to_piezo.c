// Qick-n-dirty example to get some sound out of a
// piezo element with Pulse Width Modulation (PWM)
// ===============================================

#include <xc.h>

// Doc: xc16/v1.11/docs/config_docs/30F2011.html
#pragma config FCKSMEN=CSW_FSCM_OFF /* Clock switching and monitor off */
#pragma config FOSFPR=FRC_PLL16     /* FCY=29.48MHz & FOSC=16x7.37MHz */
#pragma config WDT=WDT_OFF          /* Wathdog timer off */
#pragma config MCLRE=MCLR_DIS       /* Disable reset pin */
#pragma config FPWRT=PWRT_16        /* Power-up timer */

#include "debug/led_debug.h"
#include "time/delayer.h"

// Frequencey C: 523.25 ~ 220
#define C 220
// Frequencey D: 587.33 ~ 198
#define D 198
// Frequencey E: 659.26 - 176
#define E 176
// Frequencey F: 698.46 ~ 166
#define F 166
// Frequencey G: 783.99 ~ 148
#define G 148

#define A 263;

static const int song_length = 14;
static int song[] = { C, D, E, C, C, D, E, C, E, F, G, E, F, G };

static int song_index = 0;

static int count = 0;

static int PR2_shadow = 16;
static int PR1_flip = 581;
static int PR1_note = 58280;

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
    //PR2 = 58280;
    PR1 = PR1_note;
    PR2 = PR2_shadow;

    // The PWM signal is output on the OC1 pin

	// Fixed high level length of the PWM pulse when tones are played.
    OC1RS = 128;
    
    // Set OC1R with the initial duty cycle value
    OC1R = OC1RS;

    // Timer2 is used to control the PWM period
    _T2IP = 1; // Interrput priority flag
    _T2IF = 0; // Interrupt flag
    _T2IE = 1; // Interrupt Enable

	// Timer1 is used to change notes.
    _T1IP = 1;
    _T1IF = 0;
    _T1IE = 1;

    // Choose PWM mode
    OC1CON = 0;
    OC1CONbits.OCTSEL = 0; // Use timer2
    OC1CONbits.OCM0   = 0; // No faulting on compare input pins
    OC1CONbits.OCM1   = 1;
    OC1CONbits.OCM2   = 1;

    // Set prescale value and enable timer
    T2CON = 0;
    T2CONbits.TCKPS = 3; // 1:256 prescaler
    T2CONbits.TCS   = 0; // Use internal timer
    T2CONbits.TON   = 1; // Start timer

    T1CON = 0;
    T1CONbits.TCKPS = 3;
    T1CONbits.TCS   = 0;
    T1CONbits.TON   = 1;

    while (1);


    return 0;
}

// Interrupt to change note by changing duty cycle.
void __attribute__((__interrupt__,__auto_psv__)) _T1Interrupt(void) {
    // Test that we get the interrupts.
    if (count == 0 || count > 10) {
        DEBUG_BIT_0 = 1 - DEBUG_BIT_0;
    }
    count++;

    if (PR1 == PR1_note) {
	    // Keep quite a while, before flipping to the next note.
        PR1 = PR1_flip;
		// Zeor duty cycle == silence
		OC1RS = 0;
    } else {
	    // Fixed length of the tones.
        PR1 = PR1_note;
        // Fixed high level length of the PWM puls.
        OC1RS = 128;

		// Step to next note, but post-pone the actual change until
		// the next PWM pulse is completed. This prevents glitches.
		PR2_shadow = song[song_index++];
		if (song_index >= song_length) {
		  song_index = 0;
		}
    }

    _T1IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(void) {
    // A PWM pulse is completed. Maybe switch PWM period
    // if the next note of the song should be played.
    PR2 = PR2_shadow;
    _T2IF = 0;
}
