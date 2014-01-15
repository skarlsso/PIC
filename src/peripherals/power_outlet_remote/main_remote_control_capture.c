// Fetches the binary commands of one of my power outlet wireless remote controls
// ==============================================================================

#include <xc.h>

// Doc: xc16/v1.11/docs/config_docs/30F2011.html
#pragma config FCKSMEN=CSW_FSCM_OFF /* Clock switching and monitor off */
#pragma config FOSFPR=FRC_PLL16     /* FCY=29.48MHz & FOSC=16x7.37MHz */
#pragma config WDT=WDT_OFF          /* Wathdog timer off */
#pragma config MCLRE=MCLR_DIS       /* Disable reset pin */
#pragma config FPWRT=PWRT_16        /* Power-up timer */


#include "time/delayer.h"

#include <stdio.h>


#define MAX_SAMPLES 128
static int samples[MAX_SAMPLES * 2];
static int next_sample = 0;
static int send_samples = 0;
static int first_sample_level = -1;

void print_sample(int i, int level, int msw, int lsw);

static char hex[] = "0123456789ABCDEF";
int main(void) {
    // Setup UART
    // ==========
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


    IC1CON = 0;
    IC1CONbits.ICTMR = 1; // Timer 2
    IC1CONbits.ICM   = 1; // Edge detection mode

    TRISDbits.TRISD0 = 1; // Input pin

    PR2 = 0xFFFF; // Free running timer

    T2CON = 0;
    T2CONbits.TCKPS = 3; // 1:256 prescaler
    T2CONbits.TCS   = 0; // Use internal timer
    T2CONbits.TON   = 1; // Start timer

    _IC1IF = 0;
    _IC1IE = 1;

    _T2IF = 0;
    _T2IE = 1;

    // Setup timer2

    while (1) {
        if (send_samples) {
            int i;
            int level = first_sample_level;
#if 0
            printf("--- Samples ---\n");
            for (i = 0; i < next_sample; i += 2) {
                print_sample(i, level, samples[i], samples[i + 1]);
                level = 1 - level;
            }
            printf("---------------\n");
            fflush(stdout);
#endif
            level = first_sample_level;
            for (i = 0; i < next_sample; i += 2) {
                int val = samples[i + 1];
                char str = level ? '1' : '0';
				// The approximate lengths were recorded by an earlier input capture.
                if (val >= 0x40 && val <= 0x48) {
                    printf("%c", str);
                } else if (val >= 0x84 && val <= 0x92) {
                    printf("%c", str);
                    printf("%c", str);
                } else {
                    printf("*");
                }
                level = 1 - level;
            }
            printf("\n");
            fflush(stdout);

            first_sample_level = -1;
            next_sample = 0;
            send_samples = 0;
        }
    }

    return 0;
}

void print_sample(int i, int level, int msw, int lsw) {
    printf("%2d %c 0x%04X%04X\n", i >> 1, (level ? '+': '-'), msw, lsw);
}

static unsigned int prev_value = 0;
static unsigned int timeout_count = 0;

void __attribute__((__interrupt__,__auto_psv__)) _IC1Interrupt(void) {
    unsigned int timeout_periods = timeout_count;
    unsigned int value = IC1BUF;

    if (send_samples) {
        _IC1IF = 0;
        return;
    }

    if (first_sample_level == -1) {
        first_sample_level = PORTDbits.RD0;
        timeout_count = 0;
        prev_value = value;
        _IC1IF = 0;
        return;
    }

    unsigned int fraction_of_timeout_periods;

    if (timeout_periods == 0) {
        fraction_of_timeout_periods = value - prev_value;
    } else {
        // |PPPPDDDDDDD|TTTTTTTTTTT|TTTTTTTTTTT|VVVV       |
        //
        // P = prev_value
        // T = timed out period
        // V = value
        // | = timeout delims

        if (prev_value < value) {
            timeout_periods += 1;
            fraction_of_timeout_periods = value - prev_value;
        } else {
            fraction_of_timeout_periods = 0xFFFF - prev_value + value;
        }
    }

    samples[next_sample] = timeout_periods;
    samples[next_sample + 1] = fraction_of_timeout_periods;
    next_sample += 2;
#if 0
    lcd_home();
    lcd_send_char('<');
    lcd_send_char((PORTDbits.RD0) ? '+' : '-');
    lcd_send_as_hex(timeout_periods);
    lcd_send_char(':');
    lcd_send_as_hex(fraction_of_timeout_periods);
    lcd_send_char('>');
#endif
    if (next_sample >= (MAX_SAMPLES << 1)) {
#if 0
        lcd_home();
        lcd_send_str("Samples done");
#endif
        send_samples = 1;
    }

    timeout_count = 0;
    prev_value = value;

    _IC1IF = 0;
}


void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(void) {
    if (!send_samples) {
        if (timeout_count < 0xFFFF) {
            ++timeout_count;
        }

        if (timeout_count > 0x8) {
            timeout_count = 0;
            send_samples = 1;
        }
    }

    _T2IF = 0;
}
