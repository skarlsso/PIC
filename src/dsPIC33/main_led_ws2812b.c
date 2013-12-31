// Make nine serially connected WS2812B RGB LEDs shift in a rainbow pattern
// ========================================================================

// Reference:
//
// http://www.mikrocontroller.net/attachment/180459/WS2812B_preliminary.pdf
//   WS2812B data sheet

#include "xc.h"

#define FCY 29840000
#include "delayer.h"
#include "led_debug.h"
#include "i2c_helper.h"

#include <stdio.h>

// DSPIC33FJ128MC802 Configuration Bit Settings

#include <xc.h>

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Mode (Internal Fast RC (FRC) w/ PLL)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

void init_fcy(void) {
    // Timing for Internal FRC /w PPL

    // Fin     : 7370000
    //  Fin    : FRC / FRCDIV
    //  FRC    : 7370000 // Internal Fast RC Oscillator: 7.37 MHz
    //  FRCDIV : 1       // Internal Fast RC Oscillator Postscaler bits
    //   Default: CLKDIV.FRCDIV = 1;

    // PPL calculation
    //  VCOin  :    Fin / N1 must be within 0.8 MHz and 8 MHz
    //  VCOout :  VCOin * M  must be within 100 MHz and 200 MHz
    //  Fosc   : VCOout / N2 must be within 12.5 MHz and 80 MHz
    // Final frequency
    //  Fcy    : Fosc / 2

    // N1 : 2
    //  Default: CLKDIVbits.PLLPRE = 0;

    // M : 64 (default: 50)
    PLLFBDbits.PLLDIV = 64; // Set to 64 to match Fcy of dsPIC32f2011 FRC /w 16 PLL

    // N2 : 4
    //  Default: CLKDIVbits.PLLPOST = 1;

    // Fcy = Fin / N1 * M / N2 / 2
    // Fcy = 737000 / 2 * 64 / 4 / 2
    // Fcy = 29840000

    // Processor Clock Reduction Select Bits
    //  Default: CLKDIV.DOZE = 0x3; // Fcy/8
}

void init_pins(void) {
    // Make output
    TRISBbits.TRISB15 = 0;
}

#define START_LEVEL_FOR_SET 0
#define END_LEVEL_FOR_SET   1

int main(void) {
    debug_init();
    init_fcy();
    init_pins();

    // Start level - inverted
    LATBbits.LATB15 = 1;

// Each bit is encoded as:
//  0: High(0.4 us +- 150 ns) then Low(0.85 us +- 150ns)
//  1: High(0.8 us +- 150 ns) then Low(0.45 us +- 150ns)
//
// The timing is very sensitive and jmps and shifts takes a long time.
// I had to rearrange the if statement and put the shifts in the "long"
// part of the bits. The Nop() lengths were timed with the help of an
// oscilloscop.

#define HIGH_LONG()  Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop()
#define HIGH_SHORT() Nop(); Nop(); Nop()
#define LOW_LONG()   Nop(); Nop(); Nop(); Nop(); Nop(); Nop()
#define LOW_SHORT()

#define INVERT_BOOL(x) ((x) = (x) == 0)

#define SEND_AND_SHIFT_BIT(value) \
    INVERT_BOOL(LATBbits.LATB15); \
    if (!(value & 0b10000000)) { \
        HIGH_SHORT(); \
        INVERT_BOOL(LATBbits.LATB15); \
        value <<= 1; \
        LOW_LONG(); \
    } else { \
        value <<= 1; \
        HIGH_LONG(); \
        INVERT_BOOL(LATBbits.LATB15); \
        LOW_SHORT(); \
    }
    

    
    // Get it to shine!

    // RGB values for the nine LEDs
    // plus three extra values to make shifting of the colors easier.
    char ga[9 + 3];
    char ra[9 + 3];
    char ba[9 + 3];

    // Intensity progression. Offset sine curve.
    // Use int instead of floats.
    // 1000 * (1 + sin(360 / 9 * i) when i = [0..8]
    unsigned int intensity_progression[9] = {
        1000,
        1642,
        1984,
        1866,
        1342,
        658,
        134,
        16,
        356
    };


#define INTENSITY_MAX 2000
#define AMPLITUDE 30

    // Add the intensity curve and let the intensity be shifted between the colors.
    int j;
    for (j = 0; j < 9; j++) {
        unsigned char intensity = (unsigned char) (intensity_progression[j] * AMPLITUDE) / INTENSITY_MAX;
        ga[(j + 0) % 9] = intensity;
        ra[(j + 3) % 9] = intensity;
        ba[(j + 6) % 9] = intensity;
    }

    while (1) {
        int i;

        // This part is not really timing sensitive,
        // since we want to wait a while between every color shift.

        // Shift one LED color at a time.
#define SHIFT 1

        // Shift the LED colors
        for (i = 8; i >= 0; i--) {
            ga[i + SHIFT] = ga[i];
            ra[i + SHIFT] = ra[i];
            ba[i + SHIFT] = ba[i];
        }
        // Take care of the colors that were shifted out of the 0..8 first position
        // and move them to the begining of the arrays.
        for (i = 0; i < SHIFT; i++) {
            ga[i] = ga[9 + i];
            ra[i] = ra[9 + i];
            ba[i] = ba[9 + i];
        }

        // Delay before updating the LED colors.
        delay_ms(100);

        // Send the colors to all nine LEDs.
        for (j = 0; j < 9; j++) {
            char g = ga[j];
            char r = ra[j];
            char b = ba[j];

            // Send and shift the MSB bit for green.
            SEND_AND_SHIFT_BIT(g);
            SEND_AND_SHIFT_BIT(g);
            SEND_AND_SHIFT_BIT(g);
            SEND_AND_SHIFT_BIT(g);
            SEND_AND_SHIFT_BIT(g);
            SEND_AND_SHIFT_BIT(g);
            SEND_AND_SHIFT_BIT(g);
            SEND_AND_SHIFT_BIT(g);

            // Send and shift the MSB bit for red.
            SEND_AND_SHIFT_BIT(r);
            SEND_AND_SHIFT_BIT(r);
            SEND_AND_SHIFT_BIT(r);
            SEND_AND_SHIFT_BIT(r);
            SEND_AND_SHIFT_BIT(r);
            SEND_AND_SHIFT_BIT(r);
            SEND_AND_SHIFT_BIT(r);
            SEND_AND_SHIFT_BIT(r);

            // Send and shift the MSB bit for blue.
            SEND_AND_SHIFT_BIT(b);
            SEND_AND_SHIFT_BIT(b);
            SEND_AND_SHIFT_BIT(b);
            SEND_AND_SHIFT_BIT(b);
            SEND_AND_SHIFT_BIT(b);
            SEND_AND_SHIFT_BIT(b);
            SEND_AND_SHIFT_BIT(b);
            SEND_AND_SHIFT_BIT(b);
        }
        // Must wait at least 50 us before trying to send new color bits.
        delay_ms(1);
    }

    while (1);

    return 0;
}

