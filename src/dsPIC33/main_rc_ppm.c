// Produces a six channel PPM signal that can be used to drive RC transmitters.
//
// The timing was found by probing the output of the Spektrum DX6i slave port.
//
// The signal can be passed along to a master RC controller, e.g. Spektrum DX8,
// through a trainer cable. It would also be possible to send the signal
// directly to a RF module.
// ===========================================================================

// DSPIC33FJ128GP802 Configuration Bit Settings

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

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)


// Hard-coded Fcy. See calculation below.
#define FCY 29840000
//#include "delayer.h"

// Setup the frequency.
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

void init_pins() {
    // Setup pins for the PPM output signal.

    // The PPM is normally high.
    LATAbits.LATA2   = 1;
    // Make RA2 an output pin.
    TRISAbits.TRISA2 = 0;
}

#define PPM_OUTPUT LATAbits.LATA2


// TODO: Tune times. Not really 22 ms.
//       Use external oscillator?

// Times derived from 22 ms == 0xA4000 = 671744 in TMR
// Low pulse duration
#define TMR_400us 0x2FB5
#define TMR_600us 0x478F
// Should be 1000us, but 1024 makes it easier to
// convert the 10 bit channel value to a TMR duration.
#define TMR_1024us 0x7763

// Value is 10 bit.
int channel_value_to_TMR(unsigned int value) {
    // The 10 bit value represent a value within [0, 1024) us.
    // Take care to not lose precision when ´
    return TMR_600us + ((value * (unsigned long) TMR_1024us) >> 10);
}

#define num_channels 6

typedef struct {
    unsigned int cached_TMR;
    unsigned int value : 10; // 10 bits per channel.
} channel;


// The last channel is a dummy channel to mark the end of the channel 6 pulse.
static channel channels[num_channels + 1];

#define INIT_STATE            0
#define channel_START_STATE    1
#define channel_DURATION_STATE 2

static int     PPM_state      = INIT_STATE;
//static channel* current_channel = &channels[0];
int current_channel_i = 0;

int main(void) {
    init_fcy();
    init_pins();

    OC1CONbits.OCM = 0b000; // Disable Output Compare Module
    OC1CONbits.OCTSEL =     0; // Timer2

    OC1CONbits.OCM    = 0b001; // Active-Low One-Shot

    // What values
    OC1R;
    OC1RS;

    // 32 bit timer [Timer3, Timer2]
    T2CONbits.TON   = 0; // Disable timer
    T2CONbits.TCS   = 0; // Internal Tosc/2
    T2CONbits.TGATE = 0;
    T2CONbits.TCKPS = 0; // Prescaler
    T2CONbits.T32   = 1;
    TMR3            = 0;
    TMR2            = 0;
    // Hand tuned to get a period of 22 ms.
    PR3             = 0xA;
    PR2             = 0x4000;
    // Setup interrupts.
    IPC2bits.T3IP   = 0x01;
    IFS0bits.T3IF   = 0;
    IEC0bits.T3IE   = 1;

    T4CONbits.TON   = 0; // Disable timer
    T4CONbits.TCS   = 0; // Internal Tosc/2
    T4CONbits.TGATE = 0;
    T4CONbits.TCKPS = 0; // Prescaler
    T4CONbits.T32   = 0;
    TMR4            = 0;
    // PR4 set during pulse train
    IPC6bits.T4IP   = 0x01;
    IFS1bits.T4IF   = 0;
    IEC1bits.T4IE   = 1;

    // Some test values.
    channels[0].value = 0;
    channels[1].value = 128;
    channels[2].value = 256;
    channels[3].value = 512;
    channels[4].value = 768;
    channels[5].value = 1023;
    channels[6].value = 1023; // What ever

    T2CONbits.TON = 1;

    while (1);

    return 0;
}

void output_PPM_pulse_step() {
    // State machine for channel pulse train.

    // Every channel value pulse starts with low for 400us and then
    // a high somewhere between 600us and 1600us depending on the channel value.

    switch (PPM_state) {
        case INIT_STATE:
            PPM_state = channel_START_STATE;

            PPM_OUTPUT = 0;
            PR4 = TMR_400us;
            break;

        case channel_START_STATE:
            PPM_state = channel_DURATION_STATE;

            PPM_OUTPUT = 1;
            PR4 = channel_value_to_TMR(channels[current_channel_i].value);
            break;

        case channel_DURATION_STATE:
            current_channel_i++;
            if (current_channel_i != num_channels + 1) {
                // More channels to handle.
                PPM_state = channel_START_STATE;

                PPM_OUTPUT = 0;
                PR4 = TMR_400us;
            } else {
                // Done with all channels.
                PPM_state = INIT_STATE;
                current_channel_i = 0;

                // Rest Timer4 until next pulse train.
                T4CONbits.TON = 0;
                TMR4 = 0;
            }
            break;

        default: // Error
            PPM_OUTPUT = 1;
            while (1) {
            PPM_OUTPUT = ~PPM_OUTPUT;
            Nop();Nop();Nop();Nop();Nop();
            }
            break;
    };
}

// Timer3 ISR
void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void) {
    // Start the PPM pulse train.
    output_PPM_pulse_step();
    T4CONbits.TON = 1;

    IFS0bits.T3IF = 0; // Clear Timer3 Interrupt Flag
}

// Timer4 ISR
void __attribute__((__interrupt__, no_auto_psv)) _T4Interrupt(void) {
    // Continue with the next step in the PPM pulse train.
    output_PPM_pulse_step();

    IFS1bits.T4IF = 0; // Clear Timer4 Interrupt Flag
}