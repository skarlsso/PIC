
// Setup UART so that printf sends to the serial port
// ==================================================

#include "xc.h"

// Needed to provide FCY to delayer.h
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

void init_pins(void);
void init_uart(void);
void init_fcy(void);

void leds(int addr, int bits);

int main(void) {
    debug_init();
    init_fcy();
    init_pins();
    init_uart();

    delay_ms(100);


    printf("\n");
    printf("=== main_MCP23008.c ===\n");
    fflush(stdout);

    TRISBbits.TRISB8 = 0;
    TRISBbits.TRISB9 = 0;

    // Setup I2C
    // =========

    // Baud rate
    // FCY     = 29840000
    // FCL     = 400000
    // config2 = (FCY/FSCL - FCY/1111111) - 1;
    // config2 = (29840000/400000 - 29840000/1111111) - 1;
    // config2 = (74.6 - 26.85) - 1;  => 57
    // config2 = (298.4 - 26.85) - 1; => 270
    unsigned int config2 = 270;
    // I2C in 7 bit address mode
    unsigned int config1 = (I2C1_ON
            & I2C1_IDLE_CON
            & I2C1_CLK_HLD
            & I2C1_IPMI_DIS
            & I2C1_7BIT_ADD
            & I2C1_SLW_EN // ?
            & I2C1_SM_DIS
            & I2C1_GCALL_DIS
            & I2C1_STR_DIS // ?
            & I2C1_NACK
            & I2C1_ACK_DIS
            & I2C1_RCV_DIS
            & I2C1_STOP_DIS
            & I2C1_RESTART_DIS
            & I2C1_START_DIS);

    int addr = 0b0100000 << 1;

    //I2C1ADD = addr;

    // WORKAROUND
    // http://ww1.microchip.com/downloads/en/DeviceDoc/80470f.pdf
    // Errata: 10
    PORTBbits.RB9 = 0;
    delay_ms(1);
    PORTBbits.RB9 = 1;

    OpenI2C1(config1, config2);

    IdleI2C1();

    printf("I2C1BRG: %d\n", I2C1BRG);
    printf("I2C1ADD: %x\n", I2C1ADD);

    printf("=== main_MCP23008.c Start Config ===\n");
    fflush(stdout);

    // Configure in sequential memory pointer mode.
    i2c_start_and_wait();
    i2c_write_and_wait(addr | 0);
    i2c_write_and_wait(0); // Position at start of memory
    i2c_write_and_wait(0); // IODIR   - All output
    i2c_write_and_wait(0); // IPOL    - Same logic state
    i2c_write_and_wait(0); // GPINTEN - Disable interrupt-on-change
    i2c_write_and_wait(0); // DEFVAL  - Default all zero
    i2c_write_and_wait(0); // INTCON  - Not used in this demo
    i2c_write_and_wait(0); // IOCON   - Default configuration
    i2c_write_and_wait(0); // GPPU    - no internal weak pull-up resistors
    i2c_stop_and_wait();

    printf("=== main_MCP23008.c Done Config ===\n");

    delay_ms(1);

    printf("=== main_MCP23008.c Start Mode Switch ===\n");

    // Switch to byte addressing mode
    i2c_start_and_wait();
    i2c_write_and_wait(addr | 0);
    i2c_write_and_wait(0x5); // ICON
    i2c_write_and_wait(1 << 5); // IOCON.SEQOP = 1; Operate in byte mode
    i2c_stop_and_wait();

    printf("=== main_MCP23008.c Done Mode Switch ===\n");

    delay_ms(1);


    //i2c_start_and_wait();
#if 0
    i2c_restart_and_wait();
    i2c_write_and_wait(addr | 0);
    i2c_write_and_wait(0x09); // GPIO
    i2c_write_and_wait(0b10101010); // 4 outputs on, 4 ouputs off
    i2c_stop_and_wait();
#endif

    while (1) {
        int i;
        for (i = 0; i <= 255; i++) {
            leds(addr, i);
            delay_ms(100);
        }
    }

    printf("=== main_MCP23008.c Done Drive Output ===\n");

    CloseI2C1();

    while (1);

    return 0;
}

void leds(int addr, int bits) {
    i2c_restart_and_wait();
    i2c_write_and_wait(addr | 0);
    i2c_write_and_wait(0x09); // GPIO
    i2c_write_and_wait(bits); // 4 outputs on, 4 ouputs off
}

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
    // Unlock Pin Select
    OSCCON = 0x46;
    OSCCON = 0x47;
    OSCCONbits.IOLOCK = 0;

    // RP10 => UART1 RX
    RPINR18bits.U1RXR = 10;

    // RP11 => UART1 TX
    RPOR5bits.RP11R = 0b11; // UART1 TX

    // Lock Pin Select
    OSCCON = 0x46;
    OSCCON = 0x47;
    OSCCONbits.IOLOCK = 1;
}

void init_uart(void) {
    // Baud Rate Generator calculation
    // Baud_rate = Fcy / (16 * (BRG + 1))
    // BRG       = Fcy / (16 * Baud_rate) - 1
    // Fcy       = 29840000
    // Baud_rate = 9600
    // BRG = 29840000 / (16 * 9600) - 1
    // BRG ~= 149


    U1BRG = 193;

    U1MODE = 0;
    U1MODEbits.UARTEN = 1; // Enable UART
    U1MODEbits.UEN    = 0;
    U1MODEbits.PDSEL  = 0; // 8-bit data, no parity
    U1MODEbits.STSEL  = 0; // 1 stop bit
    U1MODEbits.ABAUD  = 0; // No auto baud
    U1MODEbits.LPBACK = 0; // No loopback mode
    U1STA             = 0;
    U1STAbits.UTXEN   = 1; // Enable transmit
}