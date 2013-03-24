// Control a power outlet wireless remote control.
// The commands where decoded by main_remote_control_capture.c
// ===========================================================

#include "xc.h"

#include "delayer.h"
#include "led_debug.h"

#include <stdio.h>

// Doc: xc16/v1.11/docs/config_docs/30F2011.html
#pragma config FCKSMEN=CSW_FSCM_OFF /* Clock switching and monitor off */
#pragma config FOSFPR=FRC_PLL16     /* FCY=29.48MHz & FOSC=16x7.37MHz */
#pragma config WDT=WDT_OFF          /* Wathdog timer off */
#pragma config MCLRE=MCLR_DIS       /* Disable reset pin */
#pragma config FPWRT=PWRT_16        /* Power-up timer */

// Reverse engineered commands
// ===========================

#define BYTE(high, low) (((high) << 4) | ((low) & 0xF))
static char commands[2][4][3] = {
    //      <Constant                   >  <Device            >  <Off >
    {{ BYTE(0b0000, 0b0111), BYTE(0b0000, 0b1100), BYTE(0b1011, 0b1111) },
     { BYTE(0b0000, 0b0111), BYTE(0b0000, 0b1101), BYTE(0b0100, 0b1111) },
     { BYTE(0b0000, 0b0111), BYTE(0b0000, 0b1110), BYTE(0b0001, 0b1111) },
     { BYTE(0b0000, 0b0111), BYTE(0b0000, 0b1111), BYTE(0b1000, 0b1111) }},
    //      <Constant                   >  <Device            >  <On  >
    {{ BYTE(0b0000, 0b0111), BYTE(0b0000, 0b1100), BYTE(0b1011, 0b0000) },
     { BYTE(0b0000, 0b0111), BYTE(0b0000, 0b1101), BYTE(0b0100, 0b0000) },
     { BYTE(0b0000, 0b0111), BYTE(0b0000, 0b1110), BYTE(0b0001, 0b0000) },
     { BYTE(0b0000, 0b0111), BYTE(0b0000, 0b1111), BYTE(0b1000, 0b0000) }}
};

#define OUT_PORT_BIT LATDbits.LATD0
#define BIT_DURATION 0x44 /* With internal timer and 256 prescale*/

static int tmr2_done = 1;
#define WAIT_ON_TMR2() \
    tmr2_done = 0; \
    T2CONbits.TON   = 1; \
    while (!tmr2_done);

#define wait_bit_write_duration() WAIT_ON_TMR2()

static void write_bit(int myb) {
    if (myb) {
        OUT_PORT_BIT = 1;
        //printf("1");
    } else {
        OUT_PORT_BIT = 0;
        //printf("0");
    }
}
#define write_bit_and_wait(b) \
    write_bit(b); \
    wait_bit_write_duration()

#define send_raw_bit(b) \
    write_bit_and_wait(((b) & 1) == 0); \
    write_bit_and_wait(((b) & 1) == 1);

static void send_marshal_bits() {
    write_bit_and_wait(0);
    write_bit_and_wait(1);
    write_bit_and_wait(0);
}

static void send_preamble() {
    send_raw_bit(1);
    send_raw_bit(1);
    send_raw_bit(1);
    send_raw_bit(1);
    send_marshal_bits();
}

static void send_bit(char b) {
    send_raw_bit(b);
    send_marshal_bits();
}

static void send_byte(char value) {
    send_bit(value >> 7);
    send_bit(value >> 6);
    send_bit(value >> 5);
    send_bit(value >> 4);
    send_bit(value >> 3);
    send_bit(value >> 2);
    send_bit(value >> 1);
    send_bit(value >> 0);
}

static void send_command(const char command[3]) {
    send_preamble();
    send_byte(command[0]);
    send_byte(command[1]);
    send_byte(command[2]);
}

int main(void) {
    debug_init();

    PR2 = 0x44;
    T2CON = 0;
    T2CONbits.TCKPS = 3; // 1:256 prescaler
    T2CONbits.TCS   = 0; // Use internal timer

    _T2IF = 0;
    _T2IE = 1;

    TRISDbits.TRISD0 = 0;

        // Baud Rate Generator calculation
    // Baud_rate = Fcy / (16 * (BRG + 1))
    // BRG       = Fcy / (16 * Baud_rate) - 1
    // Fcy       = 29840000
    // Baud_rate = 9600
    // BRG = 29840000 / (16 * 9600) - 1
    // BRG ~= 193
#if 0
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
#endif

    while (1) {
        //delay_ms(500);
#if 0
        printf("--- Sending command ---\n");
        send_command(commands[1][0]);
        printf("\n");
        send_command(commands[1][1]);
        printf("\n");
        send_command(commands[1][2]);
        printf("\n");
        send_command(commands[1][3]);
        printf("\n");
        send_command(commands[0][0]);
        printf("\n");
        send_command(commands[0][1]);
        printf("\n");
        send_command(commands[0][2]);
        printf("\n");
        send_command(commands[0][3]);
        printf("\n");
        printf("--- Sending done ---\n");
        delay_ms(10000);
#endif
        int  i;
        DEBUG_BIT_0 = 1;
        DEBUG_BIT_1 = 0;
        for (i = 0; i < 5; i++) {
            send_command(commands[1][0]);
            delay_ms(1);
        }
        delay_ms(1000);
        DEBUG_BIT_0 = 0;
        DEBUG_BIT_1 = 1;
        for (i = 0; i < 5; i++) {
            send_command(commands[0][0]);
            delay_ms(1);
        }
        delay_ms(1000);
    }
    return 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(void) {
    //printf("Timedout\n");
    tmr2_done = 1;
    _T2IF = 0;
}
