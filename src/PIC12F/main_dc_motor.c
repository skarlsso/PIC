// Control a DC motor connected to a H-Bridge with a PIC
// =====================================================

// Done:
//  Step 1: Get PWM module to output a PWM signal.
//  Step 2: Use the ADC module to read the voltage comming out of a potentiometer.
//  Step 3: Control the duty cycle of the PWM with the input from the potentiometer.
//  Step 4: Use the PWM signal to control the speed of the motor connected to the H-Bridge.
//  Step 5: Make it possible to go both forward and backwards by splitting the ADC input into two parts.

// Reference:
//   http://www.mcmanis.com/chuck/robotics/tutorial/h-bridge/bjt-circuit.html
//     I built and used the H-Bridge circuit described on the page.
//     NOTE: Don't disconect any of the FWD and REV pins. That will cause
//           the two opto ouplers on one side of the H-Bridge to turn on the
//           BJTs and current will surge through and potentially break your
//           components.

// PIC12LF1822 Configuration Bit Settings

#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)


// Instruction time 125 ns when running at 32MHz, according to the manual.
#define delay_us() _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop()
#define delay_5us() delay_us(); delay_us(); delay_us(); delay_us(); delay_us()
#define delay_10us() delay_5us(); delay_5us()

// TAD (ADC period) == 1 us with 32 MHz and ADC Clock Selection Fosc/32
#define delay_10_5_TADs() delay_10us(); _nop(); _nop(); _nop(); _nop(); _nop(); _nop();

#define FORWARD_PIN LATAbits.LATA0
#define REVERSE_PIN LATAbits.LATA1

void go_forward() {
    FORWARD_PIN = 1;
    REVERSE_PIN = 0;
}

void go_reverse() {
    FORWARD_PIN = 0;
    REVERSE_PIN = 1;
}

int main(void) {

    // === Clock ===
    
    // Us Internal Oscillator (FRC)

    OSCCONbits.IRCF   = 0b1110; // 8 MHz
    OSCCONbits.SCS    =   0b00; // Use config bits above
    // OSCCONbits.SPLLEN = 0b1; // PLL x4, set in config bits above


    // === Pins ===

    // Forward pin
    TRISA0 = 0; // Output
    ANSA1  = 0; // Digital

    // Reverse pin
    TRISA1 = 0; // Output

    // TRISA2 - Enable/PWM pin is set below when PWM is setup.


    // === PWM ===

    //  <-- Period -------------------------------------->
    //        _____________
    // ______|             |_____________________________|
    // TMR2:  <-- 0         <-- CCPR1H:CCP1CON<5:4>TMR2   <-- PR2

    // Disable CCP1 pin while setting up the PWM module.
    TRISA2 = 1;

    // Load PR2

    // Example from the manual
    // PWM_Period = 1 / 1950
    // Fosc       = 32000000
    // Prescaler  = 16
    T2CONbits.T2CKPS = 0b10;
    // PWM_Period = (PR2 + 1) * 4 * Tosc * Prescaler
    // PR2 = PWM_Period / (4 * Tosc * Prescaler) - 1
    // Tosc = 1 / Fosc
    // PR2 = PWM_Period * Fosc / (4 * Prescaler) - 1
    // PR2 = PWM_Period * Fosc / (4 * 16) - 1
    // PR2 = PWM_Period * Fosc / 64 - 1
    // PR2 = PWM_Period * 32000000 / 64 - 1
    // PR2 = PWM_Period * 500000 - 1
    // PR2 = 1 / 1950 * 500000 - 1
    // PR2 ~= 0xFF;

    // I use one less than the max value (0xFF), so that I can manually snap it
    // to 0xFF and thereby get a 100% duty cycle.
    //
    // From the manual:
    //   If the pulse width value is greater than the
    //   period the assigned PWM pin(s) will
    //   remain unchanged
    PR2 = 0xFE;

    // Start with a 50% duty cycle.
    CCPR1L = 0x80; // 8 MSBs
    DC1B1 = 0;     // 2 LSBs

    // Use RA2 as CPP1.
    APFCONbits.CCP1SEL = 0;
 
    // Setup PWM mode - active low
    CCP1CONbits.CCP1M = 0b1111;

    // Clear timer interrupt flag.
    PIR1bits.TMR2IF = 0;

    // Start timer.
    T2CONbits.TMR2ON = 1;

    // Wait for the current time period to complete, before enabling the PWM.
    while (!PIR1bits.TMR2IF);
    
    // Enable the PWM on the CCP1 pin.
    TRISA2 = 0;


    // === ADC ===

    // Use Fosc/32 since it will make the ADC Workaround easier (because TAD == 1 us.)
    ADCON1bits.ADCS   = 0b10; // Fosc/32
    ADCON1bits.ADFM   =    0; // Left justified == 6 LSBs in ADRESL is 0.
    ADCON1bits.ADPREF =    0; // Vref == AVdd
    

    ANSELAbits.ANSA4 =    1; // Analog
    TRISAbits.TRISA4 =    1; // Input
    ADCON0bits.CHS   = 0b11; // Turn on AN3
    ADCON0bits.ADON  =    1; // Enable ADC module

    // Let the ADC capacitor charge up.
    delay_10us();

    while (1) {
        // Start AD conversion.
        ADCON0bits.ADGO = 1;
  
        // WORKAROUND 2.1:
        // http://ww1.microchip.com/downloads/en/DeviceDoc/80502C.pdf
        // Only for Device ID == 6.
        // Sometimes the ADC module doesn't finish by itself. Workaround method 2
        // is to wait 10.5 TAD and then manually stop the ADC.
        delay_10_5_TADs();
        ADGO = 0;
       
        unsigned char adc_high = ADRESH;      // Bits 9:2
        unsigned char adc_low  = ADRESL >> 6; // Bits 1:0

        // ADC value determins speed and direction.
        //    0: Max forward
        //  511: Min forward
        //  512: Zero point
        //  513: Min reverse
        // 1023: Max reverse

        // TODO: Add some dead band values around the Zero point.


        // Setup the PWM duty cycle

        unsigned char pwm_high;
        unsigned char pwm_low;

        if (adc_high < 0x80) {
            go_forward();
            // Use 9 bits of the ADC value, but invert it.
            pwm_high = 0x7F - adc_high;
            pwm_low  = 0b11 - adc_low;
        } else {
            go_reverse();
            // Use only the lower bits.
            pwm_high = adc_high & 0x7F;
            pwm_low  = adc_low;
        }

        // Shift to make it a 10 bit value.
        pwm_high = pwm_high << 1;
        pwm_low  = (pwm_low << 1) & 0b11;

        // Set the values to min and max when near the limits.
        // To make sure we reach 0% and 100% duty cycle.
#define EPSILON 1
        if (pwm_high < EPSILON) {
            pwm_high = 0;
            pwm_low  = 0;
        } else if (pwm_high > PR2 - EPSILON) {
            // Can only get 100% duty cycle if PR2 is less than the max value.
            pwm_high = (PR2 < 0xFF) ? (PR2 + 1) : 0xFF;
            pwm_low  = 0b11;
        }

        // Set the PWM duty cycle registers.
        CCPR1L = pwm_high;
        DC1B1  = pwm_low;
    }

    return 0;
}