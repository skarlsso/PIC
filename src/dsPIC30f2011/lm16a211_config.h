#ifndef LM16A211_CONFIG_H
#define LM16A211_CONFIG_H

// Example config file, based on my current setup
// ==============================================

#include <p30F2011.h>

// Turn RB0, RB1, RB2, RB3 into output pins.
#define PREPARE_FOR_READ_FROM_LCD \
    TRISB = 0b1111111100111111
    //TRISB |= 0b00001111

// Turn RBO, RB1, RB2, RB3 into input pins.
#define PREPARE_FOR_WRITE_TO_LCD \
    TRISB = 0b1111111100110000
    //TRISB &= 0b11110000

// Set RC13 into an input pin.
// FIXME: RC14, RC15 is hardcoded
#define PRE_LCD_INIT \
    /* Set input output bits. */ \
    TRISB = 0b1111111100110000; \
    TRISC = 0b0001111111111111; \
    /* Clear the output bits */ \
    LATB = 0; \
    LATC = 0; \
    /* Change to digital pins */ \
    ADPCFG = 0b11111111;

// PIC Output pins to the LCD, available after PREPARE_FOR_WRITE_TO_LCD.
#define LCD_E  LATBbits.LATB7
#define LCD_RS LATBbits.LATB6
#define LCD_RW LATCbits.LATC14
#define LCD_D4 LATBbits.LATB0
#define LCD_D5 LATBbits.LATB1
#define LCD_D6 LATBbits.LATB2
#define LCD_D7 LATBbits.LATB3

// PIC Input port to the LCD. Available after PREPARE_FOR_READ_FROM_LCD.
#define LCD_D7_WHEN_READABLE PORTBbits.RB3

#endif // LM16A211_CONFIG_H